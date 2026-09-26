"""Local, read-only API for full-resolution tram rosbag playback.

Run from any directory with::

    py -3.12 analysis/simulator/serve_simulator.py --port 8999

Open http://127.0.0.1:8999/ . No ROS installation or internet is needed.
All times in the API are seconds relative to the earliest *vehicle message header*
in the selected bag. Bag reception times can differ from these header stamps.
GNSS is exposed to inspect the recording, not as an input to the estimator.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import gzip
import json
import math
import os
import re
import subprocess
import sys
import threading
from collections import OrderedDict
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "dataset" / "data"
STATIC = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "analysis" / "viewer"))
from rosbag_cdr import messages  # noqa: E402
sys.path.insert(0, str(ROOT / "tools"))
from route_map_io import (  # noqa: E402
    RouteMap, geodetic_to_enu, ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M,
)


TOPIC_KEYS = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
    "/vehicle/driver_position_cmd": "cmd",
    "/sensing/gnss/master/vel": "master",
    "/sensing/gnss/rover/vel": "rover",
    "/sensing/gnss/master/fix": "master_fix",
    "/sensing/gnss/rover/fix": "rover_fix",
}
VEHICLE_KEYS = ("front", "rear", "cmd")
SERIES_KEYS = ("front", "rear", "cmd", "master", "rover")
ROUTE_KEYS = ("master", "rover")
BAG_ID_RE = re.compile(r"^[0-9]{5}_[0-9a-f]{8}$")

# Hand-inspected episodes from analysis/signals and analysis/routes. These are
# visual bookmarks only. Some were identified with GNSS and must not be used as
# online estimator inputs or as automatic ground-truth labels.
BOOKMARKS: dict[str, list[tuple[float, float, str, str]]] = {
    "30618_33bec73f": [
        (109.57, 111.47, "rear_spin", "Буксование задней тележки"),
    ],
    "30618_2050d396": [
        (399.87, 404.27, "rear_slide", "Юз задней тележки при торможении"),
        (443.37, 444.37, "shared_slide_candidate", "Возможный юз обеих тележек"),
    ],
    "30639_50956d6e": [
        (96.98, 98.68, "front_spin", "Буксование передней тележки"),
    ],
    "30618_4d487b0d": [
        (101.47, 116.67, "gnss_master_freeze", "Замирание GNSS master"),
        (147.87, 158.87, "timing_skew", "Сдвиг времени колёсного сигнала около 0,93 с"),
    ],
    "30639_e4379d7f": [
        (198.44, 198.94, "gnss_rover_spike", "Выброс скорости GNSS rover"),
    ],
}

# The same fixed ENU origin used in analysis/viewer/build_explorer.py.
LAT0, LON0, H0 = 55.805, 37.425, 170.0
PHI0, LAMBDA0 = math.radians(LAT0), math.radians(LON0)
SIN_PHI0, COS_PHI0 = math.sin(PHI0), math.cos(PHI0)
SIN_LAMBDA0, COS_LAMBDA0 = math.sin(LAMBDA0), math.cos(LAMBDA0)
WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def _ecef(lat: float, lon: float, h: float) -> tuple[float, float, float]:
    phi, lam = math.radians(lat), math.radians(lon)
    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    sin_lam, cos_lam = math.sin(lam), math.cos(lam)
    radius = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_phi * sin_phi)
    return ((radius + h) * cos_phi * cos_lam,
            (radius + h) * cos_phi * sin_lam,
            (radius * (1 - WGS84_E2) + h) * sin_phi)


X0, Y0, Z0 = _ecef(LAT0, LON0, H0)
MAP_X0, MAP_Y0, MAP_Z0 = _ecef(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M)
MAP_PHI0, MAP_LAMBDA0 = math.radians(ORIGIN_LAT_DEG), math.radians(ORIGIN_LON_DEG)
MAP_SIN_PHI0, MAP_COS_PHI0 = math.sin(MAP_PHI0), math.cos(MAP_PHI0)
MAP_SIN_LAMBDA0, MAP_COS_LAMBDA0 = math.sin(MAP_LAMBDA0), math.cos(MAP_LAMBDA0)


def _map_enu_to_viewer_enu(east: float, north: float, up: float) -> tuple[float, float, float]:
    """Rigid ECEF change of ENU datum; no GNSS trajectory enters the map point."""
    x = MAP_X0 - MAP_SIN_LAMBDA0 * east - MAP_SIN_PHI0 * MAP_COS_LAMBDA0 * north + MAP_COS_PHI0 * MAP_COS_LAMBDA0 * up
    y = MAP_Y0 + MAP_COS_LAMBDA0 * east - MAP_SIN_PHI0 * MAP_SIN_LAMBDA0 * north + MAP_COS_PHI0 * MAP_SIN_LAMBDA0 * up
    z = MAP_Z0 + MAP_COS_PHI0 * north + MAP_SIN_PHI0 * up
    dx, dy, dz = x - X0, y - Y0, z - Z0
    return (-SIN_LAMBDA0 * dx + COS_LAMBDA0 * dy,
            -SIN_PHI0 * COS_LAMBDA0 * dx - SIN_PHI0 * SIN_LAMBDA0 * dy + COS_PHI0 * dz,
            COS_PHI0 * COS_LAMBDA0 * dx + COS_PHI0 * SIN_LAMBDA0 * dy + SIN_PHI0 * dz)


def _enu(lat: float, lon: float, h: float) -> tuple[float, float, float]:
    x, y, z = _ecef(lat, lon, h)
    dx, dy, dz = x - X0, y - Y0, z - Z0
    return (-SIN_LAMBDA0 * dx + COS_LAMBDA0 * dy,
            -SIN_PHI0 * COS_LAMBDA0 * dx - SIN_PHI0 * SIN_LAMBDA0 * dy + COS_PHI0 * dz,
            COS_PHI0 * COS_LAMBDA0 * dx + COS_PHI0 * SIN_LAMBDA0 * dy + SIN_PHI0 * dz)


def _rounded_time(stamp_ns: int, origin_ns: int) -> float:
    # Microsecond resolution preserves the recorded timing without enormous JSON.
    return round((stamp_ns - origin_ns) / 1e9, 6)


def _catalog() -> list[dict]:
    with (ROOT / "analysis" / "catalog" / "bags.csv").open(encoding="utf-8", newline="") as fh:
        source = list(csv.DictReader(fh))
    return [{
        "id": row["bag"],
        "vehicle": row["vehicle"],
        "duration_s": float(row["duration_s"]),
        "has_gnss": int(row["gnss_count"]) > 0,
        "counts": {
            "front": int(row["front_count"]),
            "rear": int(row["rear_count"]),
            "cmd": int(row["cmd_count"]),
            "master_fix": int(row["master_fix_count"]),
            "rover_fix": int(row["rover_fix_count"]),
            "gnss_total": int(row["gnss_count"]),
        },
        "start_utc": row["start_utc"],
    } for row in source]


CATALOG = _catalog()
META_BY_ID = {item["id"]: item for item in CATALOG}
CATALOG_JSON = json.dumps({"bags": CATALOG}, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def _gap_events(name: str, points: list[list[float]], threshold_s: float) -> list[dict]:
    events = []
    for before, after in zip(points, points[1:]):
        gap = after[0] - before[0]
        if gap > threshold_s:
            events.append({"t_s": before[0], "end_s": after[0], "type": "sensor_gap",
                           "channel": name,
                           "source": "gnss_audit" if name in ROUTE_KEYS else "inputs_only",
                           "label": f"Пропуск {name}: {gap:.1f} с"})
    return events


def _disagreement_events(front: list[list[float]], rear: list[list[float]]) -> list[dict]:
    """Find sustained wheel mismatch; it is evidence, not a slip verdict."""
    if not front or not rear:
        return []
    events = []
    j = 0
    open_start = None
    peak = 0.0
    prev_t = None

    def close(end_t: float | None) -> None:
        nonlocal open_start, peak
        if open_start is not None and end_t is not None and end_t - open_start >= 0.3:
            events.append({"t_s": open_start, "end_s": end_t,
                           "type": "bogie_disagreement",
                           "source": "inputs_only",
                           "peak_delta_mps": round(peak, 3),
                           "label": f"Расхождение тележек до {peak:.2f} м/с"})
        open_start, peak = None, 0.0

    for t, v in front:
        while j + 1 < len(rear) and rear[j + 1][0] <= t:
            j += 1
        k = j
        if j + 1 < len(rear) and abs(rear[j + 1][0] - t) < abs(rear[j][0] - t):
            k = j + 1
        if abs(rear[k][0] - t) > 0.25:
            close(prev_t)
            continue
        mismatch = abs(v - rear[k][1])
        if mismatch > 0.8:
            if open_start is None or (prev_t is not None and t - prev_t > 0.4):
                close(prev_t)
                open_start = t
            peak = max(peak, mismatch)
        else:
            close(prev_t)
        prev_t = t
    close(prev_t)
    return events


def _build_bag(bag_id: str) -> dict:
    if bag_id not in META_BY_ID or not BAG_ID_RE.fullmatch(bag_id):
        raise KeyError(bag_id)

    # Keep integer nanoseconds until the origin is known. Channel-local sorting
    # repairs occasional out-of-order header stamps without altering bag files.
    raw: dict[str, list[tuple[int, object]]] = {key: [] for key in TOPIC_KEYS.values()}
    for topic, _received_ns, msg in messages(DATA / bag_id, set(TOPIC_KEYS)):
        key = TOPIC_KEYS[topic]
        stamp = msg["stamp_ns"]
        if key in ("front", "rear"):
            val = msg["velocity_raw"] / 3.6  # This dataset stores km/h.
            if math.isfinite(val):
                raw[key].append((stamp, val))
        elif key == "cmd":
            raw[key].append((stamp, msg["position"]))
        elif key in ("master", "rover"):
            linear = msg["linear"]
            if all(math.isfinite(x) for x in linear):
                raw[key].append((stamp, math.sqrt(sum(x * x for x in linear))))
        else:
            lat, lon, h = msg["latitude"], msg["longitude"], msg["altitude"]
            if all(math.isfinite(x) for x in (lat, lon, h)) and -90 <= lat <= 90 and -180 <= lon <= 180:
                raw[key].append((stamp, (*_enu(lat, lon, h), msg["status"])))

    for items in raw.values():
        items.sort(key=lambda item: item[0])
    origin_candidates = [raw[key][0][0] for key in VEHICLE_KEYS if raw[key]]
    if not origin_candidates:
        raise ValueError(f"No vehicle messages in {bag_id}")
    origin_ns = min(origin_candidates)

    series = {key: [[_rounded_time(stamp, origin_ns), round(float(val), 5)]
                    for stamp, val in raw[key]] for key in SERIES_KEYS}
    route = {}
    for receiver in ROUTE_KEYS:
        route[receiver] = [[_rounded_time(stamp, origin_ns),
                            round(val[0], 3), round(val[1], 3), round(val[2], 3), val[3]]
                           for stamp, val in raw[f"{receiver}_fix"]]

    events = []
    for key in VEHICLE_KEYS:
        events.extend(_gap_events(key, series[key], 0.8))
    for key in ROUTE_KEYS:
        events.extend(_gap_events(key, series[key], 2.0))
    events.extend(_disagreement_events(series["front"], series["rear"]))
    for start, end, kind, label in BOOKMARKS.get(bag_id, []):
        events.append({"t_s": start, "end_s": end, "type": kind, "label": label,
                       "source": "manual_audit", "diagnostic_only": True})
    events.sort(key=lambda event: (event["t_s"], event["type"]))

    meta = META_BY_ID[bag_id]
    return {
        "id": bag_id,
        "vehicle": meta["vehicle"],
        "duration_s": meta["duration_s"],
        "origin_header_ns": origin_ns,
        "frame": "ENU",
        "enu_origin": {"latitude": LAT0, "longitude": LON0, "altitude_m": H0},
        "units": {"time": "s", "speed": "m/s", "position": "m", "cmd": "notch"},
        "counts": meta["counts"],
        "series": series,
        "route": route,
        "events": events,
    }


REPLAY_CLI: Path | None = None
REPLAY_ERROR = "Estimator replay is not configured"
REPLAY_AUTO_BUILT = False


def _configure_replay(explicit: Path | None) -> None:
    """Use a supplied executable or compile the current production core once."""
    global REPLAY_CLI, REPLAY_ERROR, REPLAY_AUTO_BUILT
    if explicit:
        executable = explicit.resolve()
        if not executable.is_file():
            REPLAY_ERROR = f"Replay executable does not exist: {executable}"
            return
        REPLAY_CLI = executable
        REPLAY_ERROR = ""
        return
    executable = STATIC / f"replay_cli_viewer_{os.getpid()}{'.exe' if sys.platform == 'win32' else ''}"
    package = ROOT / "ros2_ws" / "src" / "tram_odometry"
    command = ["g++", "-std=c++17", "-O2", "-I", str(package / "include"),
               str(package / "src" / "estimator.cpp"),
               str(ROOT / "evaluation" / "replay_cli.cpp"), "-o", str(executable)]
    try:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                check=True, timeout=120)
        REPLAY_CLI = executable
        REPLAY_AUTO_BUILT = True
        REPLAY_ERROR = ""
        if result.stderr:
            print(result.stderr, file=sys.stderr)
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        REPLAY_ERROR = f"Could not compile estimator replay: {detail.strip()}"
        print(REPLAY_ERROR, file=sys.stderr)


def _estimator_events(points: list[list[float]]) -> list[dict]:
    """Compress output flags into intervals for the existing event navigator."""
    events = []
    for column, label in ((6, "Передняя отвергнута оценивателем"),
                          (7, "Задняя отвергнута оценивателем"),
                          (8, "Оцениватель работает только по модели")):
        start = previous = None
        for point in points:
            t = point[0]
            active = bool(point[column])
            if active and (start is None or (previous is not None and t - previous > 0.35)):
                if start is not None and previous - start >= 0.25:
                    events.append({"t_s": start, "end_s": previous, "source": "estimator",
                                   "type": "estimator_flag", "label": label})
                start = t
            elif not active and start is not None:
                if previous - start >= 0.25:
                    events.append({"t_s": start, "end_s": previous, "source": "estimator",
                                   "type": "estimator_flag", "label": label})
                start = None
            previous = t
        if start is not None and previous is not None and previous - start >= 0.25:
            events.append({"t_s": start, "end_s": previous, "source": "estimator",
                           "type": "estimator_flag", "label": label})
    return events


def _attach_map_pose(bag_id: str, origin_ns: int, points: list[list[float]]) -> dict:
    """Show a base_link map projection only for an unambiguous master startup.

    This mirrors the node's default terminal-direction choice, 3-fix median,
    nearest-segment match, 100 m residual decay, rigid master->base_link TF,
    and ENU datum. GNSS is read *after* C++ replay and only for startup.
    Mid-route, missing-GNSS and invalid anchors deliberately have no marker.
    """
    map_path = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "route_map.csv"
    if not map_path.is_file() or not points:
        return {"available": False, "reason": "Карта маршрута недоступна"}
    route = RouteMap.from_csv(map_path)
    fixes: list[tuple[float, float, float, float, float]] = []
    stamp_times = [point[0] for point in points]
    for topic, _receive_ns, msg in messages(DATA / bag_id, {"/sensing/gnss/master/fix"}):
        stamp_ns = int(msg["stamp_ns"])
        relative_s = (stamp_ns - origin_ns) * 1e-9
        if relative_s < -0.5 or relative_s > 5.0 or msg["status"] < 0:
            continue
        lat, lon, alt = msg["latitude"], msg["longitude"], msg["altitude"]
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            continue
        if not math.isfinite(alt):
            alt = ORIGIN_ALT_M
        east, north, up = geodetic_to_enu(lat, lon, alt)
        if route.project(east, north).horizontal_distance_m > 50.0:
            continue
        i = bisect.bisect_right(stamp_times, relative_s) - 1
        if i >= 0:
            estimate = points[i]
            delta = max(-0.3, min(0.3, relative_s - estimate[0]))
            distance = estimate[2] + estimate[1] * delta
        else:
            distance = 0.0
        fixes.append((east, north, up, distance, relative_s))
        if len(fixes) == 3:
            break
    if len(fixes) < 3:
        return {"available": False, "reason": "Нет трёх корректных master GNSS fix в первые 5 с"}

    median = lambda column: sorted(fix[column] for fix in fixes)[len(fixes) // 2]
    initial = (median(0), median(1), median(2))
    anchor_distance_m = median(3)
    if initial[0] > -200.0:
        direction = "out"
    elif initial[0] < -4400.0:
        direction = "return"
    else:
        return {"available": False, "reason": "Старт вне конечных: направление неоднозначно"}
    match = route.project(initial[0], initial[1], direction)
    if match.horizontal_distance_m > 50.0:
        return {"available": False, "reason": "Стартовый GNSS дальше 50 м от выбранного пути"}
    map_initial = route.sample(direction, match.s)
    residual_x, residual_y = initial[0] - map_initial[0], initial[1] - map_initial[1]
    start_s_m = match.s - anchor_distance_m
    for point in points:
        progress = max(0.0, point[2] - anchor_distance_m)
        master_s_m = start_s_m + point[2]
        base_x, base_y, base_z = route.master_to_base_enu(direction, master_s_m,
                                                          forward_m=9.873,
                                                          antenna_height_m=3.0)
        weight = math.exp(-progress / 100.0)
        viewer_pose = _map_enu_to_viewer_enu(base_x + residual_x * weight,
                                             base_y + residual_y * weight, base_z)
        point.extend(round(value, 3) for value in viewer_pose)
    return {"available": True, "direction": direction,
            "start_s_m": round(start_s_m, 3),
            "anchor_lateral_m": round(match.horizontal_distance_m, 3),
            "fixes_used": 3,
            "frame": "viewer_ENU_base_link",
            "source": "train route map + startup master GNSS"}


def _build_estimate(bag_id: str) -> dict:
    if bag_id not in META_BY_ID or not BAG_ID_RE.fullmatch(bag_id):
        raise KeyError(bag_id)
    if REPLAY_CLI is None:
        raise RuntimeError(REPLAY_ERROR)
    # GNSS topics are deliberately absent. SQLite reader yields bag receive order.
    codes = {"/vehicle/front_bogie_velocity": "F",
             "/vehicle/rear_bogie_velocity": "R",
             "/vehicle/driver_position_cmd": "C"}
    input_rows: list[tuple[int, int, str, float]] = []
    for topic, receive_ns, msg in messages(DATA / bag_id, set(codes)):
        value = msg["position"] if codes[topic] == "C" else msg["velocity_raw"]
        if math.isfinite(float(value)):
            input_rows.append((receive_ns, int(msg["stamp_ns"]), codes[topic], float(value)))
    if not input_rows:
        raise ValueError("No estimator inputs in bag")
    origin_ns = min(stamp for _, stamp, _, _ in input_rows)
    input_csv = "".join(f"{receive},{stamp},{code},{value}\n"
                        for receive, stamp, code, value in input_rows)
    command = [str(REPLAY_CLI), bag_id.split("_", 1)[0]]
    table = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "drive_accel_table.csv"
    if bag_id.startswith("30618_") and table.is_file():
        command.append(str(table))  # Mirrors ROS node's default for this vehicle.
    result = subprocess.run(command, input=input_csv, text=True, capture_output=True,
                            check=True, timeout=180, cwd=ROOT)
    points = []
    table_active = False
    for row in csv.DictReader(result.stdout.splitlines()):
        values = [float(row[k]) for k in ("velocity_mps", "distance_m",
                    "model_accel_mps2", "front_weight", "rear_weight")]
        if not all(math.isfinite(value) for value in values):
            continue
        table_active = table_active or bool(int(row["drive_table_active"]))
        points.append([_rounded_time(int(row["stamp_ns"]), origin_ns),
                       *[round(value, 5) for value in values],
                       int(row["front_slip"]), int(row["rear_slip"]),
                       int(row["model_only"]), int(row["drive_table_used"])])
    # Event timestamps may occasionally move backward in header time; sort only
    # the display copy. The C++ replay above remains in unmodified receive order.
    points.sort(key=lambda row: row[0])
    map_pose = _attach_map_pose(bag_id, origin_ns, points)
    return {"id": bag_id, "origin_header_ns": origin_ns,
            "table_active": table_active, "series": points,
            "events": _estimator_events(points), "inputs": len(input_rows),
            "map_pose": map_pose}


class BagCache:
    """Keep at most two *compressed* bags so 122-bag browsing has bounded RAM."""

    def __init__(self, limit: int = 2, builder=_build_bag):
        self.limit = limit
        self.builder = builder
        self.lock = threading.Lock()
        self.entries: OrderedDict[str, bytes] = OrderedDict()

    def get(self, bag_id: str) -> bytes:
        with self.lock:
            if bag_id in self.entries:
                self.entries.move_to_end(bag_id)
                return self.entries[bag_id]
        payload = json.dumps(self.builder(bag_id), separators=(",", ":"),
                             ensure_ascii=False, allow_nan=False).encode("utf-8")
        compressed = gzip.compress(payload, compresslevel=4)
        with self.lock:
            self.entries[bag_id] = compressed
            self.entries.move_to_end(bag_id)
            while len(self.entries) > self.limit:
                self.entries.popitem(last=False)
        return compressed


CACHE = BagCache()
ESTIMATE_CACHE = BagCache(builder=_build_estimate)


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC), **kwargs)

    def do_GET(self) -> None:
        path = unquote(urlsplit(self.path).path)
        if path.startswith("/replay_cli_viewer_"):
            self.send_error(HTTPStatus.NOT_FOUND)
        elif path == "/api/catalog":
            self._json_response(CATALOG_JSON)
        elif path.startswith("/api/bag/"):
            bag_id = path.removeprefix("/api/bag/")
            if bag_id not in META_BY_ID or not BAG_ID_RE.fullmatch(bag_id):
                self.send_error(HTTPStatus.NOT_FOUND, "Unknown bag")
                return
            try:
                payload = CACHE.get(bag_id)
                self._json_response(payload, compressed=True)
            except (OSError, ValueError, OverflowError) as exc:
                self.log_error("Failed to read bag %s: %s", bag_id, exc)
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Could not read bag")
        elif path.startswith("/api/estimate/"):
            bag_id = path.removeprefix("/api/estimate/")
            if bag_id not in META_BY_ID or not BAG_ID_RE.fullmatch(bag_id):
                self.send_error(HTTPStatus.NOT_FOUND, "Unknown bag")
                return
            if REPLAY_CLI is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, REPLAY_ERROR)
                return
            try:
                payload = ESTIMATE_CACHE.get(bag_id)
                self._json_response(payload, compressed=True)
            except (OSError, ValueError, OverflowError, RuntimeError,
                    subprocess.SubprocessError) as exc:
                self.log_error("Estimator replay failed for %s: %s", bag_id, exc)
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Estimator replay failed")
        elif path == "/api/health":
            self._json_response(b'{"ok":true}')
        else:
            super().do_GET()

    def _json_response(self, payload: bytes, *, compressed: bool = False) -> None:
        accept_gzip = "gzip" in self.headers.get("Accept-Encoding", "").lower()
        if compressed and not accept_gzip:
            payload = gzip.decompress(payload)
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        if compressed and accept_gzip:
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="Listen interface (default: localhost only)")
    parser.add_argument("--port", type=int, default=8999)
    parser.add_argument("--replay-cli", type=Path,
                        help="Optional prebuilt production replay_cli; otherwise compile C++ at startup")
    args = parser.parse_args()
    _configure_replay(args.replay_cli)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Tram simulator data: http://{args.host}:{args.port}/", flush=True)
    print(f"{len(CATALOG)} bags; API /api/catalog and /api/bag/<id>", flush=True)
    print(f"Estimator replay: {REPLAY_CLI or REPLAY_ERROR}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if REPLAY_AUTO_BUILT and REPLAY_CLI is not None:
            REPLAY_CLI.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
