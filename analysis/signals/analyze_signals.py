"""Read hackathon rosbag2 SQLite/CDR data without a ROS installation.

Only writes derived CSVs under this script's directory. The input bag files are
read only. Run: py -3.12 analysis/signals/analyze_signals.py
"""

from __future__ import annotations

import csv
import math
import sqlite3
import struct
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "dataset" / "data"
OUT = Path(__file__).resolve().parent


def cdr_header(data: bytes) -> tuple[int, int]:
    sec, nsec, length = struct.unpack_from("<iII", data, 4)
    return sec * 1_000_000_000 + nsec, 16 + length


def aligned(offset: int, n: int) -> int:
    return 4 + ((offset - 4 + n - 1) // n) * n


def decode(data: bytes, kind: str) -> tuple[int, float]:
    stamp, offset = cdr_header(data)
    if kind == "tram_vehicle_msgs/msg/VelocitySensor":
        # The dataset's wheel values empirically track GNSS speed at a 3.6:1
        # ratio, despite the README claiming m/s. Convert observed km/h to m/s.
        return stamp, struct.unpack_from("<d", data, aligned(offset, 8))[0] / 3.6
    if kind == "tram_vehicle_msgs/msg/DriverControllerCommand":
        return stamp, float(struct.unpack_from("<b", data, offset)[0])
    if kind == "geometry_msgs/msg/TwistStamped":
        xyz = struct.unpack_from("<3d", data, aligned(offset, 8))
        return stamp, math.sqrt(sum(x * x for x in xyz))
    raise ValueError(kind)


def read_bag(path: Path) -> dict[str, np.ndarray]:
    conn = sqlite3.connect(f"file:{path.as_posix()}?mode=ro", uri=True)
    try:
        topics = {
            row[0]: (row[1], row[2])
            for row in conn.execute("SELECT id, name, type FROM topics")
            if row[1] in {
                "/vehicle/front_bogie_velocity",
                "/vehicle/rear_bogie_velocity",
                "/vehicle/driver_position_cmd",
                "/sensing/gnss/master/vel",
                "/sensing/gnss/rover/vel",
            }
        }
        raw: dict[str, list[tuple[int, int, float]]] = defaultdict(list)
        ids = ",".join(str(i) for i in topics)
        for topic_id, recv, data in conn.execute(
            f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({ids}) "
            "ORDER BY timestamp"
        ):
            name, kind = topics[topic_id]
            stamp, val = decode(data, kind)
            raw[name].append((stamp, recv, val))
        arrs: dict[str, np.ndarray] = {}
        for name, triples in raw.items():
            arr = np.array(triples, dtype=np.float64)
            arr = arr[np.argsort(arr[:, 0], kind="stable")]
            arrs[name] = arr
        return arrs
    finally:
        conn.close()


def nearest(base_t: np.ndarray, arr: np.ndarray, max_dt: float = 0.15) -> tuple[np.ndarray, np.ndarray]:
    if len(arr) == 0:
        return np.full(len(base_t), np.nan), np.full(len(base_t), np.nan)
    t = arr[:, 0] / 1e9
    pos = np.searchsorted(t, base_t)
    left = np.clip(pos - 1, 0, len(t) - 1)
    right = np.clip(pos, 0, len(t) - 1)
    idx = np.where(abs(t[left] - base_t) <= abs(t[right] - base_t), left, right)
    dt = t[idx] - base_t
    val = arr[idx, 2].copy()
    val[abs(dt) > max_dt] = np.nan
    return val, dt


def latest(base_t: np.ndarray, arr: np.ndarray, max_age: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    if len(arr) == 0:
        return np.full(len(base_t), np.nan), np.full(len(base_t), np.nan)
    t = arr[:, 0] / 1e9
    idx = np.searchsorted(t, base_t, side="right") - 1
    valid = idx >= 0
    idx = np.clip(idx, 0, len(t) - 1)
    age = base_t - t[idx]
    valid &= age <= max_age
    val = arr[idx, 2].copy()
    val[~valid] = np.nan
    age[~valid] = np.nan
    return val, age


def quantile(values: np.ndarray, q: float) -> float:
    x = values[np.isfinite(values)]
    return float(np.quantile(x, q)) if len(x) else float("nan")


def rate(arr: np.ndarray) -> float:
    return float(len(arr) / (arr[-1, 0] - arr[0, 0]) * 1e9) if len(arr) > 1 else float("nan")


def max_gap(arr: np.ndarray) -> float:
    return float(np.max(np.diff(arr[:, 0])) / 1e9) if len(arr) > 1 else float("nan")


def max_value(arr: np.ndarray) -> float:
    return float(np.nanmax(arr[:, 2])) if len(arr) else float("nan")


def rmse(error: np.ndarray) -> float:
    x = error[np.isfinite(error)]
    return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")


def intervals(mask: np.ndarray, time: np.ndarray, max_gap: float = 0.3) -> list[tuple[int, int]]:
    index = np.flatnonzero(mask)
    if not len(index):
        return []
    splits = np.flatnonzero(np.diff(time[index]) > max_gap)
    groups = np.split(index, splits + 1)
    return [(int(g[0]), int(g[-1])) for g in groups if time[g[-1]] - time[g[0]] >= 0.4]


def main() -> None:
    summaries: list[dict] = []
    events: list[dict] = []
    notch_rows: list[dict] = []
    for n, path in enumerate(sorted(DATA.glob("*/*.db3")), 1):
        bag = path.parent.name
        train = bag.split("_")[0]
        arr = read_bag(path)
        front = arr["/vehicle/front_bogie_velocity"]
        rear = arr["/vehicle/rear_bogie_velocity"]
        cmd = arr["/vehicle/driver_position_cmd"]
        master = arr.get("/sensing/gnss/master/vel", np.empty((0, 3)))
        rover = arr.get("/sensing/gnss/rover/vel", np.empty((0, 3)))
        t = (master if len(master) else front)[:, 0] / 1e9
        origin = float(min(x[0, 0] for x in arr.values() if len(x))) / 1e9
        f, f_dt = nearest(t, front)
        r, r_dt = nearest(t, rear)
        c, c_age = latest(t, cmd)
        rv, rv_dt = nearest(t, rover)
        g = master[:, 2] if len(master) else np.full(len(t), np.nan)
        pair = np.isfinite(f) & np.isfinite(r) & np.isfinite(g)
        avg = (f + r) / 2
        mismatch = f - r
        error = avg - g
        valid_g = np.isfinite(g) & (g >= 0) & (g < 35)
        pair &= valid_g
        row = {
            "bag": bag,
            "train": train,
            "duration_s": round(float(max(t) - min(t)), 3),
            "n_front": len(front), "n_rear": len(rear), "n_cmd": len(cmd),
            "n_master": len(master), "n_rover": len(rover),
            "front_hz": rate(front),
            "rear_hz": rate(rear),
            "cmd_hz": rate(cmd),
            "master_hz": rate(master),
            "front_gap_p99_s": quantile(np.diff(front[:, 0]) / 1e9, .99),
            "rear_gap_p99_s": quantile(np.diff(rear[:, 0]) / 1e9, .99),
            "cmd_gap_p99_s": quantile(np.diff(cmd[:, 0]) / 1e9, .99),
            "front_gap_max_s": max_gap(front),
            "rear_gap_max_s": max_gap(rear),
            "cmd_gap_max_s": max_gap(cmd),
            "front_recv_age_median_s": quantile((front[:, 1] - front[:, 0]) / 1e9, .5),
            "rear_recv_age_median_s": quantile((rear[:, 1] - rear[:, 0]) / 1e9, .5),
            "cmd_recv_age_median_s": quantile((cmd[:, 1] - cmd[:, 0]) / 1e9, .5),
            "master_recv_age_median_s": quantile((master[:, 1] - master[:, 0]) / 1e9, .5),
            "front_recv_age_p99_s": quantile((front[:, 1] - front[:, 0]) / 1e9, .99),
            "rear_recv_age_p99_s": quantile((rear[:, 1] - rear[:, 0]) / 1e9, .99),
            "cmd_recv_age_p99_s": quantile((cmd[:, 1] - cmd[:, 0]) / 1e9, .99),
            "master_recv_age_p99_s": quantile((master[:, 1] - master[:, 0]) / 1e9, .99),
            "pair_coverage": float(np.mean(pair)),
            "speed_max_gnss": max_value(master),
            "speed_max_front": max_value(front),
            "speed_max_rear": max_value(rear),
            "mismatch_p50_mps": quantile(abs(mismatch[pair]), .5),
            "mismatch_p95_mps": quantile(abs(mismatch[pair]), .95),
            "mismatch_p99_mps": quantile(abs(mismatch[pair]), .99),
            "mismatch_gt_0p5_fraction": float(np.mean(abs(mismatch[pair]) > .5)) if np.any(pair) else np.nan,
            "avg_gnss_rmse": rmse(error[pair]),
            "front_gnss_rmse": rmse((f - g)[pair]),
            "rear_gnss_rmse": rmse((r - g)[pair]),
            "avg_gnss_bias": float(np.nanmean(error[pair])) if np.any(pair) else np.nan,
            "rover_master_rmse": rmse(rv - g),
            "rover_master_p95_abs": quantile(abs(rv - g), .95),
            "rover_master_gt_0p5_fraction": float(np.nanmean(abs(rv - g) > .5)),
            "cmd_min": float(np.nanmin(cmd[:, 2])), "cmd_max": float(np.nanmax(cmd[:, 2])),
            "cmd_nonzero_fraction": float(np.mean(cmd[:, 2] != 0)),
            "front_negative_fraction": float(np.mean(front[:, 2] < -0.05)),
            "rear_negative_fraction": float(np.mean(rear[:, 2] < -0.05)),
        }
        summaries.append(row)

        # Event windows show both axle disagreement and shared slip against GNSS.
        conditions = {
            "axle_disagreement": pair & (abs(mismatch) > .5),
            "wheel_vs_gnss": pair & (abs(error) > .7),
            "shared_wheel_error": pair & (abs(error) > .7) & (abs(mismatch) < .25),
        }
        for label, mask in conditions.items():
            for a, b in intervals(mask, t):
                sel = slice(a, b + 1)
                events.append({
                    "bag": bag, "train": train, "kind": label,
                    "start_s": round(float(t[a] - origin), 3),
                    "end_s": round(float(t[b] - origin), 3),
                    "duration_s": round(float(t[b] - t[a]), 3),
                    "n": b - a + 1,
                    "mean_gnss_mps": float(np.nanmean(g[sel])),
                    "mean_front_mps": float(np.nanmean(f[sel])),
                    "mean_rear_mps": float(np.nanmean(r[sel])),
                    "mean_cmd": float(np.nanmean(c[sel])),
                    "peak_abs_diff_mps": float(np.nanmax(abs(mismatch[sel]))),
                    "peak_abs_gnss_error_mps": float(np.nanmax(abs(error[sel]))),
                })

        # Approximately 1.1 second centered derivative after 0.1 s regularization.
        if len(master) >= 20:
            dt = .1
            grid = np.arange(t[0], t[-1], dt)
            if len(grid) >= 21:
                gi = np.interp(grid, t, g)
                smooth = savgol_filter(gi, 11, 2)
                accel = np.gradient(smooth, dt)
                cg, _ = latest(grid, cmd)
                fg, _ = nearest(grid, front)
                rg, _ = nearest(grid, rear)
                clean = np.isfinite(cg) & np.isfinite(fg) & np.isfinite(rg) & (gi > .5)
                clean &= (abs(fg - rg) < .3) & (abs((fg + rg) / 2 - gi) < .5)
                for notch in range(-15, 16):
                    mask = clean & (cg == notch)
                    if np.count_nonzero(mask) >= 30:
                        notch_rows.append({
                            "bag": bag, "train": train, "notch": notch,
                            "n": int(np.count_nonzero(mask)),
                            "accel_median": float(np.median(accel[mask])),
                            "accel_mean": float(np.mean(accel[mask])),
                            "speed_median": float(np.median(gi[mask])),
                        })
        if n % 10 == 0 or n == 1:
            print(f"{n}/{len(list(DATA.glob('*/*.db3')))} {bag}", flush=True)

    with (OUT / "bag_summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader(); writer.writerows(summaries)
    with (OUT / "anomaly_events.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(events[0]))
        writer.writeheader(); writer.writerows(events)
    with (OUT / "notch_response.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(notch_rows[0]) if notch_rows else ["bag", "train", "notch", "n", "accel_median", "accel_mean", "speed_median"])
        writer.writeheader(); writer.writerows(notch_rows)
    print(f"Wrote {len(summaries)} summaries, {len(events)} events, {len(notch_rows)} notch rows")


if __name__ == "__main__":
    main()
