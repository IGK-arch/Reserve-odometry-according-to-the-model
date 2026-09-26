"""Train-only check of master→rover vector against travel/map tangents.

Run: py -3.12 tools/antenna_orientation_audit.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "routes"))
from analyze_gnss import DATA, enu, extract, finite_positions  # noqa: E402
from map_builder import SOURCES  # noqa: E402
from route_map_io import RouteMap  # noqa: E402


def nearest_times(source: np.ndarray, query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    idx = np.searchsorted(source, query)
    idx = np.clip(idx, 1, len(source) - 1)
    idx -= (abs(source[idx - 1] - query) < abs(source[idx] - query)).astype(int)
    return idx, abs(source[idx] - query)


def summarize(value: np.ndarray) -> dict:
    return {"n": int(len(value)), "median": float(np.median(value)),
            "p01": float(np.quantile(value, .01)),
            "p05": float(np.quantile(value, .05)),
            "p95": float(np.quantile(value, .95))}


def main() -> None:
    route = RouteMap.from_csv(ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "route_map.csv")
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    train_members = {bag for group in manifest["groups"] if group["split"] == "train" for bag in group["bags"]}
    results = []
    for direction, bags in SOURCES.items():
        map_xy = np.asarray([row[1:3] for row in route.rows[direction]])
        map_tan = np.gradient(map_xy, axis=0)
        map_tan /= np.linalg.norm(map_tan, axis=1)[:, None]
        tree = cKDTree(map_xy)
        for bag in bags:
            assert bag in train_members
            _, _, records = extract(next((DATA / bag).glob("*.db3")))
            mf = finite_positions(records["mf"])
            rf = finite_positions(records["rf"])
            mv = records["mv"]
            mf = mf[np.argsort(mf[:, 1])]
            rf = rf[np.argsort(rf[:, 1])]
            mv = mv[np.argsort(mv[:, 1])]
            pm = enu(mf[:, 2], mf[:, 3], mf[:, 4])
            pr = enu(rf[:, 2], rf[:, 3], rf[:, 4])
            r_idx, r_age = nearest_times(rf[:, 1], mf[:, 1])
            v_idx, v_age = nearest_times(mv[:, 1], mf[:, 1])
            delta = pr[r_idx, :2] - pm[:, :2]
            sep = np.linalg.norm(delta, axis=1)
            vel = mv[v_idx, 2:4]
            speed = np.linalg.norm(vel, axis=1)
            good = (r_age < .06) & (v_age < .10) & (sep > 9) & (sep < 16) & (speed > 3) & (speed < 20)
            # Fixed stride bounds output and avoids overcounting adjacent 10Hz
            # measurements as independent evidence.
            selection = np.flatnonzero(good)[::10]
            u = delta[selection] / sep[selection, None]
            v = vel[selection] / speed[selection, None]
            _, map_idx = tree.query(pm[selection, :2])
            dot_velocity = np.sum(u * v, axis=1)
            dot_map = np.sum(u * map_tan[map_idx], axis=1)
            row = {"bag": bag, "direction": direction,
                   "master_to_rover_vs_velocity": summarize(dot_velocity),
                   "master_to_rover_vs_map_tangent": summarize(dot_map)}
            results.append(row)
            print(bag, direction, len(selection),
                  f"dot_velocity median={np.median(dot_velocity):.5f}",
                  f"dot_map median={np.median(dot_map):.5f}", flush=True)
    for direction in SOURCES:
        subset = [r for r in results if r["direction"] == direction]
        print(direction, "bag-median dot velocity", np.median([r["master_to_rover_vs_velocity"]["median"] for r in subset]),
              "bag-median dot map", np.median([r["master_to_rover_vs_map_tangent"]["median"] for r in subset]))
    output = ROOT / "tools" / "antenna_orientation_audit.json"
    output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
