"""Offline A/B west-branch geometry comparison on frozen train/validation only.

Nearest-rail distances are a *map geometry* diagnostic, not an odometry score.
Validation GNSS is read solely here, never in map generation or online ROS.
"""

from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "routes"))
from analyze_gnss import DATA, enu, extract, finite_positions  # noqa: E402
from map_builder import _nearest_indices  # noqa: E402
from route_map_io import RouteMap  # noqa: E402


def track_xy(bag: str) -> np.ndarray:
    path = next((DATA / bag).glob("*.db3"))
    _, _, records = extract(path)
    master = finite_positions(records["mf"])
    rover = finite_positions(records["rf"])
    if len(master) < 100 or len(rover) < 100:
        return np.empty((0, 2))
    master = master[np.argsort(master[:, 1], kind="stable")]
    rover = rover[np.argsort(rover[:, 1], kind="stable")]
    pm = enu(master[:, 2], master[:, 3], master[:, 4])
    pr = enu(rover[:, 2], rover[:, 3], rover[:, 4])
    ri, age = _nearest_indices(rover[:, 1], master[:, 1])
    sep = np.linalg.norm(pm[:, :2] - pr[ri, :2], axis=1)
    local = median_filter(pm, size=(9, 1), mode="nearest")
    good = ((age < .15) & (sep >= 9) & (sep <= 16) &
            (np.linalg.norm(pm - local, axis=1) < 3) &
            np.isfinite(pm).all(axis=1))
    return pm[good, :2][::10]


def main() -> None:
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text())
    summaries = {item["bag"]: item for item in json.loads(
        (ROOT / "analysis" / "routes" / "summaries.json").read_text())}
    assets = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets"
    maps = {branch: RouteMap.from_csv(assets / csv) for branch, csv in
            (("B", "route_map.csv"), ("A", "route_map_branch_a.csv"))}
    trees = {branch: cKDTree(np.array([row[1:3] for row in maps[branch].rows["out"]]))
             for branch in maps}
    results = []
    for split in ("train", "validation"):
        for bag in manifest["representatives"][split]:
            summary = summaries[bag]["master_path"]
            if (summary.get("net_m", 0) < 4400 or
                summary["end_enu"][0] > summary["start_enu"][0]):
                continue
            xy = track_xy(bag)
            if len(xy) < 200:
                continue
            terminal = xy[:, 0] < -4450
            if terminal.sum() < 20:
                continue
            xt = xy[terminal]
            dist = {branch: trees[branch].query(xt)[0] for branch in ("A", "B")}
            # The last observed terminal samples matter for truncated runs;
            # this is deliberately a geometric endpoint diagnostic only.
            last = min(20, len(xt))
            a = float(np.median(dist["A"][-last:]))
            b = float(np.median(dist["B"][-last:]))
            winner = "A" if b - a > 5 else "B" if a - b > 5 else "ambiguous"
            results.append({"bag": bag, "split": split, "clean_terminal_samples": len(xt),
                            "endpoint_x": float(xy[-1, 0]), "endpoint_y": float(xy[-1, 1]),
                            "median_A_m": float(np.median(dist["A"])),
                            "median_B_m": float(np.median(dist["B"])),
                            "last20_A_m": a, "last20_B_m": b, "branch": winner})
            print(split, bag, winner, f"A={a:.1f}m B={b:.1f}m", flush=True)
    report = {"method": "two-antenna-cleaned master GNSS to nearest mapped XY node; final 20 decimated terminal fixes",
              "terminal_selection": "ENU x < -4450m; out trips with net displacement >4400m",
              "branch_threshold_m": 5,
              "counts": {s: dict(Counter(r["branch"] for r in results if r["split"] == s))
                         for s in ("train", "validation")},
              "runs": results}
    output = ROOT / "tools" / "branch_geometry_compare.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["counts"], indent=2))
    print(output)


if __name__ == "__main__":
    main()
