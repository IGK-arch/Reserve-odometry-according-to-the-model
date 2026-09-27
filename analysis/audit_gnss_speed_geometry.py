"""Compare GNSS velocity integral with train-map geometric progress by session.

Read-only holdout diagnostic. A shared speed bias in both GNSS receivers would
change map metres / GNSS-integrated metres on a long common route segment.
This cannot prove GNSS has no common error, but is independent of wheel speeds.
"""

from __future__ import annotations

import csv
import json
import statistics as stats
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from map_builder import load_clean_track  # noqa: E402
from route_map_io import RouteMap  # noqa: E402


def score_bag(bag, route):
    try:
        track = load_clean_track(bag)
    except (ValueError, StopIteration) as exc:
        return {"bag": bag, "error": str(exc)}
    direction = "out" if track.xyz[0, 0] > -200 else "return" if track.xyz[0, 0] < -4400 else None
    if direction is None:
        return {"bag": bag, "error": "mid-route start"}
    rows = np.asarray(route.rows[direction], dtype=float)
    tree = cKDTree(rows[:, 1:3])
    spacing = 5
    sample = track.xyz[::spacing]
    source_s = track.s[::spacing]
    distance, node = tree.query(sample[:, :2])
    map_s = rows[node, 0]
    common = (map_s >= 500) & (map_s <= 4000) & (distance < 15.0)
    if common.sum() < 100:
        return {"bag": bag, "error": "<100 matched central-route positions",
                "common_points": int(common.sum())}
    x = source_s[common]
    y = map_s[common]
    p = np.polyfit(x, y, 1)
    for _ in range(2):
        residual = y - np.polyval(p, x)
        keep = abs(residual) < 10.0
        if keep.sum() < 100:
            break
        p = np.polyfit(x[keep], y[keep], 1)
    return {"bag": bag, "direction": direction,
            "common_points": int(common.sum()),
            "central_nearest_xy_median_m": float(np.median(distance[common])),
            "map_m_per_gnss_speed_integral_m": float(p[0]),
            "fit_residual_abs_p90_m": float(np.quantile(abs(y - np.polyval(p, x)), .9)),
            "gnss_velocity_integral_full_m": float(track.velocity_integral_m)}


if __name__ == "__main__":
    with (ROOT / "analysis/catalog/bags.csv").open(newline="", encoding="utf-8") as f:
        metadata = {r["bag"]: r for r in csv.DictReader(f)}
    manifest = json.loads((ROOT / "tools/split_manifest.json").read_text(encoding="utf-8"))
    route = RouteMap.from_csv(ROOT / "ros2_ws/src/tram_odometry/assets/route_map.csv")
    cohorts = [
        ("30618_2026-07-27_train", "train", "30618", "2026-07-27"),
        ("30618_2026-09-03_train", "train", "30618", "2026-09-03"),
        ("30639_2026-08-26_train", "train", "30639", "2026-08-26"),
        ("30639_2026-05-05_holdout", "holdout", "30639", "2026-05-05"),
    ]
    report = {"method": "Nearest 2-m train-map node for two-antenna-cleaned master GNSS; fit central 500-4000m route map_s vs integral of master GNSS speed; holdout diagnostics only.", "cohorts": {}}
    for key, split, vehicle, date in cohorts:
        ids = [bag for bag in manifest["representatives"][split]
               if metadata[bag]["vehicle"] == vehicle and metadata[bag]["start_utc"].startswith(date)]
        rows = []
        for bag in ids:
            result = score_bag(bag, route)
            rows.append(result)
            print(key, bag, result.get("map_m_per_gnss_speed_integral_m", result.get("error")), flush=True)
        good = [r for r in rows if "map_m_per_gnss_speed_integral_m" in r]
        report["cohorts"][key] = {
            "bags": len(rows), "valid_bags": len(good),
            "median_bag_map_m_per_gnss_speed_integral_m":
                stats.median(r["map_m_per_gnss_speed_integral_m"] for r in good) if good else None,
            "median_bag_central_xy_m":
                stats.median(r["central_nearest_xy_median_m"] for r in good) if good else None,
            "runs": rows,
        }
    target = ROOT / "analysis/audit_gnss_speed_geometry.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(target)
