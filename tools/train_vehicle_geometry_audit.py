"""Compare train-only GNSS tracks from vehicle 30639 with the shared route map.

This is a diagnostic for route/antenna geometry, not an online estimator.  The
held-out May session is deliberately never read by this program.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "routes"))
from analyze_gnss import DATA, enu, extract, finite_positions  # noqa: E402
from map_builder import _nearest_indices  # noqa: E402
from route_map_io import RouteMap  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", choices=("train", "holdout"), default="train")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text())
    names = sorted({bag for group in manifest["groups"] if group["split"] == args.split
                    for bag in group["bags"] if bag.startswith("30639_")})
    route = RouteMap.from_csv(ROOT / "ros2_ws" / "src" / "tram_odometry" /
                              "assets" / "route_map.csv")
    trees = {direction: cKDTree(np.array([row[1:3] for row in route.rows[direction]]))
             for direction in ("out", "return")}
    reports = []
    for name in names:
        path = next((DATA / name).glob("*.db3"))
        _, _, records = extract(path)
        master = finite_positions(records["mf"])
        rover = finite_positions(records["rf"])
        if len(master) < 3000 or len(rover) < 3000:
            continue
        master = master[np.argsort(master[:, 1], kind="stable")]
        rover = rover[np.argsort(rover[:, 1], kind="stable")]
        pm = enu(master[:, 2], master[:, 3], master[:, 4])
        pr = enu(rover[:, 2], rover[:, 3], rover[:, 4])
        ri, age = _nearest_indices(rover[:, 1], master[:, 1])
        sep = np.linalg.norm(pm[:, :2] - pr[ri, :2], axis=1)
        median = median_filter(pm, size=(9, 1), mode="nearest")
        valid = ((age <= .15) & (sep >= 9) & (sep <= 16) &
                 (np.linalg.norm(pm - median, axis=1) < 3) &
                 np.isfinite(pm).all(axis=1))
        xyz = pm[valid][::10]
        if len(xyz) < 200:
            continue
        direction = "out" if xyz[0, 0] > -200 else "return"
        d, ix = trees[direction].query(xyz[:, :2])
        map_z = np.array([route.rows[direction][i][3] for i in ix])
        body = xyz[:, 0] > -4400
        west = ~body
        def summary(mask: np.ndarray) -> dict:
            return {"n": int(mask.sum()), "xy_median_m": float(np.median(d[mask])),
                    "xy_p90_m": float(np.percentile(d[mask], 90)),
                    "xy_gt5_fraction": float(np.mean(d[mask] > 5)),
                    "z_minus_map_median_m": float(np.median(xyz[mask, 2] - map_z[mask]))} if mask.sum() else {}
        reports.append({"bag": name, "direction": direction, "clean_points": int(valid.sum()),
                        "start_enu": xyz[0].tolist(), "end_enu": xyz[-1].tolist(),
                        "all": summary(np.ones(len(xyz), dtype=bool)),
                        "body": summary(body), "west_terminal": summary(west)})
        print(name, direction, "body median/p90", summary(body).get("xy_median_m"),
              summary(body).get("xy_p90_m"), "west median/p90",
              summary(west).get("xy_median_m"), summary(west).get("xy_p90_m"), flush=True)
    out = ROOT / "tools" / f"{args.split}_vehicle_geometry_audit.json"
    out.write_text(json.dumps(reports, indent=2) + "\n")
    print(out)


if __name__ == "__main__":
    main()
