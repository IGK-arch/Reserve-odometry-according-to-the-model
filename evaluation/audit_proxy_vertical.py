"""Re-score position proxy with antenna vertical-noise term removed.

The legacy proxy rotates the 3D master→rover GNSS vector into base_link, so
GNSS altitude disagreement changes estimated base z. Both antennas have the
same +3 m body height. This audit instead uses master altitude minus 3 m for
the reference; it isolates the noise question, though real vehicle pitch can
make a small vertical offset along the 9.873 m rigid lever arm. Production and
the existing position CSVs are untouched.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path

import numpy as np

import position_proxy as pp
from route_map_io import RouteMap


ROOT = pp.ROOT
TABLE = ROOT / "ros2_ws/src/tram_odometry/assets/drive_accel_table.csv"
INPUTS = {
    "validation": ROOT / "evaluation/validation_position_table.csv",
    "holdout": ROOT / "evaluation/holdout_position_deployed_proxy.csv",
}
ORIGINAL_CLEANER = pp.clean_reference_positions
ANTENNA_NOISE = {}


def flat_z_reference(bag):
    data = ORIGINAL_CLEANER(bag)
    if data is None:
        return None
    times, master, old_base, raw_times, raw_master = data
    false_vertical_term = old_base[:, 2] - (master[:, 2] - 3.0)
    ANTENNA_NOISE[bag] = {
        "n": len(false_vertical_term),
        "median_abs_antenna_vertical_term_m":
            float(np.median(abs(false_vertical_term))),
        "p95_abs_antenna_vertical_term_m":
            float(np.quantile(abs(false_vertical_term), .95)),
        "max_abs_antenna_vertical_term_m":
            float(np.max(abs(false_vertical_term))),
    }
    base = old_base.copy()
    base[:, 2] = master[:, 2] - 3.0
    return times, master, base, raw_times, raw_master


if __name__ == "__main__":
    pp.clean_reference_positions = flat_z_reference
    route = RouteMap.from_csv(pp.MAP)
    report = {"method": "Same production C++ estimator and master-GNSS map; replace only reference base_link z with master_z-3. Vertical pitch offset along 9.873m is ignored as an intentional diagnostic. Original CSVs untouched.", "splits": {}}
    for split, input_file in INPUTS.items():
        with input_file.open(newline="", encoding="utf-8") as f:
            old = list(csv.DictReader(f))
        results = []
        for row in old:
            bag = row["bag"]
            table = TABLE if bag.startswith("30618") else None
            new = pp.score_bag(bag, route, (ROOT / "evaluation/replay_cli.exe").resolve(), table)
            if new is None or int(row["matched"]) != new["matched"]:
                raise ValueError(f"Unexpected score mismatch {bag}")
            results.append({
                "bag": bag,
                "matched": new["matched"],
                "original_base_rmse_3d_m": float(row["base_rmse_3d_m"]),
                "corrected_base_rmse_3d_m": new["base_rmse_3d_m"],
                "rmse_change_m": new["base_rmse_3d_m"] - float(row["base_rmse_3d_m"]),
                "original_base_end_error_3d_m": float(row["base_end_error_3d_m"]),
                "corrected_base_end_error_3d_m": new["base_end_error_3d_m"],
                **ANTENNA_NOISE[bag],
            })
            print(split, bag, f"3D RMSE {row['base_rmse_3d_m']} -> {new['base_rmse_3d_m']:.6f}", flush=True)
        output_file = ROOT / f"evaluation/audit_position_vertical_{split}.csv"
        with output_file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0]))
            writer.writeheader()
            writer.writerows(results)
        by_vehicle = {}
        for vehicle in ("30618", "30639"):
            selected = [r for r in results if r["bag"].startswith(vehicle)]
            if not selected:
                continue
            by_vehicle[vehicle] = {
                "bags": len(selected),
                "median_original_rmse_m": statistics.median(r["original_base_rmse_3d_m"] for r in selected),
                "median_corrected_rmse_m": statistics.median(r["corrected_base_rmse_3d_m"] for r in selected),
                "median_rmse_change_m": statistics.median(r["rmse_change_m"] for r in selected),
                "max_abs_rmse_change_m": max(abs(r["rmse_change_m"]) for r in selected),
                "median_bag_p95_abs_antenna_vertical_term_m":
                    statistics.median(r["p95_abs_antenna_vertical_term_m"] for r in selected),
            }
        report["splits"][split] = {"by_vehicle": by_vehicle, "runs": results}
    target = ROOT / "evaluation/audit_position_vertical.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(target)
