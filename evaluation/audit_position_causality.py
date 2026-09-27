"""Audit startup lookahead in the offline position proxy; never change runtime.

The existing position_proxy scores all estimator stamps after anchoring itself
from three startup GNSS fixes. Production ROS withholds position until that
anchor exists. This script removes pre-third-fix samples from the published
proxy summary mathematically and reports the resulting RMSE. The third fix is
ordered by sensor stamp, matching position_proxy; actual DDS arrival can differ.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path

import numpy as np

from causal_baseline import read_run
from core_benchmark import replay_cpp
from position_proxy import MAP, ROOT, clean_reference_positions, nearest_ref
from route_map_io import RouteMap


TABLE = ROOT / "ros2_ws/src/tram_odometry/assets/drive_accel_table.csv"
INPUTS = (
    ROOT / "evaluation/validation_position_table.csv",
    ROOT / "evaluation/holdout_position_deployed_proxy.csv",
)


def score_early(row: dict[str, str], route: RouteMap) -> dict:
    bag = row["bag"]
    events, _ = read_run(bag)
    reference = clean_reference_positions(bag)
    if reference is None:
        raise ValueError(f"reference disappeared: {bag}")
    ref_times, ref_xyz, ref_base_xyz, raw_times, raw_xyz = reference
    t0 = min(stamp for _, stamp, _, _ in events) * 1e-9
    startup = np.flatnonzero((raw_times >= t0) & (raw_times <= t0 + 5.0))
    if len(startup) < 3:
        raise ValueError(f"third startup fix disappeared: {bag}")
    third_fix_stamp_s = float(raw_times[startup[2]])
    initial = np.median(raw_xyz[startup[:3]], axis=0)
    direction = row["direction"]
    anchor = route.project(float(initial[0]), float(initial[1]), direction)
    residual = initial - np.asarray(route.sample(direction, anchor.s))
    table = TABLE if bag.startswith("30618") else None
    core = replay_cpp((ROOT / "evaluation/replay_cli.exe").resolve(), bag, events, table)
    first_s = core[min(core)]["distance_m"]
    early_master_sq = early_base_sq = 0.0
    early_matched = early_outputs = 0
    for stamp in sorted(core):
        stamp_s = stamp * 1e-9
        if stamp_s >= third_fix_stamp_s:
            break
        early_outputs += 1
        ref = nearest_ref(ref_times, ref_xyz, stamp_s)
        ref_base = nearest_ref(ref_times, ref_base_xyz, stamp_s)
        if ref is None:
            continue
        progress = core[stamp]["distance_m"] - first_s
        map_s = anchor.s + progress
        correction = residual[:2] * math.exp(-max(0.0, progress) / 100.0)
        master_pose = np.asarray(route.sample(direction, map_s))
        master_pose[:2] += correction
        early_master_sq += float(np.sum((master_pose - ref) ** 2))
        if ref_base is not None:
            base_pose = np.asarray(route.master_to_base_enu(direction, map_s))
            base_pose[:2] += correction
            early_base_sq += float(np.sum((base_pose - ref_base) ** 2))
        early_matched += 1
    total_n = int(row["matched"])
    if early_matched >= total_n:
        raise ValueError(f"all points before anchor: {bag}")
    def gated_rmse(field: str, early_sq: float) -> float:
        total_sq = total_n * float(row[field]) ** 2
        return math.sqrt(max(0.0, total_sq - early_sq) / (total_n - early_matched))
    return {
        "bag": bag,
        "split": "validation" if bag in VALIDATION else "holdout",
        "third_fix_stamp_s": third_fix_stamp_s,
        "third_fix_delay_from_first_input_s": third_fix_stamp_s - t0,
        "pre_anchor_core_outputs": early_outputs,
        "pre_anchor_matched": early_matched,
        "proxy_matched_total": total_n,
        "master_rmse_original_m": float(row["rmse_3d_m"]),
        "master_rmse_after_gate_m": gated_rmse("rmse_3d_m", early_master_sq),
        "base_rmse_original_m": float(row["base_rmse_3d_m"]),
        "base_rmse_after_gate_m": gated_rmse("base_rmse_3d_m", early_base_sq),
    }


if __name__ == "__main__":
    route = RouteMap.from_csv(MAP)
    groups = []
    for path in INPUTS:
        with path.open(newline="", encoding="utf-8") as f:
            groups.append(list(csv.DictReader(f)))
    VALIDATION = {row["bag"] for row in groups[0]}
    output = []
    for group in groups:
        for row in group:
            result = score_early(row, route)
            output.append(result)
            print(result["split"], result["bag"],
                  f"early={result['pre_anchor_matched']}/{result['proxy_matched_total']}",
                  f"base RMSE {result['base_rmse_original_m']:.6f}"
                  f" -> {result['base_rmse_after_gate_m']:.6f}", flush=True)
    summary = {}
    for split in ("validation", "holdout"):
        runs = [row for row in output if row["split"] == split]
        deltas = [row["base_rmse_after_gate_m"] - row["base_rmse_original_m"]
                  for row in runs]
        summary[split] = {
            "bags": len(runs),
            "pre_anchor_core_outputs": sum(row["pre_anchor_core_outputs"] for row in runs),
            "pre_anchor_matched": sum(row["pre_anchor_matched"] for row in runs),
            "proxy_matched_total": sum(row["proxy_matched_total"] for row in runs),
            "bags_with_early_matches": sum(row["pre_anchor_matched"] > 0 for row in runs),
            "max_abs_base_rmse_change_m": max(map(abs, deltas)),
            "median_base_rmse_change_m": statistics.median(deltas),
            "max_third_fix_delay_s": max(
                row["third_fix_delay_from_first_input_s"] for row in runs),
        }
    report = {"method": "remove samples before 3rd raw master fix from existing position_proxy RMSE; production ROS withholds position", "summary": summary, "runs": output}
    target = ROOT / "evaluation/audit_position_causality.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(target)
