"""3D route-map position proxy using the production C++ speed estimator.

Only the first three master GNSS fixes anchor the predicted path. Later GNSS
fixes are decoded in this separate evaluator for scoring, never passed to the
estimator. A second metric approximates the judge's base_link target from the
known dual-antenna TF. The judge's fused localization is not in the bags.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter

from causal_baseline import causal_wheel_baseline, read_run
from core_benchmark import DEFAULT_EXE, replay_cpp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "routes"))
sys.path.insert(0, str(ROOT / "tools"))
from analyze_gnss import enu, extract, finite_positions  # noqa: E402
from route_map_io import RouteMap  # noqa: E402

MAP = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "route_map.csv"


def clean_reference_positions(bag):
    path = next((ROOT / "dataset" / "data" / bag).glob("*.db3"))
    _, _, records = extract(path)
    master = finite_positions(records["mf"])
    rover = finite_positions(records["rf"])
    if len(master) < 100 or len(rover) < 100:
        return None
    master = master[np.argsort(master[:, 1], kind="stable")]
    rover = rover[np.argsort(rover[:, 1], kind="stable")]
    pm = enu(master[:, 2], master[:, 3], master[:, 4])
    pr = enu(rover[:, 2], rover[:, 3], rover[:, 4])
    rv_idx = np.searchsorted(rover[:, 1], master[:, 1])
    rv_idx = np.clip(rv_idx, 1, len(rover) - 1)
    before = np.abs(rover[rv_idx - 1, 1] - master[:, 1])
    after = np.abs(rover[rv_idx, 1] - master[:, 1])
    rv_idx -= before < after
    age = np.abs(rover[rv_idx, 1] - master[:, 1])
    separation = np.linalg.norm(pm[:, :2] - pr[rv_idx, :2], axis=1)
    local_median = median_filter(pm, size=(9, 1), mode="nearest")
    jumps = np.linalg.norm(pm - local_median, axis=1)
    good = ((age <= 0.15) & (separation >= 9.0) & (separation <= 16.0)
            & (jumps < 3.0) & (master[:, 5] >= 0))
    if good.sum() < 100:
        return None
    antenna_vector = median_filter(pr[rv_idx] - pm, size=(9, 1), mode="nearest")
    antenna_norm = np.linalg.norm(antenna_vector, axis=1)
    antenna_norm = np.maximum(antenna_norm, 1e-6)
    base_xyz = pm + 9.873 * antenna_vector / antenna_norm[:, None]
    base_xyz[:, 2] -= 3.0
    return master[good, 1], pm[good], base_xyz[good], master[:, 1], pm


def nearest_ref(times, positions, stamp_s):
    index = bisect.bisect_left(times, stamp_s)
    candidates = range(max(0, index - 1), min(len(times), index + 1))
    i = min(candidates, key=lambda k: abs(times[k] - stamp_s))
    return positions[i] if abs(times[i] - stamp_s) <= 0.05 else None


def score_bag(bag, route, exe, drive_table=None):
    events, _ = read_run(bag)
    if not events:
        return None
    reference_data = clean_reference_positions(bag)
    if reference_data is None:
        return None
    ref_times, ref_xyz, ref_base_xyz, raw_times, raw_xyz = reference_data
    t0 = min(stamp for _, stamp, _, _ in events) * 1e-9
    startup = raw_xyz[(raw_times >= t0) & (raw_times <= t0 + 5.0)]
    if len(startup) < 3:
        return None
    initial = np.median(startup[:3], axis=0)
    direction = "out" if initial[0] > -200 else "return" if initial[0] < -4400 else None
    if direction is None:
        return None
    anchor = route.project(float(initial[0]), float(initial[1]), direction)
    if anchor.horizontal_distance_m > 50.0:
        return None
    initial_map = np.asarray(route.sample(direction, anchor.s))
    residual = initial - initial_map
    core = replay_cpp(exe, bag, events, drive_table)
    if not core:
        return None
    baseline = sorted(causal_wheel_baseline(events), key=lambda row: row.stamp_ns)
    baseline_distance = {}
    elapsed_m = 0.0
    previous = None
    for row in baseline:
        if previous is not None:
            dt = (row.stamp_ns - previous.stamp_ns) * 1e-9
            if 0.0 < dt <= 1.0:
                elapsed_m += 0.5 * (previous.speed_mps + row.speed_mps) * dt
        baseline_distance[row.stamp_ns] = elapsed_m
        previous = row
    common = sorted(core.keys() & baseline_distance.keys())
    if not common:
        return None
    first_s = core[min(core)]["distance_m"]
    first_baseline_s = baseline_distance[common[0]]
    errors_3d = []
    errors_xy = []
    first_100 = []
    baseline_errors_3d = []
    baseline_errors_xy = []
    base_errors_3d = []
    baseline_base_errors_3d = []
    for stamp in common:
        state = core[stamp]
        ref = nearest_ref(ref_times, ref_xyz, stamp * 1e-9)
        ref_base = nearest_ref(ref_times, ref_base_xyz, stamp * 1e-9)
        if ref is None:
            continue
        progress = state["distance_m"] - first_s
        position = np.asarray(route.sample(direction, anchor.s + progress))
        position[:2] += residual[:2] * math.exp(-max(0.0, progress) / 100.0)
        delta = position - ref
        distance_3d = float(np.linalg.norm(delta))
        errors_3d.append(distance_3d)
        errors_xy.append(float(np.linalg.norm(delta[:2])))
        baseline_progress = baseline_distance[stamp] - first_baseline_s
        baseline_position = np.asarray(route.sample(direction, anchor.s + baseline_progress))
        baseline_position[:2] += residual[:2] * math.exp(-max(0.0, baseline_progress) / 100.0)
        baseline_delta = baseline_position - ref
        baseline_errors_3d.append(float(np.linalg.norm(baseline_delta)))
        baseline_errors_xy.append(float(np.linalg.norm(baseline_delta[:2])))
        if ref_base is not None:
            base_position = np.asarray(route.master_to_base_enu(direction, anchor.s + progress))
            base_position[:2] += residual[:2] * math.exp(-max(0.0, progress) / 100.0)
            baseline_base_position = np.asarray(route.master_to_base_enu(direction, anchor.s + baseline_progress))
            baseline_base_position[:2] += residual[:2] * math.exp(-max(0.0, baseline_progress) / 100.0)
            base_errors_3d.append(float(np.linalg.norm(base_position - ref_base)))
            baseline_base_errors_3d.append(float(np.linalg.norm(baseline_base_position - ref_base)))
        if progress <= 100.0:
            first_100.append(distance_3d)
    if len(errors_3d) < 100:
        return None
    return {
        "bag": bag,
        "direction": direction,
        "matched": len(errors_3d),
        "start_projection_m": anchor.horizontal_distance_m,
        "rmse_3d_m": math.sqrt(sum(e * e for e in errors_3d) / len(errors_3d)),
        "rmse_xy_m": math.sqrt(sum(e * e for e in errors_xy) / len(errors_xy)),
        "baseline_rmse_3d_m": math.sqrt(sum(e * e for e in baseline_errors_3d) / len(baseline_errors_3d)),
        "baseline_rmse_xy_m": math.sqrt(sum(e * e for e in baseline_errors_xy) / len(baseline_errors_xy)),
        "base_rmse_3d_m": math.sqrt(sum(e * e for e in base_errors_3d) / len(base_errors_3d)),
        "baseline_base_rmse_3d_m": math.sqrt(sum(e * e for e in baseline_base_errors_3d) / len(baseline_base_errors_3d)),
        "first_100m_rmse_3d_m": math.sqrt(sum(e * e for e in first_100) / len(first_100)) if first_100 else math.nan,
        "end_error_3d_m": errors_3d[-1],
        "baseline_end_error_3d_m": baseline_errors_3d[-1],
        "base_end_error_3d_m": base_errors_3d[-1],
        "baseline_base_end_error_3d_m": baseline_base_errors_3d[-1],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="*")
    parser.add_argument("--split", choices=("train", "validation", "holdout"))
    parser.add_argument("--exe", type=Path, default=DEFAULT_EXE)
    parser.add_argument("--drive-table", type=Path,
                        help="Enable the train-only drive table in the C++ replay")
    parser.add_argument("--table-vehicle", choices=("30618", "30639", "all"),
                        default="all", help="Vehicle receiving --drive-table")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    route = RouteMap.from_csv(MAP)
    bags = list(args.bags)
    if args.split:
        manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
        bags.extend(manifest["representatives"][args.split])
    bags = list(dict.fromkeys(bags))
    if not bags:
        parser.error("provide bag IDs or --split")
    rows = []
    for bag in bags:
        table = args.drive_table if args.table_vehicle == "all" or bag.startswith(args.table_vehicle) else None
        row = score_bag(bag, route, args.exe.resolve(), table)
        if row:
            rows.append(row)
            print(f"{bag}: n={row['matched']} 3D_RMSE={row['rmse_3d_m']:.2f}m end={row['end_error_3d_m']:.2f}m")
        else:
            print(f"{bag}: no usable 3D proxy")
    if rows:
        print(
            f"{len(rows)} bags: median 3D_RMSE="
            f"{statistics.median(x['rmse_3d_m'] for x in rows):.2f}m, "
            f"median end_error={statistics.median(x['end_error_3d_m'] for x in rows):.2f}m"
        )
    if rows and args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
