"""Compare along-track integrated distance with GNSS velocity on labeled bags.

This is a *proxy* for position scoring. The judge's fused 3D reference and
coordinate frame are not present in the supplied bag. Intervals without a
valid GNSS speed reference are excluded from all three integrals equally.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from causal_baseline import causal_wheel_baseline, make_reference, read_run
from core_benchmark import DEFAULT_EXE, reference_at, replay_cpp

ROOT = Path(__file__).resolve().parents[1]


def score_bag(bag, exe, drive_table=None):
    events, raw_truth = read_run(bag)
    core = replay_cpp(exe, bag, events, drive_table)
    baseline = {x.stamp_ns: x for x in causal_wheel_baseline(events)}
    truth = make_reference(raw_truth)
    ref_dist = base_dist = core_dist = 0.0
    seconds = 0.0
    previous = None
    for stamp in sorted(baseline.keys() & core.keys()):
        reference = reference_at(truth, stamp)
        if reference is None:
            previous = None
            continue
        current = (stamp, reference, baseline[stamp].speed_mps,
                   core[stamp]["velocity_mps"], core[stamp]["distance_m"])
        if previous:
            dt = (current[0] - previous[0]) / 1e9
            if 0 < dt <= 0.15:
                seconds += dt
                ref_dist += 0.5 * (current[1] + previous[1]) * dt
                base_dist += 0.5 * (current[2] + previous[2]) * dt
                core_dist += current[4] - previous[4]
        previous = current
    if ref_dist < 1.0:
        return None
    return {
        "bag": bag,
        "scored_seconds": seconds,
        "reference_distance_m": ref_dist,
        "baseline_distance_m": base_dist,
        "core_distance_m": core_dist,
        "baseline_drift_m": base_dist - ref_dist,
        "core_drift_m": core_dist - ref_dist,
        "baseline_drift_pct": 100 * (base_dist - ref_dist) / ref_dist,
        "core_drift_pct": 100 * (core_dist - ref_dist) / ref_dist,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="*")
    parser.add_argument("--split", choices=("train", "validation", "holdout"))
    parser.add_argument("--exe", type=Path, default=DEFAULT_EXE)
    parser.add_argument("--drive-table", type=Path,
                        help="Experimental train-only acceleration CSV; default off")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    bag_ids = list(args.bags)
    if args.split:
        manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
        bag_ids.extend(manifest["representatives"][args.split])
    bag_ids = list(dict.fromkeys(bag_ids))
    if not bag_ids:
        parser.error("provide bag IDs or --split")
    rows = []
    for bag in bag_ids:
        row = score_bag(bag, args.exe.resolve(), args.drive_table)
        if row:
            rows.append(row)
            print(
                f"{bag}: ref={row['reference_distance_m']:.1f}m "
                f"baseline={row['baseline_drift_pct']:+.3f}% "
                f"core={row['core_drift_pct']:+.3f}%"
            )
    if rows:
        print(
            f"Median absolute end drift: baseline "
            f"{statistics.median(abs(r['baseline_drift_pct']) for r in rows):.3f}% "
            f"core {statistics.median(abs(r['core_drift_pct']) for r in rows):.3f}% "
            f"on {len(rows)} bags"
        )
    if args.out and rows:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
