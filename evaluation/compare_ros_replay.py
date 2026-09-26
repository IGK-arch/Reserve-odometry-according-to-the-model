"""Audit numeric ROS output against the offline C++ replay and GNSS proxies.

Use with `ROS_SMOKE_RECORD=1 bash tools/ros_smoke_wsl.sh anchored`.  The bag is
read only for its recorded input order and *evaluation* references.  GNSS never
enters the estimator.  This script does not change production code.
"""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics

from core_benchmark import DEFAULT_EXE, metrics, read_run, reference_at, replay_cpp
from causal_baseline import CMD, FRONT, REAR, make_reference

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "ros2_ws/src/tram_odometry/assets/drive_accel_table.csv"


def summary(errors):
    result = metrics(errors)
    result["p50_abs"] = statistics.median(map(abs, errors)) if errors else None
    result["p95_abs"] = sorted(map(abs, errors))[math.ceil(len(errors) * .95) - 1] if errors else None
    result["max_abs"] = max(map(abs, errors)) if errors else None
    return result


def load_samples(path):
    speed, position = {}, {}
    duplicate = Counter()
    with path.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            stamp = int(row["stamp_ns"])
            if row["topic"] == "velocity":
                duplicate[stamp] += bool(stamp in speed)
                speed[stamp] = float(row["velocity_mps"])
            elif row["topic"] == "position":
                duplicate[stamp] += bool(stamp in position)
                position[stamp] = tuple(float(row[key]) for key in ("x_m", "y_m", "z_m"))
    return speed, position, sum(duplicate.values())


def position_reference(bag):
    """Produce quality-filtered dual-GNSS base_link proxy in official MGRS XYZ."""
    try:
        import numpy as np
        from pyproj import Transformer
        from position_proxy import clean_reference_positions
        from analyze_gnss import extract, finite_positions
    except ImportError:
        return None
    cleaned = clean_reference_positions(bag)
    if cleaned is None:
        return None
    good_times = cleaned[0]
    db = next((ROOT / "dataset/data" / bag).glob("*.db3"))
    _, _, records = extract(db)
    master = finite_positions(records["mf"])
    rover = finite_positions(records["rf"])
    master = master[np.argsort(master[:, 1], kind="stable")]
    rover = rover[np.argsort(rover[:, 1], kind="stable")]
    master = master[np.isin(master[:, 1], good_times)]
    if not len(master) or len(rover) < 2:
        return None
    idx = np.searchsorted(rover[:, 1], master[:, 1])
    idx = np.clip(idx, 1, len(rover) - 1)
    before = np.abs(rover[idx - 1, 1] - master[:, 1])
    after = np.abs(rover[idx, 1] - master[:, 1])
    idx -= before < after
    rover = rover[idx]
    transformer = Transformer.from_crs(4326, 32637, always_xy=True)
    me, mn = transformer.transform(master[:, 3], master[:, 2])
    re, rn = transformer.transform(rover[:, 3], rover[:, 2])
    dx, dy = re - me, rn - mn
    norm = np.maximum(np.hypot(dx, dy), 1e-6)
    x = me - 300000.0 + 9.873 * dx / norm
    y = mn - 6100000.0 + 9.873 * dy / norm
    z = master[:, 4] - 3.0
    return master[:, 1].tolist(), list(zip(x.tolist(), y.tolist(), z.tolist()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Bag ID, such as 30618_af7496f0")
    parser.add_argument("samples", type=Path, help="CSV written by ROS smoke monitor")
    parser.add_argument("--exe", type=Path, default=DEFAULT_EXE)
    parser.add_argument("--table", type=Path, default=TABLE)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    events, truth = read_run(args.bag)
    core = replay_cpp(args.exe.resolve(), args.bag, events,
                      args.table if args.bag.startswith("30618") else None)
    speed, position, duplicates = load_samples(args.samples)
    reference = make_reference(truth)
    same = sorted(speed.keys() & core.keys())
    ros_vs_core = [speed[t] - core[t]["velocity_mps"] for t in same]
    ros_ref = [(speed[t] - ref) for t in sorted(speed)
               if (ref := reference_at(reference, t)) is not None]
    core_ref = [(core[t]["velocity_mps"] - ref) for t in same
                if (ref := reference_at(reference, t)) is not None]
    first_topic = {}
    topics_by_stamp = {}
    for _, stamp, topic, _ in events:
        first_topic.setdefault(stamp, topic)
        topics_by_stamp.setdefault(stamp, []).append(topic)
    source_counts = Counter(first_topic.get(t, "missing") for t in speed)
    pos_ref = position_reference(args.bag)
    position_error = []
    if pos_ref:
        times, xyzs = pos_ref
        for t, xyz in position.items():
            target = t * 1e-9
            i = bisect.bisect_left(times, target)
            candidate = min(range(max(0, i - 1), min(len(times), i + 1)),
                            key=lambda j: abs(times[j] - target))
            if abs(times[candidate] - target) <= .05:
                position_error.append(math.dist(xyz, xyzs[candidate]))
    result = {
        "bag": args.bag,
        "sample_file": str(args.samples),
        "ros_speed_stamps": len(speed),
        "ros_position_stamps": len(position),
        "ros_duplicate_output_stamps": duplicates,
        "offline_controller_stamps": len(core),
        "exact_stamp_intersection": len(same),
        "bag_first_input_topic_for_ros_stamps": dict(source_counts),
        "ros_minus_offline_mps": summary(ros_vs_core),
        "ros_vs_offline_delta_gt_0_01_mps": sum(abs(delta) > .01 for delta in ros_vs_core),
        "ros_vs_offline_delta_gt_0_10_mps": sum(abs(delta) > .10 for delta in ros_vs_core),
        "largest_ros_offline_differences": [
            {"stamp_ns": t, "delta_mps": speed[t] - core[t]["velocity_mps"],
             "bag_event_topics": topics_by_stamp.get(t, [])}
            for t in sorted(same, key=lambda stamp: abs(
                speed[stamp] - core[stamp]["velocity_mps"]), reverse=True)[:8]
        ],
        "ros_minus_gnss_mps": summary(ros_ref),
        "offline_minus_gnss_on_same_stamps_mps": summary(core_ref),
        "ros_position_minus_dual_gnss_base_proxy_3d_m": summary(position_error),
    }
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                            encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
