"""Causal wheel-speed baseline and independent GNSS reference audit.

The estimator sees only the three permitted vehicle topics, in SQLite receive
order. GNSS is decoded separately *after* producing estimates and never gates
an input. Metrics are a local proxy: the judge's fused localization is absent
from the supplied bags. The script is intentionally dependency-free.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "viewer"))
from rosbag_cdr import messages  # noqa: E402

FRONT = "/vehicle/front_bogie_velocity"
REAR = "/vehicle/rear_bogie_velocity"
CMD = "/vehicle/driver_position_cmd"
MASTER = "/sensing/gnss/master/vel"
ROVER = "/sensing/gnss/rover/vel"
TOPICS = {FRONT, REAR, CMD, MASTER, ROVER}


@dataclass
class Wheel:
    stamp_ns: int = -1
    speed_mps: float = math.nan


@dataclass
class Estimate:
    stamp_ns: int
    receive_ns: int
    speed_mps: float
    front_used: bool
    rear_used: bool


def read_run(bag: str):
    bag_dir = ROOT / "dataset" / "data" / bag
    events = []
    truth = {MASTER: [], ROVER: []}
    for topic, receive_ns, msg in messages(bag_dir, TOPICS):
        stamp_ns = int(msg["stamp_ns"])
        if topic in truth:
            vec = msg["linear"]
            speed = math.sqrt(sum(float(x) ** 2 for x in vec))
            if math.isfinite(speed) and 0 <= speed <= 25:
                truth[topic].append((stamp_ns, speed))
            continue
        value = msg["position"] if topic == CMD else msg["velocity_raw"]
        events.append((receive_ns, stamp_ns, topic, value))
    return events, truth


def causal_wheel_baseline(events, max_wheel_age_s: float = 0.25):
    """Publish at each valid controller stamp, preserving receive order."""
    wheels = {FRONT: Wheel(), REAR: Wheel()}
    estimates = []
    last_output_stamp = -1
    age_ns = int(max_wheel_age_s * 1e9)
    for receive_ns, stamp_ns, topic, value in events:
        if stamp_ns <= 0 or not math.isfinite(float(value)):
            continue
        if topic in wheels:
            if not 0 <= value <= 150:  # raw units are km/h
                continue
            wheel = wheels[topic]
            if stamp_ns > wheel.stamp_ns:
                wheel.stamp_ns = stamp_ns
                wheel.speed_mps = value / 3.6
            continue
        if topic != CMD or not -15 <= int(value) <= 15:
            continue
        if stamp_ns <= last_output_stamp:
            continue
        last_output_stamp = stamp_ns
        current = []
        used = {}
        for name, wheel in wheels.items():
            age = stamp_ns - wheel.stamp_ns
            good = 0 <= age <= age_ns and math.isfinite(wheel.speed_mps)
            used[name] = good
            if good:
                current.append(wheel.speed_mps)
        if current:
            speed = sum(current) / len(current)
        else:
            speed = estimates[-1].speed_mps if estimates else 0.0
        estimates.append(
            Estimate(stamp_ns, receive_ns, speed, used[FRONT], used[REAR])
        )
    return estimates


def nearest(stream, stamp_ns: int, tolerance_ns: int = 50_000_000):
    stamps, speeds = stream
    if not stamps:
        return None
    idx = bisect.bisect_left(stamps, stamp_ns)
    candidates = range(max(0, idx - 1), min(len(stamps), idx + 1))
    nearest_idx = min(candidates, key=lambda i: abs(stamps[i] - stamp_ns))
    if abs(stamps[nearest_idx] - stamp_ns) <= tolerance_ns:
        return speeds[nearest_idx]
    return None


def make_reference(truth):
    """Sort GNSS only in the evaluator; no GNSS enters causal estimator."""
    sorted_truth = {k: sorted(v) for k, v in truth.items()}
    return {
        k: (tuple(p[0] for p in rows), tuple(p[1] for p in rows))
        for k, rows in sorted_truth.items()
    }


def score(estimates, truth):
    master, rover = truth[MASTER], truth[ROVER]
    errors = []
    two_receiver = 0
    one_receiver = 0
    for est in estimates:
        m = nearest(master, est.stamp_ns)
        r = nearest(rover, est.stamp_ns)
        if m is not None and r is not None:
            if abs(m - r) > 0.3:
                continue
            ref = (m + r) / 2
            two_receiver += 1
        elif m is not None or r is not None:
            ref = m if m is not None else r
            one_receiver += 1
        else:
            continue
        errors.append(est.speed_mps - ref)
    n = len(errors)
    return {
        "outputs": len(estimates),
        "matched": n,
        "coverage": n / len(estimates) if estimates else 0.0,
        "two_receiver": two_receiver,
        "one_receiver": one_receiver,
        "rmse_mps": math.sqrt(sum(e * e for e in errors) / n) if n else math.nan,
        "mae_mps": sum(abs(e) for e in errors) / n if n else math.nan,
        "bias_mps": sum(errors) / n if n else math.nan,
        "front_only": sum(x.front_used and not x.rear_used for x in estimates),
        "rear_only": sum(x.rear_used and not x.front_used for x in estimates),
        "no_wheel": sum(not x.front_used and not x.rear_used for x in estimates),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="*", help="Bag IDs, e.g. 30618_0e41eac3")
    parser.add_argument("--all-unique", action="store_true", help="Use one bag per SQLite SHA-256 group")
    parser.add_argument("--split", choices=("train", "validation", "holdout"), help="Use frozen split manifest")
    parser.add_argument("--out", type=Path, help="Optional summary CSV")
    args = parser.parse_args()
    bag_ids = list(args.bags)
    if args.split:
        manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
        bag_ids.extend(manifest["representatives"][args.split])
    if args.all_unique:
        with (ROOT / "analysis" / "signals" / "duplicates.csv").open(
            newline="", encoding="utf-8"
        ) as file:
            seen = set()
            for row in csv.DictReader(file):
                digest = row["file_sha256"]
                if digest not in seen:
                    seen.add(digest)
                    bag_ids.append(row["bag"])
    bag_ids = list(dict.fromkeys(bag_ids))
    if not bag_ids:
        parser.error("provide bag IDs or --all-unique")
    rows = []
    for bag in bag_ids:
        events, truth = read_run(bag)
        estimates = causal_wheel_baseline(events)
        row = {"bag": bag, **score(estimates, make_reference(truth))}
        rows.append(row)
        print(
            f"{bag}: outputs={row['outputs']} matched={row['matched']} "
            f"coverage={row['coverage']:.1%} RMSE={row['rmse_mps']:.4f} m/s "
            f"MAE={row['mae_mps']:.4f} m/s bias={row['bias_mps']:.4f} m/s"
        )
    for vehicle in ("30618", "30639", "all"):
        selected = [
            row for row in rows
            if row["matched"] and (vehicle == "all" or row["bag"].startswith(vehicle))
        ]
        if not selected:
            continue
        count = sum(row["matched"] for row in selected)
        rmse = math.sqrt(sum(row["matched"] * row["rmse_mps"] ** 2 for row in selected) / count)
        mae = sum(row["matched"] * row["mae_mps"] for row in selected) / count
        bias = sum(row["matched"] * row["bias_mps"] for row in selected) / count
        median_bag_rmse = statistics.median(row["rmse_mps"] for row in selected)
        print(
            f"{vehicle} aggregate: labeled_bags={len(selected)} samples={count} "
            f"RMSE={rmse:.4f} MAE={mae:.4f} bias={bias:.4f} "
            f"median_bag_RMSE={median_bag_rmse:.4f} m/s"
        )
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
