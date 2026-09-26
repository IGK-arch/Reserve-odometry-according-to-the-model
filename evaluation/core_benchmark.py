"""Replay the production C++ estimator in bag receive order, then score it.

No ROS dependency is needed for this preflight benchmark. A fixed train/validation/
holdout manifest prevents duplicate bag leakage. GNSS stays outside the process
that runs the estimator, and is used only after replay for approximate metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))
from causal_baseline import (  # noqa: E402
    CMD,
    FRONT,
    MASTER,
    REAR,
    ROVER,
    causal_wheel_baseline,
    make_reference,
    nearest,
    read_run,
)

DEFAULT_EXE = ROOT / "evaluation" / ("replay_cli.exe" if sys.platform == "win32" else "replay_cli")


def compile_cli(output: Path):
    cmd = [
        "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-pedantic",
        "-I", str(ROOT / "ros2_ws" / "src" / "tram_odometry" / "include"),
        str(ROOT / "ros2_ws" / "src" / "tram_odometry" / "src" / "estimator.cpp"),
        str(ROOT / "evaluation" / "replay_cli.cpp"),
        "-o", str(output),
    ]
    subprocess.run(cmd, check=True, cwd=ROOT)


def replay_cpp(executable: Path, bag: str, events, drive_table: Path | None = None):
    codes = {FRONT: "F", REAR: "R", CMD: "C"}
    input_csv = "".join(
        f"{recv},{stamp},{codes[topic]},{value}\n"
        for recv, stamp, topic, value in events
    )
    command = [str(executable), bag.split("_", 1)[0]]
    if drive_table is not None:
        command.append(str(drive_table.resolve()))
    result = subprocess.run(
        command,
        input=input_csv,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
        timeout=180,
        cwd=ROOT,
    )
    output = {}
    for row in csv.DictReader(result.stdout.splitlines()):
        stamp_ns = int(row["stamp_ns"])
        output[stamp_ns] = {
            "velocity_mps": float(row["velocity_mps"]),
            "distance_m": float(row["distance_m"]),
            "front_weight": float(row["front_weight"]),
            "rear_weight": float(row["rear_weight"]),
            "front_slip": bool(int(row["front_slip"])),
            "rear_slip": bool(int(row["rear_slip"])),
            "model_only": bool(int(row["model_only"])),
        }
    return output


def reference_at(truth, stamp_ns):
    m = nearest(truth[MASTER], stamp_ns)
    r = nearest(truth[ROVER], stamp_ns)
    if m is not None and r is not None:
        return (m + r) / 2 if abs(m - r) <= 0.3 else None
    return m if m is not None else r


def metrics(errors):
    count = len(errors)
    return {
        "n": count,
        "rmse": math.sqrt(sum(e * e for e in errors) / count) if count else math.nan,
        "mae": sum(abs(e) for e in errors) / count if count else math.nan,
        "bias": sum(errors) / count if count else math.nan,
    }


def compare(bag: str, events, truth, core):
    baseline = {x.stamp_ns: x for x in causal_wheel_baseline(events)}
    reference = make_reference(truth)
    common = sorted(baseline.keys() & core.keys())
    e_base, e_core = [], []
    for stamp_ns in common:
        ref = reference_at(reference, stamp_ns)
        if ref is None:
            continue
        e_base.append(baseline[stamp_ns].speed_mps - ref)
        e_core.append(core[stamp_ns]["velocity_mps"] - ref)
    b = metrics(e_base)
    c = metrics(e_core)
    return {
        "bag": bag,
        "baseline_outputs": len(baseline),
        "core_outputs": len(core),
        "matched": b["n"],
        "baseline_rmse_mps": b["rmse"],
        "core_rmse_mps": c["rmse"],
        "baseline_mae_mps": b["mae"],
        "core_mae_mps": c["mae"],
        "baseline_bias_mps": b["bias"],
        "core_bias_mps": c["bias"],
        "core_slip_outputs": sum(x["front_slip"] or x["rear_slip"] for x in core.values()),
        "core_model_only_outputs": sum(x["model_only"] for x in core.values()),
    }


def aggregate(rows, label):
    selected = [r for r in rows if r["matched"] and (label == "all" or r["bag"].startswith(label))]
    if not selected:
        return
    n = sum(r["matched"] for r in selected)
    b_rmse = math.sqrt(sum(r["matched"] * r["baseline_rmse_mps"] ** 2 for r in selected) / n)
    c_rmse = math.sqrt(sum(r["matched"] * r["core_rmse_mps"] ** 2 for r in selected) / n)
    b_bias = sum(r["matched"] * r["baseline_bias_mps"] for r in selected) / n
    c_bias = sum(r["matched"] * r["core_bias_mps"] for r in selected) / n
    median_delta = statistics.median(
        r["core_rmse_mps"] - r["baseline_rmse_mps"] for r in selected
    )
    wins = sum(r["core_rmse_mps"] < r["baseline_rmse_mps"] for r in selected)
    print(
        f"{label}: bags={len(selected)} matched={n} baseline={b_rmse:.4f} "
        f"core={c_rmse:.4f} m/s bias={b_bias:+.4f}/{c_bias:+.4f} "
        f"wins={wins}/{len(selected)} "
        f"median_bag_delta={median_delta:+.4f} m/s"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="*")
    parser.add_argument("--split", choices=("train", "validation", "holdout"))
    parser.add_argument("--exe", type=Path, default=DEFAULT_EXE)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--drive-table", type=Path,
                        help="Experimental train-only acceleration CSV; default off")
    parser.add_argument("--table-vehicle", choices=("30618", "30639", "all"),
                        default="all", help="Vehicle receiving --drive-table; use 30618 for deployed mode")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    executable = args.exe.resolve()
    if args.build:
        compile_cli(executable)
    if not executable.is_file():
        parser.error(f"missing {executable}; pass --build")
    bag_ids = list(args.bags)
    if args.split:
        manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
        bag_ids.extend(manifest["representatives"][args.split])
    bag_ids = list(dict.fromkeys(bag_ids))
    if not bag_ids:
        parser.error("provide bag IDs or --split")
    rows = []
    for bag in bag_ids:
        events, truth = read_run(bag)
        table = args.drive_table if args.table_vehicle == "all" or bag.startswith(args.table_vehicle + "_") else None
        core = replay_cpp(executable, bag, events, table)
        row = compare(bag, events, truth, core)
        rows.append(row)
        if row["matched"]:
            print(
                f"{bag}: n={row['matched']} baseline={row['baseline_rmse_mps']:.4f} "
                f"core={row['core_rmse_mps']:.4f} m/s"
            )
        else:
            print(f"{bag}: no GNSS reference; outputs={row['core_outputs']}")
    for label in ("30618", "30639", "all"):
        aggregate(rows, label)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
