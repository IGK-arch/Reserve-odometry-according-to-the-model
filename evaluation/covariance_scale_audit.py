"""Train-only audit of the shared wheel-scale variance floor.

This scores the unchanged point estimator against an integrated GNSS-speed
proxy. It does not calibrate a confidence interval against the judge's hidden
trajectory. Holdout bags are never opened.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

from calibrate_wheels import estimate_bag
from core_benchmark import compile_cli
from distance_proxy import score_bag

ROOT = Path(__file__).resolve().parents[1]
SCALE_SIGMA = 0.01
MASTER_ANCHOR_VARIANCE_M2 = 4.0
MIN_REFERENCE_DISTANCE_M = 1000.0


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def coverage(rows: list[dict]) -> dict:
    z = [row["normalized_drift"] for row in rows]
    return {
        "bags": len(rows),
        "inside_one_diagnostic_sigma": sum(value <= 1.0 for value in z),
        "inside_two_diagnostic_sigma": sum(value <= 2.0 for value in z),
        "max_normalized_drift": max(z) if z else None,
    }


def audit(executable: Path) -> dict:
    manifest_path = ROOT / "tools/split_manifest.json"
    table_path = ROOT / "ros2_ws/src/tram_odometry/assets/drive_accel_table.csv"
    source_path = ROOT / "ros2_ws/src/tram_odometry/src/estimator.cpp"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    with (ROOT / "analysis/catalog/bags.csv").open(newline="", encoding="utf-8") as handle:
        catalog = {row["bag"]: row for row in csv.DictReader(handle)}

    session_scales = defaultdict(list)
    for bag in manifest["representatives"]["train"]:
        fitted = estimate_bag(bag)
        if fitted is None:
            continue
        key = (bag.split("_", 1)[0], catalog[bag]["start_utc"][:10])
        session_scales[key].append(
            0.5 * (fitted["front_scale"] + fitted["rear_scale"])
        )

    rows_by_split = {}
    for split in ("train", "validation"):
        rows = []
        for bag in manifest["representatives"][split]:
            table = table_path if bag.startswith("30618_") else None
            scored = score_bag(bag, executable, table)
            if scored is None or scored["reference_distance_m"] < MIN_REFERENCE_DISTANCE_M:
                continue
            traveled_m = max(0.0, scored["core_distance_m"])
            diagnostic_sigma_m = math.sqrt(
                MASTER_ANCHOR_VARIANCE_M2 + (SCALE_SIGMA * traveled_m) ** 2
            )
            rows.append({
                "bag": bag,
                "reference_distance_m": scored["reference_distance_m"],
                "core_distance_m": scored["core_distance_m"],
                "core_drift_m": scored["core_drift_m"],
                "diagnostic_sigma_m_lower_bound": diagnostic_sigma_m,
                "normalized_drift": abs(scored["core_drift_m"]) / diagnostic_sigma_m,
            })
        rows_by_split[split] = rows

    return {
        "schema_version": 1,
        "scope": "train parameter evidence; train and validation proxy coverage; no holdout access",
        "formula": "sigma_along^2 = P_s + anchor_variance + (scale_sigma * max(0, s - s_anchor))^2",
        "coverage_approximation": "For scored GNSS intervals, use 4 m^2 master-anchor variance and (0.01 * core_distance_on_scored_intervals)^2; omit positive P_s, so this is a lower bound on the reported variance. It is not a probability confidence interval.",
        "scale_sigma": SCALE_SIGMA,
        "master_anchor_variance_m2": MASTER_ANCHOR_VARIANCE_M2,
        "min_reference_distance_m": MIN_REFERENCE_DISTANCE_M,
        "source_sha256": {
            "split_manifest.json": digest(manifest_path),
            "estimator.cpp": digest(source_path),
            "drive_accel_table.csv": digest(table_path),
        },
        "train_session_gnss_to_wheel_scale_medians": [
            {"vehicle": vehicle, "date": date, "bags": len(values),
             "common_scale_median": statistics.median(values),
             "common_scale_min": min(values), "common_scale_max": max(values)}
            for (vehicle, date), values in sorted(session_scales.items())
        ],
        "coverage": {split: coverage(rows) for split, rows in rows_by_split.items()},
        "rows": rows_by_split,
        "limitations": [
            "GNSS speed is an imperfect proxy, not the hidden fused reference.",
            "Sampled points within one trip are correlated; counts are whole bags.",
            "30639 has only one training date, so its between-session scale variance is not identified.",
            "Route branch, map geometry, and startup GNSS errors are separate from this longitudinal floor.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, help="Existing replay CLI; otherwise compile current source")
    parser.add_argument("--out", type=Path,
                        default=ROOT / "evaluation/covariance_scale_audit.json")
    args = parser.parse_args()
    if args.exe is None:
        executable = ROOT / "evaluation" / (
            ".covariance_audit_replay.exe" if sys.platform == "win32"
            else ".covariance_audit_replay")
        if executable.exists():
            raise FileExistsError(f"temporary audit executable already exists: {executable}")
        try:
            compile_cli(executable)
            result = audit(executable)
        finally:
            executable.unlink(missing_ok=True)
    else:
        result = audit(args.exe.resolve())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                        encoding="utf-8")
    print(json.dumps(result["coverage"], indent=2, ensure_ascii=False))
    print(args.out)


if __name__ == "__main__":
    main()
