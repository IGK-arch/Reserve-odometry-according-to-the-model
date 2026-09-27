#!/usr/bin/env python3
"""Compare two Navigation binaries on identical causal startup-GNSS inputs.

Both executables must implement evaluation/navigation_replay.cpp. The candidate
must also accept --stops. Build the baseline from commit 1e919c8 and the
candidate from the current source. Requires the original bags, NumPy, SciPy
and a C++17 compiler. The paired-GNSS truth is used only after both replays.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess

import numpy as np

import navigation_gnss_stress as stress


def replay(executable: Path, source: Path, dest: Path, bag: str, assets: Path,
           *, use_stops: bool):
    command = [str(executable), "--input", str(source), "--output", str(dest),
               "--vehicle", bag.split("_", 1)[0],
               "--map", str(assets / "route_map.csv"),
               "--alternate-map", str(assets / "route_map_branch_a.csv"),
               "--elevation", str(assets / "official_elevation.csv"),
               "--drive-table", str(assets / "drive_accel_table.csv"),
               "--gnss-mode", "corrections"]
    if use_stops:
        command += ["--stops", str(assets / "stops.csv")]
    subprocess.run(command, check=True)
    with dest.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-exe", required=True, type=Path)
    parser.add_argument("--candidate-exe", required=True, type=Path)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--split", choices=("train", "validation"), default="validation")
    parser.add_argument("--outdir", required=True, type=Path)
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--windows", nargs="+", type=float, default=[1.5, 30.0])
    parser.add_argument("--mode", choices=("startup", "sparse"), default="startup",
                        help="Sparse follows navigation_gnss_stress.py: first 30s and 2s windows")
    parser.add_argument("--bags", nargs="*", help="Optional IDs within the selected split")
    parser.add_argument("--origin", choices=("receive", "vehicle-header"), default="receive",
                        help="Validation report uses first receive; train sanity used first vehicle header")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    assets = root / "ros2_ws/src/tram_odometry/assets"
    outdir = args.outdir.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    projection = stress.build_projection(outdir, args.cxx)
    if os.name == "nt" and not projection.exists() and projection.with_suffix(".exe").exists():
        projection = projection.with_suffix(".exe")
    manifest = json.loads((root / "tools/split_manifest.json").read_text(encoding="utf-8"))
    bags = [b for b in manifest["representatives"][args.split] if b.startswith("30618_")]
    if args.bags is not None:
        unknown = set(args.bags) - set(bags)
        if unknown:
            parser.error(f"Unknown 30618 bag(s) in {args.split}: {sorted(unknown)}")
        bags = [b for b in bags if b in args.bags]
    results = []
    for bag in bags:
        permitted = stress.read_permitted(args.dataset_root.resolve() / bag)
        try:
            truth_t, truth_p = stress.scoring_proxy(permitted, projection)
        except ValueError as error:
            results.append({"bag": bag, "status": "no_scoring_proxy", "reason": str(error)})
            continue
        t0 = (min(row[0] for row in permitted) if args.origin == "receive" else
              min(row[1] for row in permitted if row[2] in ("C", "F", "R")))
        for window in ([None] if args.mode == "sparse" else args.windows):
            if window is not None and (not np.isfinite(window) or window < 0):
                parser.error("--windows must contain finite, nonnegative seconds")
            if args.mode == "sparse":
                selected = stress.select_inputs(permitted, "sparse")
                label = "sparse"
            else:
                selected = [row for row in permitted if row[2] in ("C", "F", "R") or
                            row[1] <= t0 + round(window * 1e9)]
                label = f"{window:g}"
            source = outdir / f"{bag}_{label}_input.csv"
            with source.open("w", newline="", encoding="utf-8") as stream:
                csv.writer(stream).writerows(selected)
            variants = []
            for name, exe, use_stops in (("baseline", args.baseline_exe, False),
                                         ("candidate", args.candidate_exe, True)):
                output = replay(exe.resolve(), source, outdir / f"{bag}_{label}_{name}.csv",
                                bag, assets, use_stops=use_stops)
                metric, stamps, positions, state = stress.score(output, truth_t, truth_p)
                variants.append((metric, stamps, positions, state, output))
            a, b = variants
            if not np.array_equal(a[1], b[1]) or not np.array_equal(a[3], b[3]):
                raise ValueError(f"Unequal output stamps or speed/distance: {bag}, {label}")
            if a[0]["n"] != b[0]["n"]:
                raise ValueError(f"Unequal position masks: {bag}, {label}")
            result = {"bag": bag, "gnss_mode": args.mode,
                      "baseline": a[0], "candidate": b[0],
                      "max_position_change_m": float(np.linalg.norm(a[2] - b[2], axis=1).max()),
                      "stop_corrections": int(b[4][-1]["stop_corrections"])}
            if window is not None:
                result["startup_gnss_s"] = window
            results.append(result)
            print(bag, label, a[0]["rmse_3d_m"], b[0]["rmse_3d_m"], flush=True)
            (outdir / "stop_navigation_ab.json").write_text(
                json.dumps(results, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
