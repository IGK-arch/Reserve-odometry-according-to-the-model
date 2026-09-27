#!/usr/bin/env python3
"""Reproduce sparse-GNSS trip diagnostics without fitting any model or map.

All four modes permit both antennas during the first 30 seconds and keep GNSS
corrections ENABLED. The 'startup' mode then has a GNSS blackout. Other modes
add alternating single-antenna, two-second windows every 120 seconds, either
unchanged, delayed by one second, or frozen at that window's first XYZ payload.

Scoring uses cleaned paired GNSS and the provided antenna TF as an approximate
base_link reference, NOT the official fused-localization reference or score.
Offline interpolation and cleaning apply only to that scoring proxy.
Requires NumPy, SciPy, a C++17 compiler, the original dataset, and navigation_replay.

Example from the repository root:
  python3 evaluation/navigation_gnss_stress.py --dataset-root dataset/data \
    --exe build/navigation_replay --outdir evaluation/runs/navigation_gnss_stress
Add --split validation for all validation representatives, or --bags ID1 ID2.
The generated report.json may be copied into evaluation/results after review.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))
from reference_benchmark import INPUT_CODES, sha256  # noqa: E402
from rosbag_cdr import messages  # noqa: E402

DEFAULT_BAGS = (
    ("30618_8158f0b0", "train"), ("30618_76e1f9c7", "train"),
    ("30618_68847170", "train"), ("30618_a869780d", "validation"),
    ("30618_2050d396", "validation"),
)
MODES = ("startup", "sparse", "delayed", "frozen")
MODE_LABELS = {
    "startup": "Both GNSS antennas allowed first 30 seconds with corrections enabled, then blackout",
    "sparse": "Same first 30 seconds, then clean sparse single-antenna windows",
    "delayed": "Same first 30 seconds; sparse windows delayed by 1 second in receive order",
    "frozen": "Same first 30 seconds; repeat first XYZ of each sparse window with fresh timestamps",
}
LIMITATIONS = [
    "Paired GNSS + provided antenna TF is only an approximate base_link reference; these are not official fused-localization scores.",
    "Reference antenna interpolation and cleanup are offline scoring operations; replay receives only selected original GNSS and vehicle inputs.",
    "These trip diagnostics are not an independent population accuracy claim.",
    "Relative odom outputs are excluded from absolute MGRS RMSE and reported as missing absolute coverage; a missing pose is not a zero-error pose.",
    "When sparse windows miss a fork, scalar wheel/controller inputs cannot identify its branch; the default branch can remain wrong.",
    "The fixed-payload freeze guard requires two healthy wheel channels; broader smoothly drifting or jointly wrong GNSS is not certified.",
    "Clean corrections can increase proxy RMSE on already accurate trips; uniform improvement is not guaranteed.",
]
PROJECTION_SOURCE = r'''
#include <iomanip>
#include <iostream>
#include "tram_odometry/geo_projection.hpp"
int main() {
  tram_odometry::geo::EnuDatum datum({55.810367065,37.462266845,168.3794});
  char operation; double x,y,z;
  std::cout << std::setprecision(17);
  while (std::cin >> operation >> x >> y >> z) {
    tram_odometry::geo::Point3 result;
    if (operation == 'G') result = datum.toEnu({x,y,z});
    else if (operation == 'E') result = datum.enuToMgrs37Ucb({x,y,z});
    else return 1;
    std::cout << result.x << ' ' << result.y << ' ' << result.z << '\n';
  }
}
'''


def build_projection(outdir: Path, compiler: str) -> Path:
    """Compile a small offline adapter against the separately tested datum code."""
    source = outdir / "projection_cli.cpp"
    source.write_text(PROJECTION_SOURCE, encoding="utf-8")
    executable = outdir / "projection_cli"
    subprocess.run([compiler, "-std=c++17", "-O2", "-I",
                    str(ROOT / "ros2_ws/src/tram_odometry/include"),
                    str(source), "-o", str(executable)], check=True)
    return executable


def project(points, operation: str, executable: Path):
    payload = "".join(f"{operation} {x} {y} {z}\n" for x, y, z in points)
    result = subprocess.run([str(executable)], input=payload, text=True,
                            capture_output=True, check=True)
    return np.loadtxt(result.stdout.splitlines(), ndmin=2)


def nearest(source, query):
    """Nearest header; ties choose earlier, with all timestamp differences in ns."""
    index = np.clip(np.searchsorted(source, query), 1, len(source) - 1)
    index -= abs(source[index - 1] - query) <= abs(source[index] - query)
    return index, abs(source[index] - query)


def read_permitted(bag: Path):
    """Never request the fused-localization topic, even if a bag contains it."""
    rows = []
    for topic, receive, message in messages(bag, set(INPUT_CODES)):
        code = INPUT_CODES[topic]
        if code in ("MV", "RV"):
            continue
        if code == "C":
            values = (message["position"],)
        elif code in ("F", "R"):
            values = (message["velocity_raw"],)
        else:
            values = (message["latitude"], message["longitude"],
                      message["altitude"], message["status"])
        rows.append((receive, message["stamp_ns"], code, *values))
    if not rows:
        raise ValueError(f"No permitted inputs in {bag}")
    return rows


def scoring_proxy(inputs, projection: Path):
    """Separate offline antenna proxy; it is never appended to replay input."""
    fixes = {}
    for code in ("MF", "RF"):
        rows = [row for row in inputs if row[2] == code and row[6] >= 0
                and np.isfinite(row[3:6]).all()]
        rows.sort(key=lambda row: row[1])
        stamps = np.asarray([row[1] for row in rows], dtype=np.int64)
        if len(stamps) < 2:
            raise ValueError(f"At least two valid {code} fixes are needed for the scoring proxy")
        unique = np.r_[True, np.diff(stamps) > 0]
        if unique.sum() < 2:
            raise ValueError(f"At least two distinct {code} timestamps are needed for the scoring proxy")
        positions = np.asarray([row[3:6] for row in rows], dtype=float)
        fixes[code] = stamps[unique], project(positions[unique], "G", projection)
    master_t, master = fixes["MF"]
    rover_t, rover = fixes["RF"]
    _, age = nearest(rover_t, master_t)
    epoch = min(master_t[0], rover_t[0])
    paired = np.array([np.interp(master_t - epoch, rover_t - epoch, rover[:, j])
                       for j in range(3)]).T
    delta = paired - master
    separation = np.linalg.norm(delta, axis=1)
    from scipy.ndimage import median_filter

    local = median_filter(master, size=(9, 1), mode="nearest")
    good = ((age < 150_000_000) & (separation > 11) & (separation < 14)
            & (np.linalg.norm(master - local, axis=1) < 3))
    if good.sum() < 2:
        raise ValueError("Insufficient cleaned paired GNSS for the scoring proxy")
    base = project(master[good] + 9.873 * delta[good] / separation[good, None], "E", projection)
    base[:, 2] -= 3
    return master_t[good], base


def select_inputs(inputs, mode: str):
    origin = min(row[0] for row in inputs)
    selected, frozen = [], {}
    for receive, stamp, code, *values in inputs:
        if code in ("MF", "RF"):
            elapsed_ns = stamp - origin
            if elapsed_ns > 30_000_000_000:
                window, phase_ns = divmod(elapsed_ns - 120_000_000_000, 120_000_000_000)
                expected = "MF" if window % 2 == 0 else "RF"
                if mode == "startup" or window < 0 or phase_ns >= 2_000_000_000 or code != expected:
                    continue
                if mode == "delayed":
                    receive += 1_000_000_000
                elif mode == "frozen":
                    frozen.setdefault((window, code), values[:3])
                    values = frozen[window, code] + values[3:]
        selected.append((receive, stamp, code, *values))
    selected.sort(key=lambda row: row[0])
    return selected


def replay(executable: Path, input_path: Path, output_path: Path, vehicle: int):
    assets = ROOT / "ros2_ws/src/tram_odometry/assets"
    command = [str(executable), "--input", str(input_path), "--output", str(output_path),
               "--vehicle", str(vehicle),
               "--map", str(assets / "route_map.csv"),
               "--alternate-map", str(assets / "route_map_branch_a.csv"),
               "--elevation", str(assets / "official_elevation.csv"),
               "--drive-table", str(assets / "drive_accel_table.csv"),
               "--gnss-mode", "corrections"]
    subprocess.run(command, check=True)
    with output_path.open(newline="", encoding="utf-8") as stream:
        output = list(csv.DictReader(stream))
    if not output:
        raise ValueError(f"No navigation output for {input_path}")
    return output


def score(output, reference_t, reference_p):
    required = {'frame_id', 'mapped', 'anchored'}
    if any(not required.issubset(row) for row in output):
        raise ValueError('Replay CSV needs frame_id, mapped, anchored; rebuild navigation_replay and rerun')
    stamps = np.asarray([int(row["stamp_ns"]) for row in output], dtype=np.int64)
    positions = np.asarray([[float(row[key]) for key in ("x", "y", "z")] for row in output])
    state = np.asarray([[float(row[key]) for key in ("velocity_mps", "distance_m")] for row in output])
    index, age = nearest(reference_t, stamps)
    valid = np.asarray([row["position_valid"] == "1" for row in output])
    matched = age <= 50_000_000
    absolute = np.asarray([row['frame_id'] == 'mgrs_37UCB' for row in output])
    relative = np.asarray([row['frame_id'] == 'odom' for row in output])
    valid &= matched & absolute & np.isfinite(positions).all(axis=1)
    errors = positions[valid] - reference_p[index[valid]]
    result = {
        "n": int(valid.sum()),
        "scoring_status": "scored" if valid.any() else "no_absolute_position_matches",
        "publication_count": len(output),
        "proxy_matched_count": int(matched.sum()),
        "absolute_frame_output_count": int(absolute.sum()),
        "relative_frame_output_count": int(relative.sum()),
        "matched_relative_frame_count": int((matched & relative).sum()),
        "unscored_proxy_matched_count": int(matched.sum() - valid.sum()),
        "absolute_position_coverage": float(valid.sum() / matched.sum()) if matched.any() else None,
        "rmse_3d_m": float(np.sqrt(np.mean(np.sum(errors * errors, axis=1)))) if len(errors) else None,
        "end_proxy_error_m": float(np.linalg.norm(errors[-1])) if len(errors) else None,
        "clamped_outputs": sum(row["clamped"] == "1" for row in output),
        "corrections": int(output[-1]["gnss_corrections"]),
        "rejected": int(output[-1]["gnss_rejected"]),
        "branch_switches": int(output[-1]["branch_switches"]),
        "last_branch": output[-1]["branch"],
    }
    return result, stamps, positions, state


def provenance(dataset_root: Path, executable: Path, bags):
    files = [
        "evaluation/navigation_gnss_stress.py", "evaluation/navigation_replay.cpp",
        "evaluation/reference_benchmark.py", "analysis/viewer/rosbag_cdr.py",
        "ros2_ws/src/tram_odometry/src/navigation.cpp", "ros2_ws/src/tram_odometry/src/estimator.cpp",
        "ros2_ws/src/tram_odometry/include/tram_odometry/navigation.hpp",
        "ros2_ws/src/tram_odometry/include/tram_odometry/route_map.hpp",
        "ros2_ws/src/tram_odometry/include/tram_odometry/elevation_profile.hpp",
        "ros2_ws/src/tram_odometry/include/tram_odometry/estimator.hpp",
        "ros2_ws/src/tram_odometry/include/tram_odometry/geo_projection.hpp",
        "ros2_ws/src/tram_odometry/assets/route_map.csv",
        "ros2_ws/src/tram_odometry/assets/route_map_branch_a.csv",
        "ros2_ws/src/tram_odometry/assets/official_elevation.csv",
        "ros2_ws/src/tram_odometry/assets/drive_accel_table.csv",
    ]
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True)
    return {
        "git_commit": commit.stdout.strip() if commit.returncode == 0 else None,
        "artifacts_sha256": {name: sha256(ROOT / name) for name in files},
        "executable_sha256": sha256(executable),
        "projection_adapter_sha256": hashlib.sha256(PROJECTION_SOURCE.encode()).hexdigest(),
        "bag_files_sha256": {bag: {path.name: sha256(path) for path in sorted((dataset_root / bag).glob("*.db3"))}
                             for bag, _ in bags},
        "selected_bags": [{"bag": bag, "split": split} for bag, split in bags],
        "command": sys.argv,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=ROOT / "dataset/data")
    parser.add_argument("--exe", type=Path, default=ROOT / "build/navigation_replay")
    parser.add_argument("--outdir", type=Path, default=ROOT / "evaluation/runs/navigation_gnss_stress")
    parser.add_argument("--cxx", default="c++", help="C++17 compiler for the small offline projection adapter")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--bags", nargs="*", help="Explicit bag IDs; defaults to the five documented diagnostic trips")
    selection.add_argument("--split", choices=("train", "validation", "holdout"),
                           help="All representative bags from tools/split_manifest.json")
    args = parser.parse_args(argv)
    args.exe = args.exe.resolve()
    args.outdir = args.outdir.resolve()
    args.dataset_root = args.dataset_root.resolve()
    if not args.exe.is_file():
        parser.error(f"Build navigation_replay first or provide --exe: {args.exe}")
    manifest = json.loads((ROOT / "tools/split_manifest.json").read_text(encoding="utf-8"))
    split_by_bag = {bag: group["split"] for group in manifest["groups"] for bag in group["bags"]}
    if args.bags is not None:
        if not args.bags:
            parser.error("--bags requires at least one bag ID")
        bags = [(bag, split_by_bag.get(bag, "unassigned")) for bag in dict.fromkeys(args.bags)]
    elif args.split:
        bags = [(bag, args.split) for bag in manifest["representatives"][args.split]]
    else:
        bags = list(DEFAULT_BAGS)
    for bag, _ in bags:
        if not (args.dataset_root / bag).is_dir():
            parser.error(f"Missing original dataset bag: {args.dataset_root / bag}")
    args.outdir.mkdir(parents=True, exist_ok=True)
    projection = build_projection(args.outdir, args.cxx)
    report = {
        "note": "Paired-GNSS base_link proxy diagnostic; NOT official fused-reference score. No fitting.",
        "window_rule": "All modes use corrections during first 30s; then alternating MF/RF 2s windows at 120,240,...s, except startup blackout.",
        "matching": "Nearest proxy header within inclusive 50ms; ties choose earlier; integer ns. Score only valid mgrs_37UCB outputs. Coverage denominator includes all proxy-matched publications, including relative/invalid outputs.",
        "mode_labels": MODE_LABELS, "limitations": LIMITATIONS, "bags": [],
    }
    for bag, split in bags:
        vehicle = int(bag.split("_", 1)[0])
        try:
            inputs = read_permitted(args.dataset_root / bag)
            reference_t, reference_p = scoring_proxy(inputs, projection)
        except ValueError as error:
            report["bags"].append({"bag": bag, "split": split, "status": "skipped", "reason": str(error)})
            print(f"Skipped {bag}: {error}", flush=True)
            continue
        row = {"bag": bag, "split": split, "vehicle": vehicle, "proxy_count": len(reference_t), "modes": {}}
        initial = None
        for mode in MODES:
            input_path = args.outdir / f"{bag}_{mode}_input.csv"
            with input_path.open("w", newline="", encoding="utf-8") as stream:
                csv.writer(stream).writerows(select_inputs(inputs, mode))
            output = replay(args.exe, input_path, args.outdir / f"{bag}_{mode}_output.csv", vehicle)
            result, stamps, positions, state = score(output, reference_t, reference_p)
            if initial is None:
                initial = stamps, positions, state
            if not np.array_equal(stamps, initial[0]):
                raise ValueError("GNSS fault injection unexpectedly changed vehicle publication stamps")
            result["identical_wheel_state_to_startup"] = bool(np.array_equal(state, initial[2]))
            if mode == "delayed":
                result["max_position_difference_to_startup_m"] = float(np.linalg.norm(positions - initial[1], axis=1).max())
            row["modes"][mode] = result
            print(bag, mode, json.dumps(result), flush=True)
        report["bags"].append(row)
    report["provenance"] = provenance(args.dataset_root, args.exe, bags)
    destination = args.outdir / "report.json"
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Saved {destination}")
    return 0 if any("modes" in row for row in report["bags"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
