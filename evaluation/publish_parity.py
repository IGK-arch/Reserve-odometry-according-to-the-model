#!/usr/bin/env python3
"""Compare immutable pre-merge and merged executables on all unique recordings.

No accuracy tuning: byte equality of complete deterministic replay CSVs, including
coverage and flags. Raw inputs/outputs stay outside the source release.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess

from causal_baseline import ROOT, read_run, FRONT, REAR, CMD


def sha(data):
    return hashlib.sha256(data).hexdigest()


def compare(before, after):
    result = {"identical": before == after, "before_sha256": sha(before),
              "after_sha256": sha(after), "before_rows": before.count(b"\n") - 1,
              "after_rows": after.count(b"\n") - 1}
    if before != after:
        old, new = before.splitlines(), after.splitlines()
        result["different_rows"] = sum(a != b for a, b in zip(old, new)) + abs(len(old)-len(new))
        result["first_difference"] = next(
            ({"row": i, "before": a.decode(), "after": b.decode()}
             for i, (a, b) in enumerate(zip(old, new)) if a != b), None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before-build", type=Path, required=True)
    parser.add_argument("--after-build", type=Path, required=True)
    parser.add_argument("--public-input", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("Output already exists: preserve earlier evidence")
    args.out.mkdir(parents=True)
    builds = [args.before_build.resolve(), args.after_build.resolve()]
    assets = ROOT / "ros2_ws/src/tram_odometry/assets"
    manifest = json.loads((ROOT / "tools/split_manifest.json").read_text())
    report = {"purpose": "Pre-merge e248a96 vs merged source: deterministic replay equality, not a new ROS accuracy run", "builds": [str(b) for b in builds],
              "binary_sha256": [{n: sha((b/n).read_bytes()) for n in ("replay_cli", "navigation_replay")} for b in builds],
              "source_sha256": {}, "bags": [], "public_profiles": {}}
    sources = list((ROOT / "ros2_ws/src/tram_odometry/src").glob("*.cpp"))
    sources += list((ROOT / "ros2_ws/src/tram_odometry/include/tram_odometry").glob("*.hpp"))
    sources += [Path(__file__), ROOT / "tools/split_manifest.json"]
    report["source_sha256"] = {str(p.relative_to(ROOT)): sha(p.read_bytes()) for p in sources}
    with ThreadPoolExecutor(max_workers=2) as pool:
        for split in ("train", "validation", "holdout"):
            for bag in manifest["representatives"][split]:
                events, _ = read_run(bag)
                codes = {FRONT: "F", REAR: "R", CMD: "C"}
                data = "".join(f"{a},{b},{codes[c]},{d}\n" for a,b,c,d in events).encode()
                def run(build):
                    command = [str(build/"replay_cli"), bag[:5]]
                    if bag.startswith("30618"):
                        command += [str(assets/"drive_accel_table.csv")]
                    return subprocess.run(command, input=data, stdout=subprocess.PIPE, check=True).stdout
                before, after = pool.map(run, builds)
                row = {"bag": bag, "split": split, "input_sha256": sha(data), "input_rows": len(events), **compare(before, after)}
                report["bags"].append(row)
                print(bag, row["identical"], row.get("different_rows", 0), flush=True)
                (args.out/"parity.json").write_text(json.dumps(report, indent=2)+"\n")
    profiles = {"startup": ["--gnss-mode", "startup"],
                "corrections": ["--gnss-mode", "corrections"],
                "branch": ["--gnss-mode", "corrections", "--alternate-map", str(assets/"route_map_branch_a.csv")],
                "full": ["--gnss-mode", "corrections", "--alternate-map", str(assets/"route_map_branch_a.csv"), "--elevation", str(assets/"official_elevation.csv")]}
    report["public_input_sha256"] = sha(args.public_input.read_bytes())
    for profile, options in profiles.items():
        outputs = []
        for i, build in enumerate(builds):
            output = (args.out/f"public_{profile}_{i}.csv").resolve()
            subprocess.run([str(build/"navigation_replay"), "--input", str(args.public_input.resolve()), "--output", str(output),
                            "--map", str(assets/"route_map.csv"), "--drive-table", str(assets/"drive_accel_table.csv"), *options], check=True)
            outputs.append(output.read_bytes())
        report["public_profiles"][profile] = compare(*outputs)
        print("public", profile, report["public_profiles"][profile]["identical"], flush=True)
    report["all_identical"] = all(r["identical"] for r in report["bags"]) and all(r["identical"] for r in report["public_profiles"].values())
    (args.out/"parity.json").write_text(json.dumps(report, indent=2)+"\n")
    if not report["all_identical"]:
        raise SystemExit("Differences require inspection; prior metrics cannot automatically be assigned to the merged source")


if __name__ == "__main__":
    main()
