"""Exact C++ model-only blackout ablation on frozen validation sessions.

Select clean 5 s reference windows using GNSS only in this evaluator, then
replay the permitted three inputs in SQLite receive order through the same C++
core as the ROS node. Front/rear messages are removed for each selected window.
The table feature is an explicit CLI argument; production defaults stay off.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))
from causal_baseline import CMD, FRONT, REAR, read_run  # noqa: E402
from calibrate_drive import prepare_validation_bag  # noqa: E402


def select_windows(bag: str) -> tuple[list[float], np.ndarray, np.ndarray]:
    prepared = prepare_validation_bag(bag)
    if prepared is None:
        return [], np.array([]), np.array([])
    t, g, front, rear, _cmd, good = prepared
    starts = []
    for start in np.arange(math.ceil(t[0] / 30.0) * 30.0, t[-1] - 5.1, 30.0):
        i = int(np.searchsorted(t, start))
        if i >= len(t) or abs(t[i] - start) > 0.08 or not good[i] or g[i] < 0.5:
            continue
        j = int(np.searchsorted(t, t[i] + 5.0))
        if j >= len(t) or j - i < 45 or not np.all(good[i:j + 1]):
            continue
        if np.max(np.diff(t[i:j + 1])) > 0.25:
            continue
        if abs(0.5 * (front[i] + rear[i]) - g[i]) > 0.30:
            continue
        starts.append(float(t[i]))
    return starts, t, g


def filter_blackouts(events, starts):
    codes = {FRONT: "F", REAR: "R", CMD: "C"}
    lines = []
    for receive_ns, stamp_ns, topic, value in events:
        if topic != CMD and starts:
            stamp_s = stamp_ns * 1e-9
            index = bisect.bisect_right(starts, stamp_s) - 1
            if index >= 0 and starts[index] <= stamp_s <= starts[index] + 5.12:
                continue
        lines.append(f"{receive_ns},{stamp_ns},{codes[topic]},{value}\n")
    return "".join(lines)


def replay(exe: Path, bag: str, input_csv: str, table: Path | None):
    command = [str(exe), bag.split("_", 1)[0]]
    if table is not None:
        command.append(str(table))
    result = subprocess.run(command, input=input_csv, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            check=True, timeout=180, cwd=ROOT)
    rows = {}
    for row in csv.DictReader(result.stdout.splitlines()):
        stamp_ns = int(row["stamp_ns"])
        rows[stamp_ns] = {
            "v": float(row["velocity_mps"]),
            "s": float(row["distance_m"]),
            "model_only": bool(int(row["model_only"])),
            "table_active": bool(int(row["drive_table_active"])),
            "table_used": bool(int(row["drive_table_used"])),
        }
    return rows


def nearest(rows, target_s: float):
    stamps = sorted(rows)
    target_ns = round(target_s * 1e9)
    i = bisect.bisect_left(stamps, target_ns)
    candidates = stamps[max(0, i - 1): min(len(stamps), i + 1)]
    if not candidates:
        return None
    stamp = min(candidates, key=lambda item: abs(item - target_ns))
    if abs(stamp - target_ns) > 60_000_000:
        return None
    return stamp, rows[stamp]


def truth_distance(t, g, start_s: float, end_s: float) -> float:
    middle = t[(t > start_s) & (t < end_s)]
    points = np.r_[start_s, middle, end_s]
    speeds = np.interp(points, t, g)
    return float(np.trapz(speeds, points))


def summarize(rows, horizon, model):
    selected = [row for row in rows if row["horizon_s"] == horizon]
    error_v = np.asarray([row[f"{model}_speed_error_mps"] for row in selected])
    error_s = np.asarray([row[f"{model}_distance_error_m"] for row in selected])
    return {
        "windows": len(selected),
        "bags": len({row["bag"] for row in selected}),
        "speed_rmse_mps": round(float(np.sqrt(np.mean(error_v**2))), 5),
        "speed_mae_mps": round(float(np.mean(np.abs(error_v))), 5),
        "speed_bias_mps": round(float(np.mean(error_v)), 5),
        "distance_rmse_m": round(float(np.sqrt(np.mean(error_s**2))), 5),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path,
                        default=ROOT / "evaluation" / "replay_table_optional.exe")
    parser.add_argument("--table", type=Path,
                        default=ROOT / "ros2_ws" / "src" / "tram_odometry" /
                                "assets" / "drive_accel_table.csv")
    parser.add_argument("--out", type=Path,
                        default=ROOT / "evaluation" / "table_blackout_validation.json")
    args = parser.parse_args()
    exe = args.exe.resolve()
    table = args.table.resolve()
    if not exe.is_file() or not table.is_file():
        parser.error("compile the current replay CLI and provide an existing table CSV")
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(
        encoding="utf-8"))
    scored = []
    for bag in manifest["representatives"]["validation"]:
        starts, t, g = select_windows(bag)
        if not starts:
            continue
        events, _ = read_run(bag)
        input_csv = filter_blackouts(events, starts)
        physics = replay(exe, bag, input_csv, None)
        empirical = replay(exe, bag, input_csv, table)
        if not any(row["table_active"] for row in empirical.values()):
            raise RuntimeError(f"drive table not loaded for {bag}")
        for start in starts:
            for horizon in (1.0, 3.0, 5.0):
                start_p = nearest(physics, start)
                end_p = nearest(physics, start + horizon)
                start_e = nearest(empirical, start)
                end_e = nearest(empirical, start + horizon)
                if None in (start_p, end_p, start_e, end_e):
                    continue
                end_stamp_s = end_p[0] * 1e-9
                start_stamp_s = start_p[0] * 1e-9
                end_truth_v = float(np.interp(end_stamp_s, t, g))
                ref_distance = truth_distance(t, g, start_stamp_s, end_stamp_s)
                if not end_p[1]["model_only"] or not end_e[1]["model_only"]:
                    continue
                row = {"bag": bag, "start_s": start, "horizon_s": horizon,
                       "truth_end_speed_mps": end_truth_v,
                       "truth_distance_m": ref_distance,
                       "table_used_at_end": end_e[1]["table_used"]}
                for name, beginning, ending in (
                    ("physics", start_p[1], end_p[1]),
                    ("table", start_e[1], end_e[1]),
                ):
                    row[f"{name}_speed_error_mps"] = ending["v"] - end_truth_v
                    row[f"{name}_distance_error_m"] = (
                        ending["s"] - beginning["s"] - ref_distance)
                scored.append(row)
        print(bag, "candidate", len(starts), "scored", sum(
            x["bag"] == bag for x in scored), flush=True)
    summary = {}
    for horizon in (1.0, 3.0, 5.0):
        group = [r for r in scored if r["horizon_s"] == horizon]
        if not group:
            continue
        summary[str(horizon)] = {
            "physics": summarize(scored, horizon, "physics"),
            "table": summarize(scored, horizon, "table"),
            "table_used_at_end_fraction": sum(r["table_used_at_end"] for r in group) / len(group),
        }
    result = {"protocol": "C++ Estimator in SQLite receive order; GNSS only selects clean non-overlapping 30 s validation windows and evaluates outputs; 5.12 s paired-wheel blackouts", "summary": summary,
              "windows": scored}
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("saved", args.out)


if __name__ == "__main__":
    main()
