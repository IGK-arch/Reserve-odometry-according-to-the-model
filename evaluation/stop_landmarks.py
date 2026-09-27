"""Experimental train-only stop landmarks for along-track drift correction.

This module builds the train-only catalog and performs offline evaluation. The
ROS node implements its own causal stop detector in C++. Runtime detection
reads only wheel speeds, controller positions and the estimator output. GNSS
is decoded here only for train landmark building and held-out scoring.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from causal_baseline import causal_wheel_baseline, read_run
from core_benchmark import DEFAULT_EXE, replay_cpp
from position_proxy import MAP, ROOT, clean_reference_positions, nearest_ref
from route_map_io import RouteMap

STOPS_CSV = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "stops.csv"
SPLIT_JSON = ROOT / "tools" / "split_manifest.json"
TRAIN_CANDIDATES_CSV = ROOT / "evaluation" / "train_stop_candidates.csv"


@dataclass(frozen=True)
class Stop:
    start_ns: int
    detected_ns: int
    end_ns: int
    duration_s: float
    distance_m: float
    speed_before_mps: float


def detect_stops(baseline, core, *, speed_threshold_mps=0.11,
                 min_dwell_s=8.0, max_gap_s=0.35):
    """Causal detection: event is visible only after min_dwell_s at rest.

    The function returns completed intervals for convenience; ``detected_ns``
    is the first time at which the online algorithm has enough history to emit
    the landmark update. All corrections use only that instant or later.
    """
    ordered = sorted((row for row in baseline if row.stamp_ns in core),
                     key=lambda row: row.stamp_ns)
    stops = []
    begin = previous = None
    speed_before = 0.0
    peak_before = 0.0
    detected_ns = None
    for row in ordered:
        moving = (row.speed_mps > speed_threshold_mps or
                  (not row.front_used and not row.rear_used))
        gap = (row.stamp_ns - previous.stamp_ns) * 1e-9 if previous else 0.0
        if moving or gap > max_gap_s:
            if begin is not None and detected_ns is not None and previous is not None:
                stops.append(Stop(begin.stamp_ns, detected_ns, previous.stamp_ns,
                                  (previous.stamp_ns - begin.stamp_ns) * 1e-9,
                                  core[detected_ns]["distance_m"], speed_before))
            begin = detected_ns = None
            if moving:
                peak_before = max(peak_before, row.speed_mps)
        else:
            if begin is None:
                begin = row
                speed_before = peak_before
                peak_before = 0.0
            if detected_ns is None and (row.stamp_ns - begin.stamp_ns) * 1e-9 >= min_dwell_s:
                detected_ns = row.stamp_ns
        previous = row
    if begin is not None and detected_ns is not None and previous is not None:
        stops.append(Stop(begin.stamp_ns, detected_ns, previous.stamp_ns,
                          (previous.stamp_ns - begin.stamp_ns) * 1e-9,
                          core[detected_ns]["distance_m"], speed_before))
    return stops


def startup_anchor(events, reference, route):
    if reference is None:
        return None
    _, _, _, raw_times, raw_xyz = reference
    t0 = min(stamp for _, stamp, _, _ in events) * 1e-9
    startup = raw_xyz[(raw_times >= t0) & (raw_times <= t0 + 5.0)]
    if len(startup) < 3:
        return None
    initial = np.median(startup[:3], axis=0)
    direction = "out" if initial[0] > -200 else "return" if initial[0] < -4400 else None
    if direction is None:
        return None
    projection = route.project(float(initial[0]), float(initial[1]), direction)
    if projection.horizontal_distance_m > 50.0:
        return None
    return direction, projection.s, initial


def read_split(split):
    manifest = json.loads(SPLIT_JSON.read_text(encoding="utf-8"))
    return manifest["representatives"][split]


def stop_truth_s(stops, reference, route, direction):
    times, positions, *_ = reference
    result = []
    for stop in stops:
        point = nearest_ref(times, positions, stop.detected_ns * 1e-9)
        if point is None:
            result.append(None)
        else:
            projection = route.project(float(point[0]), float(point[1]), direction)
            result.append(projection.s if projection.horizontal_distance_m < 50.0 else None)
    return result


def train_candidates(route, exe):
    all_stops = []
    audit = []
    for bag in read_split("train"):
        events, _ = read_run(bag)
        if not events:
            continue
        reference = clean_reference_positions(bag)
        anchor = startup_anchor(events, reference, route)
        if anchor is None:
            continue
        direction, start_s, _ = anchor
        core = replay_cpp(exe, bag, events)
        if not core:
            continue
        first_distance = core[min(core)]["distance_m"]
        baseline = causal_wheel_baseline(events)
        stops = detect_stops(baseline, core)
        truth_s = stop_truth_s(stops, reference, route, direction)
        matched = 0
        for stop, s in zip(stops, truth_s):
            if s is None:
                continue
            # Do not learn recording-start dwell. Each learned landmark needs
            # a real approach in this recording and substantial route progress.
            predicted = start_s + stop.distance_m - first_distance
            if (stop.speed_before_mps < 2.0 or
                    abs(predicted - start_s) < 100.0 or
                    stop.detected_ns - min(x[1] for x in events) < 60_000_000_000):
                continue
            all_stops.append((direction, s, bag, stop.duration_s, predicted))
            matched += 1
        audit.append((bag, direction, len(stops), matched))
    return all_stops, audit


def cluster_stops(rows, *, radius_m=18.0, min_distinct_bags=3,
                  max_mad_m=8.0):
    """1D density clusters, guarding against isolated traffic-light stops."""
    result = []
    for direction in ("out", "return"):
        ordered = sorted((r for r in rows if r[0] == direction), key=lambda r: r[1])
        clusters = []
        for row in ordered:
            if clusters and row[1] - clusters[-1][-1][1] <= radius_m:
                clusters[-1].append(row)
            else:
                clusters.append([row])
        for members in clusters:
            bags = {r[2] for r in members}
            samples = [r[1] for r in members]
            center = statistics.median(samples)
            mad = statistics.median(abs(x - center) for x in samples)
            if len(bags) >= min_distinct_bags and mad <= max_mad_m:
                result.append({"direction": direction, "s": center,
                               "bag_count": len(bags), "event_count": len(members),
                               "mad_m": mad, "min_s": min(samples),
                               "max_s": max(samples)})
    return sorted(result, key=lambda r: (r["direction"], r["s"]))


def load_catalog(path=STOPS_CSV):
    with path.open(newline="", encoding="utf-8") as handle:
        return [{key: float(value) if key not in ("direction", "bag_count", "event_count")
                 else int(value) if key in ("bag_count", "event_count") else value
                 for key, value in row.items()} for row in csv.DictReader(handle)]


def nearest_landmark(catalog, direction, predicted_s, gate_m):
    candidates = [item for item in catalog
                  if item["direction"] == direction and
                  abs(item["s"] - predicted_s) <= gate_m]
    if not candidates:
        return None
    candidates.sort(key=lambda item: abs(item["s"] - predicted_s))
    # A positional gate alone cannot disambiguate nearby stops if accumulated
    # drift reaches half their separation.
    if len(candidates) >= 2 and abs(candidates[1]["s"] - predicted_s) - abs(candidates[0]["s"] - predicted_s) < 6.0:
        return None
    return candidates[0]


def position_errors(route, direction, s, progress, residual_xy, ref):
    mapped = np.asarray(route.sample(direction, s))
    mapped[:2] += residual_xy * math.exp(-max(0.0, progress) / 100.0)
    delta = mapped - ref
    before = np.asarray(route.sample(direction, s - 1.0))
    after = np.asarray(route.sample(direction, s + 1.0))
    tangent = after - before
    tangent /= max(1e-8, np.linalg.norm(tangent))
    return float(np.linalg.norm(delta)), float(np.dot(delta, tangent))


def evaluate_bag(bag, route, exe, catalog, *, gate_m=12.0, gain=0.8,
                 max_correction_m=10.0):
    events, _ = read_run(bag)
    if not events:
        return {"bag": bag, "status": "empty"}
    reference = clean_reference_positions(bag)
    anchor = startup_anchor(events, reference, route)
    if anchor is None:
        return {"bag": bag, "status": "no_startup_anchor"}
    direction, start_s, initial = anchor
    core = replay_cpp(exe, bag, events)
    if not core:
        return {"bag": bag, "status": "no_core"}
    baseline = causal_wheel_baseline(events)
    stops = detect_stops(baseline, core)
    first_distance = core[min(core)]["distance_m"]
    t0_ns = min(row[1] for row in events)
    stops = [stop for stop in stops
             if stop.speed_before_mps >= 2.0 and
             abs(stop.distance_m - first_distance) >= 100.0 and
             stop.detected_ns - t0_ns >= 60_000_000_000]
    stops.sort(key=lambda x: x.detected_ns)
    truth_s = stop_truth_s(stops, reference, route, direction)
    ref_times, ref_xyz, *_ = reference
    residual_xy = (initial - np.asarray(route.sample(direction, start_s)))[:2]
    correction = 0.0
    accepted = []
    used_landmarks = set()
    eligible = 0
    stop_idx = 0
    before_sq = after_sq = before_along_sq = after_along_sq = 0.0
    n = 0
    last_before = last_after = last_before_along = last_after_along = math.nan
    last_ref_stamp = None
    max_excess_error = 0.0
    segment_sums = {}
    for stamp, state in sorted(core.items()):
        while stop_idx < len(stops) and stops[stop_idx].detected_ns <= stamp:
            stop = stops[stop_idx]
            current_s = start_s + stop.distance_m - first_distance + correction
            landmark = nearest_landmark(catalog, direction, current_s, gate_m)
            if landmark is not None:
                eligible += 1
                key = (direction, landmark["s"])
                delta = landmark["s"] - current_s
                if key not in used_landmarks and abs(delta) <= max_correction_m:
                    true_s = truth_s[stop_idx]
                    old_err = abs(current_s - true_s) if true_s is not None else math.nan
                    applied = gain * delta
                    correction += applied
                    new_err = abs(current_s + applied - true_s) if true_s is not None else math.nan
                    accepted.append((landmark["s"], applied, old_err, new_err,
                                     true_s, stamp))
                    used_landmarks.add(key)
            stop_idx += 1
        ref = nearest_ref(ref_times, ref_xyz, stamp * 1e-9)
        if ref is None:
            continue
        s0 = start_s + state["distance_m"] - first_distance
        progress = state["distance_m"] - first_distance
        eb, ab = position_errors(route, direction, s0, progress, residual_xy, ref)
        ea, aa = position_errors(route, direction, s0 + correction,
                                 progress + correction, residual_xy, ref)
        before_sq += eb * eb
        after_sq += ea * ea
        before_along_sq += ab * ab
        after_along_sq += aa * aa
        last_before, last_after = eb, ea
        last_before_along, last_after_along = ab, aa
        max_excess_error = max(max_excess_error, ea - eb)
        segment = segment_sums.setdefault(len(accepted), [0, 0.0, 0.0])
        segment[0] += 1
        segment[1] += eb * eb
        segment[2] += ea * ea
        last_ref_stamp = stamp
        n += 1
    if n < 100:
        return {"bag": bag, "status": "no_reference", "direction": direction,
                "detected_stops": len(stops), "eligible_stops": eligible,
                "applied_snaps": len(accepted)}
    harmful = sum(new_err > old_err + 2.0 for _, _, old_err, new_err, _, _ in accepted
                  if math.isfinite(old_err))
    false_match = sum(abs(landmark - true_s) > gate_m for landmark, _, _, _, true_s, _ in accepted
                      if true_s is not None)
    meaningful_segments = [v for v in segment_sums.values() if v[0] >= 100]
    segment_rmse_deltas = [math.sqrt(v[2] / v[0]) - math.sqrt(v[1] / v[0])
                           for v in meaningful_segments]
    return {"bag": bag, "status": "scored", "direction": direction,
            "matched": n, "detected_stops": len(stops),
            "eligible_stops": eligible, "applied_snaps": len(accepted),
            "harmful_snaps": harmful, "false_match_snaps": false_match,
            "largest_abs_correction_m": max((abs(row[1]) for row in accepted), default=0.0),
            "net_correction_m": correction,
            "max_pointwise_error_increase_m": max_excess_error,
            "worse_segment_count": sum(delta > 0.05 for delta in segment_rmse_deltas),
            "max_segment_rmse_increase_m": max(segment_rmse_deltas, default=0.0),
            "seconds_since_last_snap_to_end":
                (last_ref_stamp - accepted[-1][5]) * 1e-9 if accepted else math.nan,
            "before_rmse_3d_m": math.sqrt(before_sq / n),
            "after_rmse_3d_m": math.sqrt(after_sq / n),
            "before_rmse_along_m": math.sqrt(before_along_sq / n),
            "after_rmse_along_m": math.sqrt(after_along_sq / n),
            "before_end_error_3d_m": last_before,
            "after_end_error_3d_m": last_after,
            "before_end_along_m": last_before_along,
            "after_end_along_m": last_after_along,
            "snap_audit": json.dumps([{"s": r[0], "correction": r[1],
                                       "before_error": r[2], "after_error": r[3],
                                       "true_s": r[4], "detected_ns": r[5]}
                                      for r in accepted]),
            }


def write_evaluation(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def write_catalog(rows, path=STOPS_CSV):
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = ["direction", "s", "bag_count", "event_count", "mad_m", "min_s", "max_s"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def write_candidates(rows, path=TRAIN_CANDIDATES_CSV):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("direction", "s", "bag", "dwell_s", "model_predicted_s"))
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, default=DEFAULT_EXE)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--split", choices=("train", "validation", "holdout"))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--catalog", type=Path, default=STOPS_CSV)
    parser.add_argument("--vehicle-prefix", choices=("30618", "30639"))
    parser.add_argument("--gate-m", type=float, default=12.0)
    parser.add_argument("--gain", type=float, default=0.8)
    args = parser.parse_args()
    exe = args.exe.resolve()
    if args.build:
        from core_benchmark import compile_cli
        compile_cli(exe)
    route = RouteMap.from_csv(MAP)
    if args.split:
        catalog = load_catalog(args.catalog)
        rows = []
        for bag in read_split(args.split):
            if args.vehicle_prefix and not bag.startswith(args.vehicle_prefix):
                continue
            row = evaluate_bag(bag, route, exe, catalog,
                               gate_m=args.gate_m, gain=args.gain)
            rows.append(row)
            if row["status"] == "scored":
                print(f"{bag}: stops={row['detected_stops']} snap={row['applied_snaps']} "
                      f"RMSE {row['before_rmse_3d_m']:.2f}->{row['after_rmse_3d_m']:.2f}m "
                      f"end {row['before_end_error_3d_m']:.2f}->{row['after_end_error_3d_m']:.2f}m",
                      flush=True)
            else:
                print(f"{bag}: {row['status']}", flush=True)
        scored = [row for row in rows if row["status"] == "scored"]
        if scored:
            print(f"{args.split}: scored={len(scored)}/{len(rows)} "
                  f"snap={sum(row['applied_snaps'] for row in scored)} "
                  f"harmful={sum(row['harmful_snaps'] for row in scored)} "
                  f"false={sum(row['false_match_snaps'] for row in scored)} "
                  f"median_RMSE={statistics.median(row['before_rmse_3d_m'] for row in scored):.2f}"
                  f"->{statistics.median(row['after_rmse_3d_m'] for row in scored):.2f}m")
        if args.out:
            write_evaluation(rows, args.out)
    else:
        rows, audit = train_candidates(route, exe)
        write_candidates(rows)
        catalog = cluster_stops(rows, min_distinct_bags=6)
        print("Train stop candidates", len(rows), "from", len(audit), "bags")
        print("Train detected/matched", sum(r[2] for r in audit), sum(r[3] for r in audit))
        for item in catalog:
            print(item)
        write_catalog(catalog)


if __name__ == "__main__":
    main()
