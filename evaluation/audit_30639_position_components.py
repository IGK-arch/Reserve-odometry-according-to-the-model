"""Read-only decomposition of May 30639 position proxy errors.

No value derived here is fed into production. Counterfactual GNSS distance and
post-hoc constant scale use future reference by construction and diagnose how
much of the published error could be longitudinal, not an achievable runtime
score. The hidden fused judge pose is unavailable.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import statistics
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from causal_baseline import MASTER, ROVER, make_reference, nearest, read_run
from core_benchmark import replay_cpp
from position_proxy import MAP, ROOT, clean_reference_positions
from route_map_io import RouteMap


ORIGINAL = ROOT / "evaluation/holdout_position_deployed_proxy.csv"
EXE = ROOT / "evaluation/replay_cli.exe"


def rms(values):
    return math.sqrt(float(np.mean(np.square(values)))) if values else None


def dual_speed_integral(raw):
    streams = make_reference(raw)
    master, rover = streams[MASTER], streams[ROVER]
    pairs = []
    for stamp, master_speed in zip(*master):
        rover_speed = nearest(rover, stamp, 60_000_000)
        if rover_speed is None or abs(master_speed - rover_speed) > 0.3:
            continue
        pairs.append((stamp * 1e-9, (master_speed + rover_speed) / 2))
    if len(pairs) < 100:
        return None
    t, v = map(np.asarray, zip(*pairs))
    keep = np.r_[True, np.diff(t) > 0.001]
    t, v = t[keep], v[keep]
    dt = np.diff(t)
    cumulative = np.r_[0.0, np.cumsum((v[:-1] + v[1:]) * dt * 0.5)]
    return t, cumulative, {
        "paired_velocity_points": len(t),
        "max_paired_velocity_gap_s": float(np.max(dt)),
        "p99_paired_velocity_gap_s": float(np.quantile(dt, .99)),
        "fraction_gap_gt_0_5s": float(np.mean(dt > .5)),
    }


def nearest_index(times, target):
    i = bisect.bisect_left(times, target)
    candidates = range(max(0, i - 1), min(len(times), i + 1))
    j = min(candidates, key=lambda k: abs(times[k] - target))
    return j if abs(times[j] - target) <= .05 else None


def score(bag, route, published):
    events, raw_truth = read_run(bag)
    core = replay_cpp(EXE.resolve(), bag, events, None)
    reference = clean_reference_positions(bag)
    if reference is None:
        return None
    ref_times, master, base_ref, raw_times, raw_xyz = reference
    t0 = min(stamp for _, stamp, _, _ in events) * 1e-9
    startup = np.flatnonzero((raw_times >= t0) & (raw_times <= t0 + 5))
    if len(startup) < 3:
        return None
    third_fix_time = float(raw_times[startup[2]])
    initial = np.median(raw_xyz[startup[:3]], axis=0)
    direction = published["direction"]
    anchor = route.project(float(initial[0]), float(initial[1]), direction)
    residual = initial - np.asarray(route.sample(direction, anchor.s))
    first_stamp = min(core)
    first_distance = core[first_stamp]["distance_m"]
    truth_integral = dual_speed_integral(raw_truth)
    if truth_integral is not None:
        truth_t, truth_cum, speed_quality = truth_integral
        oracle_origin = float(np.interp(first_stamp * 1e-9, truth_t, truth_cum))
    else:
        speed_quality = {}
    map_rows = np.asarray(route.rows[direction], dtype=float)
    tree = cKDTree(map_rows[:, 1:3])
    _, closest_map_node = tree.query(master[:, :2])
    core_points = []
    for stamp, state in sorted(core.items()):
        stamp_s = stamp * 1e-9
        if stamp_s < third_fix_time:
            continue  # mirror ROS startup position gate
        i = nearest_index(ref_times, stamp_s)
        if i is None:
            continue
        progress = state["distance_m"] - first_distance
        core_points.append((stamp_s, progress, i))
    if not core_points:
        return None
    truth_points = []
    if truth_integral is not None and speed_quality["max_paired_velocity_gap_s"] <= 1.0:
        for stamp_s, progress, i in core_points:
            idx = int(np.searchsorted(truth_t, stamp_s))
            left = max(0, idx - 1)
            right = min(len(truth_t) - 1, idx)
            if min(abs(truth_t[left] - stamp_s), abs(truth_t[right] - stamp_s)) > .15:
                continue
            truth_progress = float(np.interp(stamp_s, truth_t, truth_cum) - oracle_origin)
            truth_points.append((stamp_s, progress, truth_progress, i))
    scale = None
    if len(truth_points) > 100 and abs(truth_points[-1][1]) > 100:
        scale = truth_points[-1][2] / truth_points[-1][1]

    def predicted(progress):
        position = np.asarray(route.master_to_base_enu(direction, anchor.s + progress))
        position[:2] += residual[:2] * math.exp(-max(0.0, progress) / 100.0)
        return position

    xy_sq = z_sq = along_sq = cross_sq = 0.0
    first100_xy_sq = first100_z_sq = 0.0
    first100_n = 0
    for _, progress, i in core_points:
        delta = predicted(progress) - base_ref[i]
        xy_sq += float(np.sum(delta[:2] ** 2))
        z_sq += float(delta[2] ** 2)
        sample_before = np.asarray(route.sample(direction, anchor.s + progress - 1))
        sample_after = np.asarray(route.sample(direction, anchor.s + progress + 1))
        tangent = sample_after[:2] - sample_before[:2]
        tangent /= max(float(np.linalg.norm(tangent)), 1e-9)
        along = float(np.dot(delta[:2], tangent))
        cross = float(delta[0] * -tangent[1] + delta[1] * tangent[0])
        along_sq += along * along
        cross_sq += cross * cross
        if progress <= 100:
            first100_xy_sq += float(np.sum(delta[:2] ** 2))
            first100_z_sq += float(delta[2] ** 2)
            first100_n += 1
    n = len(core_points)
    cf_baseline_sq = cf_oracle_sq = cf_scale_sq = cf_mapmatch_sq = 0.0
    for _, progress, truth_progress, i in truth_points:
        target = base_ref[i]
        cf_baseline_sq += float(np.sum((predicted(progress) - target) ** 2))
        cf_oracle_sq += float(np.sum((predicted(truth_progress) - target) ** 2))
        if scale is not None:
            cf_scale_sq += float(np.sum((predicted(progress * scale) - target) ** 2))
        closest_s = map_rows[closest_map_node[i], 0]
        cf_mapmatch_sq += float(np.sum((predicted(closest_s - anchor.s) - target) ** 2))
    cf_n = len(truth_points)
    return {
        "bag": bag, "matched_position_points": n,
        "published_base_rmse_3d_m": float(published["base_rmse_3d_m"]),
        "strict_gate_base_rmse_3d_m": math.sqrt((xy_sq + z_sq) / n),
        "xy_rmse_m": math.sqrt(xy_sq / n),
        "z_rmse_m": math.sqrt(z_sq / n),
        "along_track_rmse_m": math.sqrt(along_sq / n),
        "cross_track_rmse_m": math.sqrt(cross_sq / n),
        "first100_xy_rmse_m": math.sqrt(first100_xy_sq / first100_n) if first100_n else None,
        "first100_z_rmse_m": math.sqrt(first100_z_sq / first100_n) if first100_n else None,
        "start_projection_xy_m": anchor.horizontal_distance_m,
        "gnss_speed_counterfactual_valid": bool(cf_n > 100),
        "gnss_speed_counterfactual_points": cf_n,
        "gnss_speed_counterfactual_same_subset_baseline_rmse_m":
            math.sqrt(cf_baseline_sq / cf_n) if cf_n else None,
        "gnss_speed_counterfactual_oracle_rmse_m":
            math.sqrt(cf_oracle_sq / cf_n) if cf_n else None,
        "hindsight_constant_scale": scale,
        "hindsight_constant_scale_rmse_m":
            math.sqrt(cf_scale_sq / cf_n) if cf_n and scale is not None else None,
        "hindsight_gnss_position_map_match_rmse_m":
            math.sqrt(cf_mapmatch_sq / cf_n) if cf_n else None,
        **speed_quality,
    }


if __name__ == "__main__":
    route = RouteMap.from_csv(MAP)
    with ORIGINAL.open(newline="", encoding="utf-8") as f:
        published = {row["bag"]: row for row in csv.DictReader(f)
                     if row["bag"].startswith("30639_")}
    runs = []
    for bag, row in published.items():
        result = score(bag, route, row)
        if result:
            runs.append(result)
            print(bag, "XY", round(result["xy_rmse_m"], 2),
                  "Z", round(result["z_rmse_m"], 2),
                  "oracle", result["gnss_speed_counterfactual_oracle_rmse_m"], flush=True)
    fields = ["xy_rmse_m", "z_rmse_m", "along_track_rmse_m", "cross_track_rmse_m",
              "first100_xy_rmse_m", "first100_z_rmse_m", "start_projection_xy_m",
              "gnss_speed_counterfactual_same_subset_baseline_rmse_m",
              "gnss_speed_counterfactual_oracle_rmse_m",
              "hindsight_constant_scale_rmse_m",
              "hindsight_gnss_position_map_match_rmse_m"]
    summary = {"bags": len(runs)}
    for field in fields:
        values = [r[field] for r in runs if r[field] is not None]
        summary[f"median_bag_{field}"] = statistics.median(values) if values else None
        summary[f"bags_with_{field}"] = len(values)
    report = {"method": "Existing 30639 holdout GNSS proxy; exact same map, startup anchor, C++ distance; strict 3rd-fix position gate. Oracle speed and constant scale are post-hoc future-data diagnostics, not runtime features. Hindsight map match uses future GNSS position and is a geometry/reference floor only.", "summary": summary, "runs": runs}
    target = ROOT / "evaluation/audit_30639_position_components.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(target)
