"""Offline audit of GNSS speed/fix agreement and wheel scale by session.

Read-only diagnostics. Holdout data in this report must never calibrate runtime.
Each bag contributes one observation to session medians to avoid treating
correlated 10 Hz samples as independent trials.
"""

from __future__ import annotations

import csv
import json
import math
import statistics as stats
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))
from causal_baseline import FRONT, MASTER, REAR, ROVER, make_reference, nearest, read_run  # noqa: E402


def median_or_none(values):
    return stats.median(values) if values else None


def quantile(values, q):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values) - 1, math.ceil(q * len(values)) - 1)]


def stream(events, topic):
    rows = sorted((stamp, value / 3.6) for _, stamp, name, value in events
                  if name == topic and math.isfinite(float(value)) and 0 <= value <= 90)
    return tuple(x[0] for x in rows), tuple(x[1] for x in rows)


def score_bag(bag, metadata, quality):
    events, raw = read_run(bag)
    truth = make_reference(raw)
    master, rover = truth[MASTER], truth[ROVER]
    if not master[0] or not rover[0]:
        return None
    front, rear = stream(events, FRONT), stream(events, REAR)
    speed_diffs = []
    moving_diffs = []
    ratios = []
    scale_numer = scale_denom = 0.0
    for stamp, m in zip(*master):
        r = nearest(rover, stamp, 60_000_000)
        if r is None:
            continue
        diff = m - r
        speed_diffs.append(diff)
        if 2.0 <= (m + r) / 2 <= 16.0:
            moving_diffs.append(diff)
        if abs(diff) > 0.3:
            continue
        reference = (m + r) / 2
        if not 2.0 <= reference <= 16.0:
            continue
        f = nearest(front, stamp, 60_000_000)
        b = nearest(rear, stamp, 60_000_000)
        if f is None or b is None or abs(f - b) > 0.2:
            continue
        wheel = (f + b) / 2
        if abs(wheel - reference) > 0.35:
            continue
        ratios.append(reference / wheel)
        scale_numer += wheel * reference
        scale_denom += wheel * wheel
    q = quality.get(bag, {})
    return {
        "bag": bag,
        "vehicle": metadata[bag]["vehicle"],
        "date": metadata[bag]["start_utc"][:10],
        "master_rover_speed_pairs": len(speed_diffs),
        "moving_speed_pairs": len(moving_diffs),
        "master_minus_rover_moving_median_mps": median_or_none(moving_diffs),
        "master_rover_moving_abs_p95_mps": quantile([abs(x) for x in moving_diffs], .95),
        "master_rover_abs_gt_0_3_fraction":
            sum(abs(x) > .3 for x in speed_diffs) / len(speed_diffs) if speed_diffs else None,
        "clean_scale_pairs": len(ratios),
        "gnss_over_mean_wheel_scale_lsq": scale_numer / scale_denom if scale_denom else None,
        "gnss_over_mean_wheel_scale_median": median_or_none(ratios),
        "master_fix_jump_gt30_fraction":
            q.get("apparent_speed_gt_30mps_count", 0) / q["step_count"]
            if q.get("step_count") else None,
        "antenna_sep_err_gt5_fraction":
            q.get("pair_sep_err_gt_5m_count", 0) / q["paired_n"]
            if q.get("paired_n") else None,
        "antenna_sep_median_m": q.get("pair_distance_m_q", [None, None, None])[2],
    }


def summary(rows):
    fields = [
        "master_minus_rover_moving_median_mps",
        "master_rover_moving_abs_p95_mps",
        "master_rover_abs_gt_0_3_fraction",
        "gnss_over_mean_wheel_scale_lsq",
        "gnss_over_mean_wheel_scale_median",
        "master_fix_jump_gt30_fraction",
        "antenna_sep_err_gt5_fraction",
        "antenna_sep_median_m",
    ]
    return {"bags": len(rows), "paired_velocity_samples": sum(x["master_rover_speed_pairs"] for x in rows),
            "clean_scale_pairs": sum(x["clean_scale_pairs"] for x in rows),
            **{f"median_bag_{field}": median_or_none([r[field] for r in rows if r[field] is not None])
               for field in fields}}


if __name__ == "__main__":
    with (ROOT / "analysis/catalog/bags.csv").open(newline="", encoding="utf-8") as f:
        metadata = {r["bag"]: r for r in csv.DictReader(f)}
    quality = {r["bag"]: r for r in json.loads(
        (ROOT / "analysis/routes/quality_summary.json").read_text(encoding="utf-8"))}
    manifest = json.loads((ROOT / "tools/split_manifest.json").read_text(encoding="utf-8"))
    cohorts = [
        ("30618_2026-07-27_train", "train", "30618", "2026-07-27"),
        ("30618_2026-09-03_train", "train", "30618", "2026-09-03"),
        ("30639_2026-08-26_train", "train", "30639", "2026-08-26"),
        ("30639_2026-05-05_holdout", "holdout", "30639", "2026-05-05"),
    ]
    result = {"method": "Offline only; GNSS master/rover norm speed paired within 60ms; scales gated on both antenna/wheel agreement. No holdout values used in estimator.", "cohorts": {}}
    for key, split, vehicle, day in cohorts:
        ids = [bag for bag in manifest["representatives"][split]
               if metadata[bag]["vehicle"] == vehicle and metadata[bag]["start_utc"].startswith(day)]
        rows = []
        for bag in ids:
            row = score_bag(bag, metadata, quality)
            if row:
                rows.append(row)
                print(bag, "speed-pairs", row["master_rover_speed_pairs"],
                      "scale", row["gnss_over_mean_wheel_scale_lsq"], flush=True)
        result["cohorts"][key] = {"summary": summary(rows), "runs": rows,
                                  "excluded_without_dual_velocity": sorted(set(ids) - {x["bag"] for x in rows})}
    out = ROOT / "analysis/audit_session_reference.json"
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(out)
