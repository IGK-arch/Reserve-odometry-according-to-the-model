"""Estimate per-bogie wheel scale from *training* GNSS only.

The observed bag values are numerically km/h. This calibrates the extra
dimensionless multiplier on top of /3.6, with whole-bag medians so long
correlated runs do not dominate. No runtime GNSS dependency is introduced.
"""

from __future__ import annotations

import json
import math
import statistics
from pathlib import Path

from causal_baseline import FRONT, MASTER, REAR, ROVER, make_reference, nearest, read_run

ROOT = Path(__file__).resolve().parents[1]


def stream_from_events(events, topic):
    rows = sorted(
        (stamp, value / 3.6)
        for _, stamp, name, value in events
        if name == topic and math.isfinite(float(value)) and 0 <= value <= 90
    )
    return tuple(p[0] for p in rows), tuple(p[1] for p in rows)


def estimate_bag(bag):
    events, truth_raw = read_run(bag)
    truth = make_reference(truth_raw)
    master, rover = truth[MASTER], truth[ROVER]
    if not master[0] or not rover[0]:
        return None
    front = stream_from_events(events, FRONT)
    rear = stream_from_events(events, REAR)
    if not front[0] or not rear[0]:
        return None
    numer = {FRONT: 0.0, REAR: 0.0}
    denom = {FRONT: 0.0, REAR: 0.0}
    count = 0
    for stamp, m in zip(*master):
        r = nearest(rover, stamp, 60_000_000)
        if r is None or abs(m - r) > 0.3:
            continue
        reference = 0.5 * (m + r)
        if not 2.0 <= reference <= 16.0:
            continue
        f = nearest(front, stamp, 60_000_000)
        b = nearest(rear, stamp, 60_000_000)
        if f is None or b is None or abs(f - b) > 0.20:
            continue
        if abs((f + b) / 2 - reference) > 0.35:
            continue
        for key, wheel in ((FRONT, f), (REAR, b)):
            numer[key] += wheel * reference
            denom[key] += wheel * wheel
        count += 1
    if count < 200:
        return None
    return {
        "bag": bag,
        "n": count,
        "front_scale": numer[FRONT] / denom[FRONT],
        "rear_scale": numer[REAR] / denom[REAR],
    }


def main():
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    rows = [estimate_bag(bag) for bag in manifest["representatives"]["train"]]
    rows = [row for row in rows if row is not None]
    result = {"source": "train split only", "bag_count": len(rows), "vehicles": {}}
    for vehicle in ("30618", "30639"):
        subset = [row for row in rows if row["bag"].startswith(vehicle)]
        if not subset:
            continue
        result["vehicles"][vehicle] = {
            "bags": len(subset),
            "points": sum(row["n"] for row in subset),
            "front_scale": statistics.median(row["front_scale"] for row in subset),
            "rear_scale": statistics.median(row["rear_scale"] for row in subset),
            "front_minmax": [min(row["front_scale"] for row in subset), max(row["front_scale"] for row in subset)],
            "rear_minmax": [min(row["rear_scale"] for row in subset), max(row["rear_scale"] for row in subset)],
        }
    out = ROOT / "evaluation" / "wheel_calibration_train.json"
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
