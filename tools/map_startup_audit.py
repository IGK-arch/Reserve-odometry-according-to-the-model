"""Audit causal direction selection from the first three master GNSS fixes.

Uses GNSS only for permitted startup and for offline labels/validation. The
route CSV is frozen before this audit; this script never writes the map.

Run: py -3.12 tools/map_startup_audit.py
"""

from __future__ import annotations

import json
import sqlite3
import statistics
import sys
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "routes"))
from inspect_cdr import fix  # noqa: E402
from route_map_io import RouteMap, geodetic_to_enu  # noqa: E402


def first_three_fix(bag: str) -> tuple[float, float, float] | None:
    path = next((ROOT / "dataset" / "data" / bag).glob("*.db3"))
    connection = sqlite3.connect(f"file:{path.as_posix()}?mode=ro", uri=True)
    try:
        tid = connection.execute("SELECT id FROM topics WHERE name='/sensing/gnss/master/fix'").fetchone()[0]
        records = connection.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 3", (tid,)
        ).fetchall()
    finally:
        connection.close()
    if len(records) < 3:
        return None
    lla = [fix(raw)[3:6] for (raw,) in records]
    return geodetic_to_enu(*(statistics.median(point[i] for point in lla) for i in range(3)))


def main() -> None:
    route = RouteMap.from_csv(ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "route_map.csv")
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    summaries = {row["bag"]: row for row in json.loads((ROOT / "analysis" / "routes" / "summaries.json").read_text())}
    results = []
    for split in ("train", "validation", "holdout"):
        for bag in manifest["representatives"][split]:
            track = summaries[bag]["master_path"]
            if track.get("net_m", 0) < 4400:
                continue
            xyz = first_three_fix(bag)
            if xyz is None:
                continue
            expected = "out" if track["end_enu"][0] < track["start_enu"][0] else "return"
            own = route.project(*xyz[:2], direction=expected)
            other = route.project(*xyz[:2], direction="return" if expected == "out" else "out")
            chosen = own.direction if own.horizontal_distance_m <= other.horizontal_distance_m else other.direction
            results.append({
                "bag": bag, "vehicle": bag[:5], "split": split,
                "expected": expected, "chosen": chosen, "correct": chosen == expected,
                "x": xyz[0], "y": xyz[1],
                "expected_s": own.s,
                "correct_track_distance_m": own.horizontal_distance_m,
                "other_track_distance_m": other.horizontal_distance_m,
                "margin_m": other.horizontal_distance_m - own.horizontal_distance_m,
            })
    summary = {}
    for split in ("train", "validation", "holdout"):
        part = [row for row in results if row["split"] == split]
        summary[split] = {
            "full_bags": len(part),
            "correct": sum(row["correct"] for row in part),
            "accuracy": sum(row["correct"] for row in part) / len(part) if part else None,
            "wrong": [row for row in part if not row["correct"]],
        }
    summary["by_start_terminal"] = dict(Counter(
        f"{r['split']}:{'east' if r['x'] > -200 else 'west'}:{r['correct']}" for r in results
    ))
    output = ROOT / "tools" / "map_startup_audit.json"
    output.write_text(json.dumps({"summary": summary, "runs": results}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(output)


if __name__ == "__main__":
    main()
