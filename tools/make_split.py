"""Freeze a leakage-resistant split of the supplied bags.

The unit of allocation is a recording date for each vehicle, and SHA-256
identical SQLite bags remain in the same set. This deliberately tests transfer
to another recording session; it is not a random per-message split.

Run from any directory: py -3.12 tools/make_split.py
"""

from __future__ import annotations

import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "analysis" / "catalog" / "bags.csv"
DESTINATION = ROOT / "tools" / "split_manifest.json"

# All six recording-date/vehicle sessions are assigned explicitly. The map
# uses only the `train` sessions. Validation has the known August 26 slips;
# holdout contains another date for each vehicle, including long sensor gaps.
SESSION_SPLITS = {
    ("30618", "2026-07-27"): "train",
    ("30618", "2026-09-03"): "train",
    ("30618", "2026-08-26"): "validation",
    ("30618", "2026-08-10"): "holdout",
    ("30639", "2026-08-26"): "train",
    ("30639", "2026-05-05"): "holdout",
}


def make_manifest() -> dict:
    with CATALOG.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    groups: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        groups[row["db_sha256"]].append(row)

    records = []
    for sha256, same_bytes in sorted(groups.items()):
        sessions = {(r["vehicle"], r["start_utc"][:10]) for r in same_bytes}
        splits = {SESSION_SPLITS[session] for session in sessions}
        if len(splits) != 1:
            raise ValueError(f"SHA group crosses split boundaries: {sha256}")
        records.append({
            "sha256": sha256,
            "split": splits.pop(),
            "vehicle": same_bytes[0]["vehicle"],
            "date": same_bytes[0]["start_utc"][:10],
            "bags": sorted(r["bag"] for r in same_bytes),
            "representative": min(r["bag"] for r in same_bytes),
            "has_master_fix": any(int(r["master_fix_count"]) > 0 for r in same_bytes),
            "has_rover_fix": any(int(r["rover_fix_count"]) > 0 for r in same_bytes),
        })
    by_split = {
        split: sorted(record["representative"] for record in records if record["split"] == split)
        for split in ("train", "validation", "holdout")
    }
    counts = Counter(r["split"] for r in records)
    return {
        "schema_version": 1,
        "basis": "Whole vehicle/date sessions; identical db_sha256 bags grouped",
        "catalog": "analysis/catalog/bags.csv",
        "sessions": [
            {"vehicle": vehicle, "date": date, "split": split}
            for (vehicle, date), split in sorted(SESSION_SPLITS.items())
        ],
        "unique_group_count": len(records),
        "unique_group_counts": dict(sorted(counts.items())),
        "representatives": by_split,
        "groups": records,
    }


if __name__ == "__main__":
    manifest = make_manifest()
    DESTINATION.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(DESTINATION)
    print(manifest["unique_group_counts"])
