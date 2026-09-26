"""Read-only, standard-library catalog of the supplied rosbag2 SQLite files.

Usage: python analysis/catalog/catalog_bags.py
Reads dataset/data/*/*.db3 and writes CSV/JSON/Markdown next to this script.
All SQL connections use SQLite mode=ro&immutable=1. No ROS installation needed.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
import sqlite3
import statistics
import struct
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DB_FILES = sorted((ROOT / "dataset" / "data").glob("*/*.db3"))
NS = 1_000_000_000


def cdr_align(offset: int, alignment: int) -> int:
    """CDR field alignment is relative to payload byte 4, after encapsulation."""
    payload_offset = offset - 4
    return 4 + ((payload_offset + alignment - 1) // alignment) * alignment


def q(values: list[float], p: float) -> float | None:
    if not values:
        return None
    z = sorted(values)
    u = (len(z) - 1) * p
    lo = int(u)
    hi = min(lo + 1, len(z) - 1)
    return z[lo] * (hi - u) + z[hi] * (u - lo)


def round_or_none(value: float | None, digits: int = 6) -> float | None:
    return None if value is None or not math.isfinite(value) else round(value, digits)


def stamp_and_value(topic: str, blob: bytes) -> tuple[int | None, float | int | None]:
    """Decode common ROS2 CDR header and one scalar; no third-party runtime."""
    if len(blob) < 16:
        return None, None
    # CDR encapsulation: 0001 = little endian, 0000 = big endian.
    if blob[:2] == b"\x00\x01":
        endian = "<"
    elif blob[:2] == b"\x00\x00":
        endian = ">"
    else:
        return None, None
    try:
        sec, nsec, frame_len = struct.unpack_from(endian + "iII", blob, 4)
        if not (0 <= nsec < NS and 0 < frame_len <= len(blob) - 16):
            return None, None
        off = 16 + frame_len
        stamp = sec * NS + nsec
        if topic.endswith("_bogie_velocity"):
            off = cdr_align(off, 8)
            return stamp, struct.unpack_from(endian + "d", blob, off)[0]
        if topic.endswith("driver_position_cmd"):
            return stamp, struct.unpack_from(endian + "b", blob, off)[0]
        if topic.endswith("/vel"):
            off = cdr_align(off, 8)
            vx, vy, vz = struct.unpack_from(endian + "ddd", blob, off)
            return stamp, math.sqrt(vx * vx + vy * vy + vz * vz)
        if topic.endswith("/fix"):
            # NavSatStatus: int8 status, uint16 service, then aligned doubles.
            off += 1
            off = cdr_align(off, 2)
            off += 2
            off = cdr_align(off, 8)
            latitude = struct.unpack_from(endian + "d", blob, off)[0]
            return stamp, latitude
        return stamp, None
    except (struct.error, OverflowError, ValueError):
        return None, None


class TopicStats:
    def __init__(self, name: str):
        self.name = name
        self.count = 0
        self.bag_first = self.bag_last = None
        self.header_first = self.header_last = None
        self.prev_bag = self.prev_header = None
        self.bag_dts: list[float] = []
        self.header_dts: list[float] = []
        self.lags: list[float] = []
        self.invalid_headers = 0
        self.header_nonincreasing = 0
        self.bag_same_stamp = 0
        self.values: list[float] = []
        self.nonfinite_values = 0
        self.negative_values = 0
        self.zero_values = 0
        self.value_first = self.value_last = None
        self.positions = Counter()
        self.gap_events: list[tuple[int, int, float]] = []
        self.timing_events: list[tuple[int, str, float]] = []

    def add(self, bag_ns: int, blob: bytes) -> None:
        self.count += 1
        if self.bag_first is None:
            self.bag_first = bag_ns
        self.bag_last = bag_ns
        if self.prev_bag is not None:
            dt = (bag_ns - self.prev_bag) / NS
            self.bag_dts.append(dt)
            if dt == 0:
                self.bag_same_stamp += 1
            gap_threshold = .2 if self.name.endswith("driver_position_cmd") else (.5 if self.name.startswith("/vehicle/") else 1.0)
            if dt > gap_threshold:
                self.gap_events.append((self.prev_bag, bag_ns, dt))
        self.prev_bag = bag_ns
        header_ns, value = stamp_and_value(self.name, blob)
        if header_ns is None:
            self.invalid_headers += 1
        else:
            if self.header_first is None:
                self.header_first = header_ns
            self.header_last = header_ns
            lag = (bag_ns - header_ns) / NS
            self.lags.append(lag)
            if lag < -.1 or lag > .5:
                self.timing_events.append((bag_ns, "header_future_gt_0_1_s" if lag < -.1 else "header_age_gt_0_5_s", lag))
            if self.prev_header is not None:
                hdt = (header_ns - self.prev_header) / NS
                self.header_dts.append(hdt)
                if hdt <= 0:
                    self.header_nonincreasing += 1
                    self.timing_events.append((bag_ns, "header_nonincreasing", hdt))
            self.prev_header = header_ns
        if value is not None:
            if not math.isfinite(value):
                self.nonfinite_values += 1
            else:
                fv = float(value)
                self.values.append(fv)
                if self.value_first is None:
                    self.value_first = fv
                self.value_last = fv
                if fv < 0:
                    self.negative_values += 1
                if fv == 0:
                    self.zero_values += 1
                if self.name.endswith("driver_position_cmd"):
                    self.positions[int(value)] += 1

    def row(self, bag: str, bag_begin: int | None) -> dict:
        dts = self.bag_dts
        hdts = self.header_dts
        lags = self.lags
        vals = self.values
        r = {
            "bag": bag,
            "topic": self.name,
            "messages": self.count,
            "first_bag_ns": self.bag_first,
            "last_bag_ns": self.bag_last,
            "first_header_ns": self.header_first,
            "last_header_ns": self.header_last,
            "first_offset_s": round_or_none((self.bag_first - bag_begin) / NS) if self.bag_first is not None else None,
            "last_offset_s": round_or_none((self.bag_last - bag_begin) / NS) if self.bag_last is not None else None,
            "span_s": round_or_none((self.bag_last - self.bag_first) / NS) if self.bag_first is not None else None,
            "bag_dt_p50_s": round_or_none(q(dts, .5)),
            "bag_dt_p95_s": round_or_none(q(dts, .95)),
            "bag_dt_p99_s": round_or_none(q(dts, .99)),
            "bag_dt_max_s": round_or_none(max(dts) if dts else None),
            "bag_gap_gt_0_2_s": sum(x > .2 for x in dts),
            "bag_gap_gt_0_5_s": sum(x > .5 for x in dts),
            "bag_gap_gt_1_s": sum(x > 1 for x in dts),
            "bag_gap_gt_5_s": sum(x > 5 for x in dts),
            "bag_same_stamp": self.bag_same_stamp,
            "header_dt_p50_s": round_or_none(q(hdts, .5)),
            "header_dt_p99_s": round_or_none(q(hdts, .99)),
            "header_dt_max_s": round_or_none(max(hdts) if hdts else None),
            "header_nonincreasing": self.header_nonincreasing,
            "lag_p01_s": round_or_none(q(lags, .01)),
            "lag_p50_s": round_or_none(q(lags, .5)),
            "lag_p95_s": round_or_none(q(lags, .95)),
            "lag_p99_s": round_or_none(q(lags, .99)),
            "lag_max_s": round_or_none(max(lags) if lags else None),
            "lag_min_s": round_or_none(min(lags) if lags else None),
            "header_future_count": sum(x < 0 for x in lags),
            "invalid_headers": self.invalid_headers,
            "value_first": round_or_none(self.value_first),
            "value_last": round_or_none(self.value_last),
            "value_min": round_or_none(min(vals) if vals else None),
            "value_p50": round_or_none(q(vals, .5)),
            "value_p99": round_or_none(q(vals, .99)),
            "value_max": round_or_none(max(vals) if vals else None),
            "nonfinite_values": self.nonfinite_values,
            "negative_values": self.negative_values,
            "zero_values": self.zero_values,
            "controller_histogram": json.dumps(dict(sorted(self.positions.items())), separators=(",", ":")) if self.positions else "",
        }
        return r


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def main() -> None:
    if not DB_FILES:
        raise SystemExit("No db3 files in dataset/data")
    bag_rows = []
    topic_rows = []
    gap_rows = []
    timing_rows = []
    hashes = defaultdict(list)
    metadata_count_mismatches = []
    for i, db in enumerate(DB_FILES, 1):
        bag = db.parent.name
        digest = hashlib.sha256()
        with db.open("rb") as f:
            while chunk := f.read(1024 * 1024):
                digest.update(chunk)
        sha = digest.hexdigest()
        hashes[sha].append(bag)
        # URI uses forward-slash file:/// URL and never creates a WAL/journal.
        conn = sqlite3.connect(db.resolve().as_uri() + "?mode=ro&immutable=1", uri=True)
        conn.execute("PRAGMA query_only=ON")
        topic_names = {tid: name for tid, name in conn.execute("SELECT id,name FROM topics")}
        stats = {name: TopicStats(name) for name in topic_names.values()}
        bag_begin = bag_end = None
        messages = 0
        for tid, timestamp, blob in conn.execute("SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id"):
            messages += 1
            if bag_begin is None:
                bag_begin = timestamp
            bag_end = timestamp
            stats[topic_names[tid]].add(timestamp, blob)
        conn.close()
        metadata = (db.parent / "metadata.yaml").read_text(encoding="utf-8")
        m_duration = re.search(r"^  duration:\s*\n    nanoseconds: (\d+)", metadata, re.M)
        m_start = re.search(r"^  starting_time:\s*\n    nanoseconds_since_epoch: (\d+)", metadata, re.M)
        m_count = re.search(r"^  message_count: (\d+)", metadata, re.M)
        declared_duration = int(m_duration.group(1)) if m_duration else None
        declared_start = int(m_start.group(1)) if m_start else None
        declared_count = int(m_count.group(1)) if m_count else None
        if declared_count != messages or declared_start != bag_begin or declared_duration != bag_end - bag_begin:
            metadata_count_mismatches.append(bag)
        rows_here = [s.row(bag, bag_begin) for s in stats.values()]
        topic_rows.extend(rows_here)
        for s in stats.values():
            gap_rows.extend({
                "bag": bag, "topic": s.name,
                "gap_start_offset_s": round((start - bag_begin) / NS, 6),
                "gap_end_offset_s": round((end - bag_begin) / NS, 6),
                "gap_s": round(gap, 6),
            } for start, end, gap in s.gap_events)
            timing_rows.extend({
                "bag": bag, "topic": s.name,
                "bag_offset_s": round((when - bag_begin) / NS, 6),
                "kind": kind, "value_s": round(value, 6),
            } for when, kind, value in s.timing_events)
        by_topic = {r["topic"]: r for r in rows_here}
        get = lambda name, key: by_topic.get(name, {}).get(key, 0) or 0
        gf = "/sensing/gnss/master/fix"
        gr = "/sensing/gnss/rover/fix"
        gnss_count = sum(s.count for s in stats.values() if s.name.startswith("/sensing/gnss/"))
        bag_rows.append({
            "bag": bag,
            "vehicle": bag.split("_")[0],
            "db_sha256": sha,
            "db_bytes": db.stat().st_size,
            "start_bag_ns": bag_begin,
            "end_bag_ns": bag_end,
            "start_utc": datetime.fromtimestamp(bag_begin / NS, timezone.utc).isoformat() if bag_begin is not None else None,
            "duration_s": round_or_none((bag_end - bag_begin) / NS) if bag_begin is not None else None,
            "messages": messages,
            "topic_count": len(stats),
            "front_count": get("/vehicle/front_bogie_velocity", "messages"),
            "rear_count": get("/vehicle/rear_bogie_velocity", "messages"),
            "cmd_count": get("/vehicle/driver_position_cmd", "messages"),
            "gnss_count": gnss_count,
            "master_fix_count": get(gf, "messages"),
            "rover_fix_count": get(gr, "messages"),
            "master_fix_first_offset_s": get(gf, "first_offset_s") if get(gf, "messages") else None,
            "master_fix_last_offset_s": get(gf, "last_offset_s") if get(gf, "messages") else None,
            "rover_fix_first_offset_s": get(gr, "first_offset_s") if get(gr, "messages") else None,
            "rover_fix_last_offset_s": get(gr, "last_offset_s") if get(gr, "messages") else None,
            "front_max_gap_s": get("/vehicle/front_bogie_velocity", "bag_dt_max_s"),
            "rear_max_gap_s": get("/vehicle/rear_bogie_velocity", "bag_dt_max_s"),
            "cmd_max_gap_s": get("/vehicle/driver_position_cmd", "bag_dt_max_s"),
            "metadata_match": bag not in metadata_count_mismatches,
        })
        if i % 10 == 0 or i == len(DB_FILES):
            print(f"{i}/{len(DB_FILES)} {bag} {messages} messages", flush=True)

    write_csv(OUT / "bags.csv", bag_rows)
    write_csv(OUT / "topics.csv", topic_rows)
    write_csv(OUT / "gaps.csv", gap_rows)
    write_csv(OUT / "timing_anomalies.csv", timing_rows)
    duplicate_groups = [group for group in hashes.values() if len(group) > 1]
    duplicate_groups.sort(key=lambda group: group[0])
    (OUT / "duplicate_groups.json").write_text(json.dumps(duplicate_groups, indent=2, ensure_ascii=False), encoding="utf-8")

    topic_names = sorted({r["topic"] for r in topic_rows})
    summary = {
        "bag_count": len(bag_rows),
        "vehicles": dict(Counter(r["vehicle"] for r in bag_rows)),
        "total_duration_h": round(sum(r["duration_s"] for r in bag_rows) / 3600, 3),
        "duration_s_min_med_max": [round_or_none(min(r["duration_s"] for r in bag_rows)), round_or_none(q([r["duration_s"] for r in bag_rows], .5)), round_or_none(max(r["duration_s"] for r in bag_rows))],
        "total_messages": sum(r["messages"] for r in bag_rows),
        "total_db_bytes": sum(r["db_bytes"] for r in bag_rows),
        "topic_names": topic_names,
        "topic_totals": {t: sum(r["messages"] for r in topic_rows if r["topic"] == t) for t in topic_names},
        "no_gnss_bags": [r["bag"] for r in bag_rows if r["gnss_count"] == 0],
        "gnss_bags": sum(r["gnss_count"] > 0 for r in bag_rows),
        "exact_duplicate_groups": duplicate_groups,
        "exact_duplicate_excess_bags": sum(len(g) - 1 for g in duplicate_groups),
        "metadata_mismatches": metadata_count_mismatches,
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
