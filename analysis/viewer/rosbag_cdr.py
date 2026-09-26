"""Minimal, dependency-free reader for the seven message types in this dataset.

ROS 2 messages in rosbag2 SQLite storage use CDR alignment relative to the
four-byte encapsulation header. This module deliberately supports only the
recorded schema; it is an audit aid, not a general ROS serializer.
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path
from typing import Iterator


class CDR:
    def __init__(self, data: bytes):
        if data[:4] != b"\x00\x01\x00\x00":
            raise ValueError(f"Unsupported CDR encapsulation: {data[:4].hex()}")
        self.data = data
        self.pos = 4

    def read(self, fmt: str, alignment: int):
        self.pos = 4 + ((self.pos - 4 + alignment - 1) // alignment) * alignment
        size = struct.calcsize("<" + fmt)
        value = struct.unpack_from("<" + fmt, self.data, self.pos)[0]
        self.pos += size
        return value

    def string(self) -> str:
        length = self.read("I", 4)
        if length > len(self.data) - self.pos:
            raise ValueError("Invalid CDR string length")
        value = self.data[self.pos : self.pos + length]
        self.pos += length
        return value.rstrip(b"\x00").decode("utf-8", errors="replace")

    def header(self) -> tuple[int, str]:
        sec = self.read("i", 4)
        nsec = self.read("I", 4)
        frame = self.string()
        return sec * 1_000_000_000 + nsec, frame


def decode(topic: str, data: bytes) -> dict:
    cdr = CDR(data)
    stamp, frame = cdr.header()
    out = {"stamp_ns": stamp, "frame_id": frame}
    if topic.endswith("bogie_velocity"):
        out["velocity_raw"] = cdr.read("d", 8)
    elif topic.endswith("driver_position_cmd"):
        out["position"] = cdr.read("b", 1)
    elif topic.endswith("/fix"):
        out["status"] = cdr.read("b", 1)
        out["service"] = cdr.read("H", 2)
        out["latitude"] = cdr.read("d", 8)
        out["longitude"] = cdr.read("d", 8)
        out["altitude"] = cdr.read("d", 8)
        out["covariance"] = tuple(cdr.read("d", 8) for _ in range(9))
        out["covariance_type"] = cdr.read("B", 1)
    elif topic.endswith("/vel"):
        out["linear"] = tuple(cdr.read("d", 8) for _ in range(3))
        out["angular"] = tuple(cdr.read("d", 8) for _ in range(3))
    else:
        raise ValueError(f"Unsupported topic {topic}")
    return out


def messages(bag_dir: Path, topics: set[str] | None = None) -> Iterator[tuple[str, int, dict]]:
    db_paths = sorted(bag_dir.glob("*.db3"))
    if not db_paths:
        raise FileNotFoundError(f"No db3 in {bag_dir}")
    for db_path in db_paths:
        conn = sqlite3.connect(f"file:{db_path.as_posix()}?mode=ro", uri=True)
        try:
            name_by_id = dict(conn.execute("SELECT id, name FROM topics"))
            selected = [tid for tid, name in name_by_id.items() if topics is None or name in topics]
            if not selected:
                continue
            placeholders = ",".join("?" for _ in selected)
            cursor = conn.execute(
                f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp",
                selected,
            )
            for topic_id, received_ns, data in cursor:
                topic = name_by_id[topic_id]
                yield topic, received_ns, decode(topic, data)
        finally:
            conn.close()
