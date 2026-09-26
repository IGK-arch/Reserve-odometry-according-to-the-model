"""Build the compact in-conversation sensor-case visualization.

The destination is the current Codex thread visualization directory. Data are
read from dataset files, and a literal HTML template is populated with small
JSON arrays; no network or ROS installation is required for extraction.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from rosbag_cdr import messages


ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = Path(__file__).with_name("tram_sensor_cases.template.html")
DEST = Path(r"C:\Users\admin\.codex\visualizations\2026\09\25\01a0d809-7ec1-7d30-a36c-268959130f37\tram-sensor-cases.html")
CASES = [
    ("30618_33bec73f", "Задняя: пробуксовка", 99, 115),
    ("30618_2050d396", "Задняя: юз при торможении", 395, 408),
    ("30639_50956d6e", "Передняя: пробуксовка", 91, 103),
    ("30618_2050d396", "Обе тележки: юз", 440, 447),
    ("30618_4d487b0d", "GNSS master: замирание", 100, 135),
    ("30618_4d487b0d", "Колёса: задержка ≈ 0,93 с", 143, 163),
    ("30639_e4379d7f", "GNSS rover: выброс", 195, 203),
    ("30639_3b3d9eb8", "Задняя: пропуск 73,5 с", 410, 495),
]
TOPIC_TO_SERIES = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
    "/vehicle/driver_position_cmd": "cmd",
    "/sensing/gnss/master/vel": "master",
    "/sensing/gnss/rover/vel": "rover",
}


def load_case(bag_id: str, label: str, start: float, end: float) -> dict:
    bag = ROOT / "dataset" / "data" / bag_id
    all_msgs: list[tuple[str, dict]] = []
    origin_ns = None
    for topic, _, msg in messages(bag, set(TOPIC_TO_SERIES)):
        if topic.startswith("/vehicle/"):
            stamp = msg["stamp_ns"]
            origin_ns = stamp if origin_ns is None else min(origin_ns, stamp)
        all_msgs.append((topic, msg))
    if origin_ns is None:
        raise ValueError(f"No vehicle messages: {bag_id}")
    series: dict[str, list[list[float]]] = {key: [] for key in TOPIC_TO_SERIES.values()}
    for topic, msg in all_msgs:
        t = (msg["stamp_ns"] - origin_ns) / 1e9
        if t < start - 2 or t > end + 2:
            continue
        key = TOPIC_TO_SERIES[topic]
        if key in ("front", "rear"):
            v = msg["velocity_raw"] / 3.6
        elif key == "cmd":
            v = msg["position"]
        else:
            v = float(np.linalg.norm(msg["linear"]))
        series[key].append([round(t, 3), round(v, 3)])
    for key in series:
        series[key].sort(key=lambda p: p[0])
    return {"id": bag_id, "label": label, "start": start, "end": end, "series": series}


def main() -> None:
    cases = [load_case(*case) for case in CASES]
    template = TEMPLATE.read_text(encoding="utf-8")
    payload = json.dumps(cases, ensure_ascii=False, separators=(",", ":"))
    html = template.replace("__CASES_JSON__", payload)
    if "__CASES_JSON__" in html:
        raise ValueError("Template replacement failed")
    DEST.parent.mkdir(parents=True, exist_ok=True)
    DEST.write_text(html, encoding="utf-8")
    print(f"{DEST} {DEST.stat().st_size:,} bytes")


if __name__ == "__main__":
    main()
