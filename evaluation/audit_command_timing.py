"""Audit controller header ordering against wheel headers in the frozen splits.

Only the three permitted vehicle topics are read, in rosbag SQLite receive
order. This is a timing audit, not a GNSS accuracy score. Run from the repo:
    python evaluation/audit_command_timing.py
"""

from __future__ import annotations

import hashlib
import json
import sys
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "viewer"))
from rosbag_cdr import messages  # noqa: E402

FRONT = "/vehicle/front_bogie_velocity"
REAR = "/vehicle/rear_bogie_velocity"
CMD = "/vehicle/driver_position_cmd"
TOPICS = {FRONT, REAR, CMD}
MANIFEST = ROOT / "tools" / "split_manifest.json"
OUT = ROOT / "evaluation"
NS = 1_000_000_000


def seconds(ns: int) -> float:
    return round(ns / NS, 6)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def describe(event: tuple[str, int, int, float], first_receive_ns: int) -> dict:
    topic, receive_ns, stamp_ns, value = event
    return {
        "topic": "cmd" if topic == CMD else "front" if topic == FRONT else "rear",
        "receive_offset_s": seconds(receive_ns - first_receive_ns),
        "header_stamp_s": seconds(stamp_ns),
        "value": value,
    }


def audit_bag(bag: str) -> dict:
    wheel_frontier_ns = -1
    command_frontier_ns = -1
    last_command_receive_ns = -1
    last_wheel_receive_ns = -1
    first_receive_ns = -1
    recent = deque(maxlen=8)
    incidents: list[dict] = []
    counts = {
        "commands": 0,
        "older_or_equal_command_headers": 0,
        "older_command_headers_lag_le_0_15_s": 0,
        "command_lead_over_wheel_frontier_gt_0_5_s": 0,
        "large_command_header_jump_with_recent_inputs": 0,
    }
    for topic, receive_ns, msg in messages(ROOT / "dataset" / "data" / bag, TOPICS):
        if first_receive_ns < 0:
            first_receive_ns = receive_ns
        stamp_ns = int(msg["stamp_ns"])
        value = msg["position"] if topic == CMD else msg["velocity_raw"]
        event = (topic, receive_ns, stamp_ns, value)
        if topic != CMD:
            wheel_frontier_ns = max(wheel_frontier_ns, stamp_ns)
            last_wheel_receive_ns = receive_ns
        else:
            counts["commands"] += 1
            older = command_frontier_ns >= 0 and stamp_ns <= command_frontier_ns
            if older:
                counts["older_or_equal_command_headers"] += 1
                if command_frontier_ns - stamp_ns <= 150_000_000:
                    counts["older_command_headers_lag_le_0_15_s"] += 1
            lead_ns = stamp_ns - wheel_frontier_ns if wheel_frontier_ns >= 0 else 0
            if lead_ns > 500_000_000:
                counts["command_lead_over_wheel_frontier_gt_0_5_s"] += 1
                command_gap_ns = stamp_ns - command_frontier_ns if command_frontier_ns >= 0 else 0
                recent_inputs = (
                    last_command_receive_ns >= 0 and last_wheel_receive_ns >= 0
                    and receive_ns - last_command_receive_ns <= 350_000_000
                    and receive_ns - last_wheel_receive_ns <= 350_000_000
                )
                candidate = command_gap_ns > 500_000_000 and recent_inputs
                if candidate:
                    counts["large_command_header_jump_with_recent_inputs"] += 1
                incident = {
                    "bag": bag,
                    "command_index": counts["commands"],
                    "receive_offset_s": seconds(receive_ns - first_receive_ns),
                    "command_header_stamp_s": seconds(stamp_ns),
                    "notch": int(value),
                    "lead_over_seen_wheel_frontier_s": seconds(lead_ns),
                    "lead_over_seen_command_frontier_s": (
                        seconds(stamp_ns - command_frontier_ns)
                        if command_frontier_ns >= 0 else None
                    ),
                    "receive_since_previous_command_s": (
                        seconds(receive_ns - last_command_receive_ns)
                        if last_command_receive_ns >= 0 else None
                    ),
                    "receive_since_previous_wheel_s": (
                        seconds(receive_ns - last_wheel_receive_ns)
                        if last_wheel_receive_ns >= 0 else None
                    ),
                    "recent_input_header_jump": candidate,
                }
                if candidate:
                    incident["preceding_events"] = [describe(x, first_receive_ns) for x in recent]
                    incident["following_events"] = []
                incidents.append(incident)
            command_frontier_ns = max(command_frontier_ns, stamp_ns)
            last_command_receive_ns = receive_ns
        for incident in incidents[-2:]:
            if not incident["recent_input_header_jump"]:
                continue
            if len(incident["following_events"]) < 8:
                if topic != CMD or incident["command_index"] != counts["commands"]:
                    incident["following_events"].append(describe(event, first_receive_ns))
        recent.append(event)
    return {"counts": counts, "incidents": incidents}


def main() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    report = {
        "method": "Three vehicle topics in SQLite receive order; header frontiers are maximum seen stamps. A lead is a controller header more than 0.5 s ahead of the latest seen wheel header. Recent-input header jumps also require >0.5 s ahead of prior command and prior command/wheel receive age <=0.35 s. These are diagnostics, not rejection rules.",
        "thresholds_s": {"lead": 0.5, "old_command_short_lag": 0.15,
                         "recent_receive_age": 0.35},
        "inputs_sha256": {"split_manifest.json": digest(MANIFEST)},
        "splits": {},
        "affected_bags": {},
        "rejected_guard_experiment": {
            "description": "A trial guard rejecting a command >0.5 s ahead of a fresh wheel was reverted after it discarded the valid newer branch. These counts are recorded from the earlier trial and are not recomputed by this timing scan.",
            "output_counts_old_to_trial": {
                "30618_2255aade": [25060, 3934],
                "30618_40ffd323": [25538, 1707],
            },
            "post_revert": "All output CSV fields on these two bags and 30639_9c362687 matched the pre-trial corrected replay byte for byte.",
        },
    }
    for split in ("train", "validation", "holdout"):
        bag_ids = manifest["representatives"][split]
        totals = {"bags": len(bag_ids), "commands": 0,
                  "older_or_equal_command_headers": 0,
                  "older_command_headers_lag_le_0_15_s": 0,
                  "command_lead_over_wheel_frontier_gt_0_5_s": 0,
                  "large_command_header_jump_with_recent_inputs": 0}
        for bag in bag_ids:
            record = audit_bag(bag)
            for key, value in record["counts"].items():
                totals[key] += value
            if record["incidents"] or record["counts"]["older_or_equal_command_headers"]:
                report["affected_bags"][bag] = record
        report["splits"][split] = totals
        print(split, totals, flush=True)
    (OUT / "audit_command_timing.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = ["# Аудит времени команд контроллера", "",
             "Проверены три разрешённых топика в порядке SQLite-получения; `header.stamp` сравнивается с максимумом уже увиденных меток. GNSS не использовался.", "",
             "| Split | Bag | Команд | Старые/равные метки | Из них лаг ≤0,15 с | Команда опережает колесо >0,5 с | Резкий скачок при недавних входах |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for split, c in report["splits"].items():
        lines.append(f"| {split} | {c['bags']} | {c['commands']} | {c['older_or_equal_command_headers']} | {c['older_command_headers_lag_le_0_15_s']} | {c['command_lead_over_wheel_frontier_gt_0_5_s']} | {c['large_command_header_jump_with_recent_inputs']} |")
    lines.extend(["", "Три резких скачка при недавних входах:", "",
                  "| Bag | Receive offset, с | Скачок относительно предыдущей команды, с | Опережение колеса, с |",
                  "|---|---:|---:|---:|"])
    for bag, record in report["affected_bags"].items():
        for incident in record["incidents"]:
            if incident["recent_input_header_jump"]:
                lines.append(f"| {bag} | {incident['receive_offset_s']:.3f} | {incident['lead_over_seen_command_frontier_s']:.3f} | {incident['lead_over_seen_wheel_frontier_s']:.3f} |")
    lines.extend(["", "Команда с заголовком старее уже принятого теперь не перезаписывает notch. Два коротких инверсных заголовка в `30639_9c362687` имели тот же notch, что и новая команда; исправление не меняет численные выходы в затронутых bag.", "",
                  "Опережение колёсного фронта само по себе не доказывает ошибочный заголовок: оно бывает при отставании колёсного потока. Столбец резких скачков требует дополнительно скачка относительно предыдущей команды >0,5 с и получения команды и колеса в последние 0,35 с; таких случаев три на holdout. В этих bag команды и колёса приходят перемежающимися потоками меток с разницей около секунды. Пробный запрет `lead>0,5 с` при свежем колесе ошибочно сократил число выходов C++ replay `30618_2255aade` с 25060 до 3934 и `30618_40ffd323` с 25538 до 1707. Этот пробный guard откатан; текущий код сохраняет новую ветку и отвергает старые заголовки. Метрики после отката на этих двух bag и `30639_9c362687` побайтно совпали с состоянием до эксперимента.", "",
                  "В JSON приведены локальные события вокруг трёх резких скачков, а также агрегаты всех случаев опережения. Для защиты от действительно одиночного ошибочного будущего заголовка нужен отдельный буфер/проверка по соседним сообщениям; этого фикса в текущем коде нет.", ""])
    (OUT / "audit_command_timing.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
