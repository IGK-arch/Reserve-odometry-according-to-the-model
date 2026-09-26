"""Summarize fixed 5-second paired-wheel blackout diagnostics."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"


def main():
    sources = [
        ("validation 30618", EVAL / "table_blackout_validation.json", True),
        ("holdout 30618", EVAL / "audit_blackout_holdout_30618.json", True),
        ("holdout 30639", EVAL / "audit_blackout_holdout_30639.json", False),
    ]
    lines = ["# Пятисекундное отключение обеих тележек", "",
             "Скрипт `table_blackout_ablation.py` удаляет сообщения обеих тележек в фиксированных окнах 5,12 с; оцениватель получает только контроллер и прежнее состояние. GNSS выбирает чистые окна по заранее заданным правилам и оценивает результат после replay. Начала окон идут с шагом 30 с, но внутри одной даты коррелированы.", "",
             "| Split / трамвай | Окна / bag | Физика: RMSE скорости, м/с | С таблицей, м/с | Физика: RMSE пути, м | С таблицей, м | Победы таблицы по bag (скорость/путь) |", "|---|---:|---:|---:|---:|---:|---:|"]
    result = {}
    for label, path, deployed in sources:
        d = json.loads(path.read_text(encoding="utf-8"))
        five = d["summary"]["5.0"]
        per_bag = defaultdict(list)
        for r in d["windows"]:
            if r["horizon_s"] == 5.0:
                per_bag[r["bag"]].append(r)
        speed_wins = 0
        distance_wins = 0
        for rs in per_bag.values():
            speed_wins += sum(r["table_speed_error_mps"] ** 2 for r in rs) < sum(r["physics_speed_error_mps"] ** 2 for r in rs)
            distance_wins += sum(r["table_distance_error_m"] ** 2 for r in rs) < sum(r["physics_distance_error_m"] ** 2 for r in rs)
        result[label] = {"deployed_table": deployed, "windows": five["physics"]["windows"],
                         "bags": len(per_bag), "table_bag_wins_speed": speed_wins,
                         "table_bag_wins_distance": distance_wins,
                         "physics": five["physics"], "table": five["table"]}
        lines.append(f"| {label} | {five['physics']['windows']}/{len(per_bag)} | {five['physics']['speed_rmse_mps']:.3f} | {five['table']['speed_rmse_mps']:.3f} | {five['physics']['distance_rmse_m']:.3f} | {five['table']['distance_rmse_m']:.3f} | {speed_wins}/{len(per_bag)} / {distance_wins}/{len(per_bag)} |")
    lines += ["", "Таблица реально включена лишь для 30618. Строка holdout 30639 с таблицей — диагностический гипотетический сценарий, а не фактический режим; она не используется для выбора режима по holdout. Валидация 30639 отсутствует. Поскольку 30618 validation и каждый holdout-трамвай представлены одной датой, эти числа не задают надёжный интервал для новых сессий.", ""]
    (EVAL / "audit_blackout_summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (EVAL / "audit_blackout_summary.md").write_text("\n".join(lines), encoding="utf-8")
    print(EVAL / "audit_blackout_summary.md")


if __name__ == "__main__":
    main()
