"""Physics vs hypothetical full-table on natural 30639 holdout model-only outputs.

This is a post-hoc diagnostic on already-inspected holdout; it does not tune
or change the deployed estimator. Run: py -3.12 evaluation/audit_natural_model_only.py
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

from causal_baseline import make_reference, read_run
from core_benchmark import reference_at
from table_blackout_ablation import filter_blackouts, replay

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"
EXE = EVAL / "replay_cli.exe"
TABLE = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "drive_accel_table.csv"


def rmse(rows, field):
    return math.sqrt(sum(r[field] for r in rows) / sum(r["n"] for r in rows))


def score_bag(bag):
    events, raw_truth = read_run(bag)
    truth = make_reference(raw_truth)
    input_csv = filter_blackouts(events, [])
    physics = replay(EXE.resolve(), bag, input_csv, None)
    table = replay(EXE.resolve(), bag, input_csv, TABLE.resolve())
    if set(physics) != set(table):
        raise ValueError(f"output stamp sets differ: {bag}")
    episode_start = None
    episode_max_duration = 0.0
    episodes = 0
    last_stamp = None
    accum = defaultdict(lambda: [0, 0.0, 0.0, 0])
    mismatch = 0
    for stamp in sorted(physics):
        p, t = physics[stamp], table[stamp]
        if p["model_only"] != t["model_only"]:
            mismatch += 1
        if p["model_only"]:
            if episode_start is None or last_stamp is None or stamp - last_stamp > 200_000_000:
                episode_start = stamp
                episodes += 1
            elapsed = (stamp - episode_start) / 1e9
            episode_max_duration = max(episode_max_duration, elapsed)
            regime = ("onset_0to1s" if elapsed < 1 else
                      "onset_1to3s" if elapsed < 3 else
                      "onset_3to5s" if elapsed < 5 else "onset_after5s")
            stages = ["model_only", regime]
        else:
            episode_start = None
            stages = ["normal"]
        last_stamp = stamp
        ref = reference_at(truth, stamp)
        if ref is None:
            continue
        for stage in stages:
            a = accum[stage]
            a[0] += 1
            a[1] += (p["v"] - ref) ** 2
            a[2] += (t["v"] - ref) ** 2
            a[3] += t["table_used"]
    out = []
    for stage, (n, pss, tss, used) in sorted(accum.items()):
        out.append({"bag": bag, "stage": stage, "n": n,
                    "physics_sse": pss, "table_sse": tss,
                    "physics_rmse_mps": math.sqrt(pss / n),
                    "table_rmse_mps": math.sqrt(tss / n),
                    "table_used_fraction": used / n,
                    "model_only_episodes": episodes,
                    "longest_episode_s": episode_max_duration,
                    "model_only_status_mismatches": mismatch})
    return out


def main():
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    bags = [b for b in manifest["representatives"]["holdout"] if b.startswith("30639_")]
    rows = []
    for bag in bags:
        rows.extend(score_bag(bag))
        print(bag, flush=True)
    with (EVAL / "audit_natural_model_only_bags.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = []
    for stage in sorted({r["stage"] for r in rows}):
        chosen = [r for r in rows if r["stage"] == stage]
        n = sum(r["n"] for r in chosen)
        summary.append({"stage": stage, "bags": len(chosen), "matched": n,
                        "physics_rmse_mps": rmse(chosen, "physics_sse"),
                        "table_rmse_mps": rmse(chosen, "table_sse"),
                        "table_bag_wins": sum(r["table_rmse_mps"] < r["physics_rmse_mps"] for r in chosen),
                        "table_used_fraction": sum(r["table_used_fraction"] * r["n"] for r in chosen) / n})
    (EVAL / "audit_natural_model_only.json").write_text(json.dumps({"split": "holdout", "vehicle": "30639",
        "deployed_mode": "physics only", "summary": summary}, indent=2) + "\n", encoding="utf-8")
    lines = ["# 30639: естественный режим без обеих свежих тележек", "",
             "Исправленный C++ replay на всех десяти bag holdout 30639. Физическая модель — фактический режим; полная таблица на 30639 — гипотетический вариант. GNSS сравнивает выходы только после replay. Holdout уже использовался в разработке: эти числа диагностические, не независимый критерий выбора новой функции.", "",
             "| Условие | Bag | GNSS метки | RMSE физика → полная таблица, м/с | Победы таблицы по bag | Доля меток с активным использованием таблицы |", "|---|---:|---:|---:|---:|---:|"]
    for r in summary:
        lines.append(f"| {r['stage']} | {r['bags']} | {r['matched']} | {r['physics_rmse_mps']:.4f} → {r['table_rmse_mps']:.4f} | {r['table_bag_wins']}/{r['bags']} | {r['table_used_fraction']:.1%} |")
    lines += ["", "## По bag: естественные model-only метки", "",
              "| Bag | Метки | RMSE физика → полная таблица, м/с | Длина самой длинной серии, с |", "|---|---:|---:|---:|"]
    for r in rows:
        if r["stage"] == "model_only":
            lines.append(f"| {r['bag']} | {r['n']} | {r['physics_rmse_mps']:.4f} → {r['table_rmse_mps']:.4f} | {r['longest_episode_s']:.2f} |")
    lines += ["", "Естественные сбои могут быть короткими и не похожи на искусственные пятисекундные окна; если после включения таблицы на 30639 потребуется новое правило gating, его следует оценивать на новой независимой сессии.", ""]
    (EVAL / "audit_natural_model_only.md").write_text("\n".join(lines), encoding="utf-8")
    print(EVAL / "audit_natural_model_only.md")


if __name__ == "__main__":
    main()
