"""Raw-bag sensitivity of speed scores to GNSS receiver and driving regime.

Replays the corrected C++ estimator from the three permitted inputs. GNSS is
decoded only by this offline scorer. Diagnostic acceleration classes use
GNSS at +/-0.5 s; no future data enters the estimator.

Run: python evaluation/audit_reference.py
Writes audit_reference_rows.csv, audit_reference.json and audit_reference.md.
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

from causal_baseline import (CMD, FRONT, MASTER, REAR, ROVER,
                             causal_wheel_baseline, make_reference, nearest, read_run)
from core_benchmark import DEFAULT_EXE, reference_at, replay_cpp

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"
TABLE = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "drive_accel_table.csv"


def wheel_disagreement(events):
    wheels = {FRONT: (-1, math.nan), REAR: (-1, math.nan)}
    flagged = {}
    for _, stamp, topic, value in events:
        if topic in wheels and stamp > wheels[topic][0] and 0 <= value <= 150:
            wheels[topic] = (stamp, value / 3.6)
        elif topic == CMD:
            a, b = wheels[FRONT], wheels[REAR]
            good = (0 <= stamp - a[0] <= 250_000_000 and
                    0 <= stamp - b[0] <= 250_000_000)
            flagged[stamp] = bool(good and abs(a[1] - b[1]) > 0.5)
    return flagged


def score_bag(split, bag):
    events, raw_truth = read_run(bag)
    truth = make_reference(raw_truth)
    baseline = {r.stamp_ns: r for r in causal_wheel_baseline(events)}
    table = TABLE if bag.startswith("30618_") else None
    core = replay_cpp(DEFAULT_EXE.resolve(), bag, events, table)
    wheel_flags = wheel_disagreement(events)
    common = sorted(baseline.keys() & core.keys())
    accum = defaultdict(lambda: [0, 0.0, 0.0])
    counts = defaultdict(int)
    for stamp in common:
        m = nearest(truth[MASTER], stamp)
        r = nearest(truth[ROVER], stamp)
        if m is None and r is None:
            counts["neither"] += 1
            continue
        if m is not None and r is not None:
            counts["both"] += 1
            if abs(m - r) > 0.3:
                counts["dual_disagree_gt_0p3"] += 1
        else:
            counts["one_only"] += 1
        refs = {}
        if m is not None:
            refs["master_only"] = m
        if r is not None:
            refs["rover_only"] = r
        if m is not None and r is not None and abs(m - r) <= 0.3:
            refs["dual_agree"] = (m + r) / 2
        if "dual_agree" in refs:
            refs["published_composite"] = refs["dual_agree"]
        elif m is None or r is None:
            refs["published_composite"] = m if m is not None else r
        if "published_composite" in refs:
            counts["published_matches"] += 1
            ref = refs["published_composite"]
            if ref < 0.5:
                regime = "stop_ref_lt_0p5"
            else:
                early = reference_at(truth, stamp - 500_000_000)
                late = reference_at(truth, stamp + 500_000_000)
                slope = late - early if early is not None and late is not None else None
                if slope is None:
                    regime = "moving_unknown_slope"
                elif slope > 0.15:
                    regime = "accel_ref_gt_0p15"
                elif slope < -0.15:
                    regime = "brake_ref_lt_minus_0p15"
                else:
                    regime = "steady_moving"
            strata = ["all", regime]
            if core[stamp]["front_slip"] or core[stamp]["rear_slip"]:
                strata.append("core_slip_flag")
            if core[stamp]["model_only"]:
                strata.append("core_model_only")
            if wheel_flags.get(stamp, False):
                strata.append("wheel_disagreement_gt_0p5")
            if not baseline[stamp].front_used and not baseline[stamp].rear_used:
                strata.append("baseline_no_fresh_wheel")
        else:
            strata = []
        for mode, ref in refs.items():
            applicable = strata if mode == "published_composite" else ["all"]
            be = baseline[stamp].speed_mps - ref
            ce = core[stamp]["velocity_mps"] - ref
            for stratum in applicable:
                cell = accum[(mode, stratum)]
                cell[0] += 1
                cell[1] += be * be
                cell[2] += ce * ce
    result = []
    for (mode, stratum), (n, bss, css) in sorted(accum.items()):
        result.append({
            "split": split, "bag": bag, "vehicle": bag.split("_", 1)[0],
            "mode": mode, "stratum": stratum, "matched": n,
            "baseline_sse": bss, "core_sse": css,
            "baseline_rmse_mps": math.sqrt(bss / n),
            "core_rmse_mps": math.sqrt(css / n),
        })
    return result, {"split": split, "bag": bag, "core_outputs": len(core),
                    "baseline_outputs": len(baseline), "common": len(common), **counts}


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        for key in ("all", row["vehicle"]):
            groups[(row["split"], key, row["mode"], row["stratum"])].append(row)
    out = []
    for (split, vehicle, mode, stratum), group in sorted(groups.items()):
        n = sum(r["matched"] for r in group)
        b = math.sqrt(sum(r["baseline_sse"] for r in group) / n)
        c = math.sqrt(sum(r["core_sse"] for r in group) / n)
        out.append({"split": split, "vehicle": vehicle, "mode": mode,
                    "stratum": stratum, "bags": len(group), "matched": n,
                    "baseline_rmse_mps": b, "core_rmse_mps": c,
                    "delta_mps": c - b,
                    "bag_wins": sum(r["core_rmse_mps"] < r["baseline_rmse_mps"] for r in group)})
    return out


def write_report(summary, quality):
    lines = ["# Чувствительность GNSS-прокси и режимы движения", "",
             "Скрипт `evaluation/audit_reference.py` заново проигрывает C++ оцениватель на сырых bag. Три разрешённых входа идут в оцениватель в порядке SQLite; GNSS используется только здесь для подсчёта ошибки. Таблица привода включена лишь для 30618.", "",
             "## Приёмники GNSS", "",
             "`master_only` и `rover_only` используют каждый приёмник отдельно; `dual_agree` требует оба и разницу скоростей ≤0,3 м/с. `published_composite` использует согласованную пару, а при отсутствии одного приёмника — другой. Все варианты сопоставляются на одинаковых выходных метках внутри своего варианта.", "",
             "| Split | Трамвай | Прокси | Bag | Метки | RMSE база → модель, м/с | Δ |", "|---|---|---|---:|---:|---:|---:|"]
    for s in summary:
        if s["stratum"] == "all" and s["split"] in ("validation", "holdout"):
            lines.append(f"| {s['split']} | {s['vehicle']} | {s['mode']} | {s['bags']} | {s['matched']} | {s['baseline_rmse_mps']:.4f} → {s['core_rmse_mps']:.4f} | {s['delta_mps']:+.4f} |")
    lines += ["", "## Режимы по GNSS-прокси", "",
              "Для диагностики классы определены по эталонной скорости: стоянка <0,5 м/с; ускорение/торможение — изменение скорости на интервале ±0,5 с соответственно >+0,15 или <−0,15 м/с. Это разметка оценки с будущими GNSS-данными, не вход модели. `core_slip_flag` и `wheel_disagreement_gt_0p5` — пересекающиеся подвыборки, не отдельные непересекающиеся классы.", "",
              "| Split | Трамвай | Режим | Bag | Метки | RMSE база → модель, м/с | Δ |", "|---|---|---|---:|---:|---:|---:|"]
    order = {"stop_ref_lt_0p5": 0, "accel_ref_gt_0p15": 1,
             "brake_ref_lt_minus_0p15": 2, "steady_moving": 3,
             "moving_unknown_slope": 4, "wheel_disagreement_gt_0p5": 5,
             "core_slip_flag": 6, "core_model_only": 7,
             "baseline_no_fresh_wheel": 8}
    for s in sorted(summary, key=lambda x: (x["split"], x["vehicle"], order.get(x["stratum"], 99))):
        if s["mode"] != "published_composite" or s["stratum"] == "all":
            continue
        lines.append(f"| {s['split']} | {s['vehicle']} | {s['stratum']} | {s['bags']} | {s['matched']} | {s['baseline_rmse_mps']:.4f} → {s['core_rmse_mps']:.4f} | {s['delta_mps']:+.4f} |")
    lines += ["", "## Покрытие", ""]
    for split in ("validation", "holdout"):
        chosen = [q for q in quality if q["split"] == split]
        total = sum(q["common"] for q in chosen)
        matched = sum(q.get("published_matches", 0) for q in chosen)
        disagree = sum(q.get("dual_disagree_gt_0p3", 0) for q in chosen)
        one = sum(q.get("one_only", 0) for q in chosen)
        lines.append(f"- {split}: {len(chosen)} bag с GNSS, {matched}/{total} ({matched/total:.1%}) общих выходов оценены; меток с одним приёмником {one}, с несогласованной парой >0,3 м/с {disagree}.")
    lines += ["", "Эти метрики условны на наличии и согласованности GNSS. Число отсчётов не равно числу независимых испытаний; для переноса на новые даты нужны новые сессии.", ""]
    (EVAL / "audit_reference.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    catalog = {r["bag"]: r for r in csv.DictReader((ROOT / "analysis" / "catalog" / "bags.csv").open(newline="", encoding="utf-8"))}
    all_rows, quality = [], []
    for split in ("validation", "holdout"):
        for bag in manifest["representatives"][split]:
            if int(catalog[bag]["gnss_count"]) == 0:
                continue
            scored, q = score_bag(split, bag)
            all_rows.extend(scored)
            quality.append(q)
            print(split, bag, q.get("published_matches", 0), flush=True)
    with (EVAL / "audit_reference_rows.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    summary = summarize(all_rows)
    (EVAL / "audit_reference.json").write_text(json.dumps({"quality": quality, "summary": summary}, indent=2) + "\n", encoding="utf-8")
    write_report(summary, quality)
    print(EVAL / "audit_reference.md")


if __name__ == "__main__":
    main()
