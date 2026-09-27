"""Independent split and paired uncertainty audit of saved evaluation CSVs.

The independent unit is a recording session (vehicle/date), not a sensor
sample. Bag bootstrap intervals below describe only within-session bag
variation; they must not be interpreted as generalisation to new dates.

Run: python evaluation/audit_stats.py [--verify-files]
Writes evaluation/audit_stats.json, audit_bags.csv and audit_stats.md.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
import statistics as st
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"
SEED = 20260927
BOOT = 20000


def rows(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def build_deployed_artifacts(manifest):
    """Freeze the actual per-vehicle mode from the two corrected replay CSVs."""
    artifacts = {}
    for split in ("validation", "holdout"):
        for kind in ("core", "distance"):
            physics_name = f"{split}_{kind}_rawfreeze_fix.csv"
            table_name = f"{split}_{kind}_table_optional.csv"
            physics = {r["bag"]: r for r in rows(EVAL / physics_name)}
            table = {r["bag"]: r for r in rows(EVAL / table_name)}
            expected = set(manifest["representatives"][split])
            if set(physics) != set(table) or not set(physics) <= expected or (kind == "core" and set(physics) != expected):
                raise ValueError(f"source bag sets differ from {split} representatives: {kind}")
            output = []
            for bag in sorted(physics):
                from_table = bag.startswith("30618_")
                source = table_name if from_table else physics_name
                selected = dict(table[bag] if from_table else physics[bag])
                selected["source_artifact"] = source
                output.append(selected)
            name = f"{split}_{kind}_deployed.csv"
            with (EVAL / name).open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(output[0]))
                writer.writeheader()
                writer.writerows(output)
            final = {r["bag"]: r for r in rows(EVAL / f"{split}_{kind}_final.csv")}
            differences = []
            for r in output:
                bag = r["bag"]
                if bag not in final or any(final[bag].get(k) != val for k, val in r.items()
                                            if k != "source_artifact"):
                    differences.append(bag)
            artifacts[name] = {"table_source": table_name,
                               "physics_source": physics_name,
                               "table_vehicle": "30618", "different_from_final_bags": differences}
    return artifacts


def f(row, col):
    return float(row[col])


def median_or_nan(vals):
    return st.median(vals) if vals else math.nan


def percentile(vals, p):
    vals = sorted(vals)
    x = (len(vals) - 1) * p
    lo = math.floor(x)
    hi = math.ceil(x)
    return vals[lo] + (vals[hi] - vals[lo]) * (x - lo)


def bootstrap_ci(data, stat, n=BOOT):
    rng = random.Random(SEED + len(data) * 17)
    draws = [stat([data[rng.randrange(len(data))] for _ in data]) for _ in range(n)]
    return [percentile(draws, 0.025), percentile(draws, 0.975)]


def pooled_rmse(data, field):
    count = sum(int(r["matched"]) for r in data)
    return math.sqrt(sum(int(r["matched"]) * f(r, field) ** 2 for r in data) / count)


def speed_delta(data):
    return pooled_rmse(data, "core_rmse_mps") - pooled_rmse(data, "baseline_rmse_mps")


def paired_sign_p(wins, losses):
    n = wins + losses
    if not n:
        return math.nan
    tail = sum(math.comb(n, k) for k in range(min(wins, losses) + 1)) / 2**n
    return min(1.0, 2 * tail)


def signed_summary(data, base, core, transform=lambda x: x):
    deltas = [transform(f(r, core)) - transform(f(r, base)) for r in data]
    wins = sum(x < 0 for x in deltas)
    losses = sum(x > 0 for x in deltas)
    return {
        "bags": len(data), "wins": wins, "losses": losses,
        "ties": len(data) - wins - losses,
        "median_baseline": median_or_nan([transform(f(r, base)) for r in data]),
        "median_core": median_or_nan([transform(f(r, core)) for r in data]),
        "median_paired_delta": median_or_nan(deltas),
        "median_delta_bag_boot_ci": bootstrap_ci(deltas, st.median) if deltas else None,
        "two_sided_sign_p_bag_level": paired_sign_p(wins, losses),
    }


def audit_split(catalog, manifest, verify_files):
    by_bag = {r["bag"]: r for r in catalog}
    by_hash = defaultdict(list)
    for r in catalog:
        by_hash[r["db_sha256"]].append(r)
    session_splits = {(x["vehicle"], x["date"]): x["split"] for x in manifest["sessions"]}
    assigned = {}
    errors = []
    for split, reps in manifest["representatives"].items():
        for bag in reps:
            if bag not in by_bag:
                errors.append(f"representative missing from catalog: {bag}")
                continue
            if bag in assigned:
                errors.append(f"representative repeated: {bag}")
            assigned[bag] = split
            members = by_hash[by_bag[bag]["db_sha256"]]
            for member in members:
                session = (member["vehicle"], member["start_utc"][:10])
                if session_splits.get(session) != split:
                    errors.append(f"cross-split SHA group: {bag}, {member['bag']}")
    if len(assigned) != len(by_hash):
        errors.append(f"representatives {len(assigned)} != SHA groups {len(by_hash)}")
    seen_hashes = {by_bag[b]["db_sha256"] for b in assigned if b in by_bag}
    if seen_hashes != set(by_hash):
        errors.append("representatives do not cover every SHA group")
    for r in catalog:
        if (r["vehicle"], r["start_utc"][:10]) not in session_splits:
            errors.append(f"unassigned session: {r['bag']}")
    inputs = rows(ROOT / "analysis" / "signals" / "duplicates.csv")
    by_input = defaultdict(set)
    for r in inputs:
        if r["bag"] in by_bag:
            cat = by_bag[r["bag"]]
            by_input[r["inputs_sha256"]].add(session_splits[(cat["vehicle"], cat["start_utc"][:10])])
    cross_input = sum(len(splits) > 1 for splits in by_input.values())
    if verify_files:
        for r in catalog:
            path = ROOT / "dataset" / "data" / r["bag"] / f"{r['bag']}_0.db3"
            if not path.exists():
                errors.append(f"missing db: {r['bag']}")
                continue
            with path.open("rb") as stream:
                digest = hashlib.file_digest(stream, "sha256").hexdigest()
            if digest != r["db_sha256"]:
                errors.append(f"catalog SHA mismatch: {r['bag']}")
    return {
        "catalog_bags": len(catalog), "sha_groups": len(by_hash),
        "duplicate_bags": len(catalog) - len(by_hash),
        "session_counts": dict(Counter(f"{v}/{d}" for v, d in
                                       ((r["vehicle"], r["start_utc"][:10]) for r in catalog))),
        "representative_counts": {s: len(v) for s, v in manifest["representatives"].items()},
        "cross_split_input_sha_groups": cross_input,
        "file_hashes_verified": bool(verify_files),
        "errors": errors,
    }


def analyse_speed(data, group_bags):
    selected = [r for r in data if int(r["matched"]) > 0 and r["bag"] in group_bags]
    if not selected:
        return None
    matches = sum(int(r["matched"]) for r in selected)
    outputs = sum(int(r["core_outputs"]) for r in data if r["bag"] in group_bags)
    deltas = [f(r, "core_rmse_mps") - f(r, "baseline_rmse_mps") for r in selected]
    wins = sum(x < 0 for x in deltas)
    losses = sum(x > 0 for x in deltas)
    counts = sorted((int(r["matched"]) for r in selected), reverse=True)
    return {
        "total_bags": len(group_bags), "labeled_bags": len(selected),
        "matched": matches, "core_outputs": outputs,
        "matched_fraction_of_all_outputs": matches / outputs if outputs else 0,
        "largest_bag_share_of_matches": counts[0] / matches,
        "largest_three_bag_share_of_matches": sum(counts[:3]) / matches,
        "pooled_baseline_rmse_mps": pooled_rmse(selected, "baseline_rmse_mps"),
        "pooled_core_rmse_mps": pooled_rmse(selected, "core_rmse_mps"),
        "pooled_delta_core_minus_baseline_mps": speed_delta(selected),
        "bag_boot_pooled_delta_95_ci_mps": bootstrap_ci(selected, speed_delta),
        "leave_one_bag_out_delta_range_mps": [
            min(speed_delta(selected[:i] + selected[i + 1:]) for i in range(len(selected))),
            max(speed_delta(selected[:i] + selected[i + 1:]) for i in range(len(selected))),
        ] if len(selected) > 1 else None,
        "median_baseline_bag_rmse_mps": st.median(f(r, "baseline_rmse_mps") for r in selected),
        "median_core_bag_rmse_mps": st.median(f(r, "core_rmse_mps") for r in selected),
        "median_paired_bag_delta_mps": st.median(deltas),
        "bag_wins": wins, "bag_losses": losses,
        "two_sided_sign_p_bag_level": paired_sign_p(wins, losses),
    }


def table_ablation(split, manifest):
    bags = [b for b in manifest["representatives"][split] if b.startswith("30618_")]
    old = {r["bag"]: r for r in rows(EVAL / f"{split}_core_rawfreeze_fix.csv")}
    new = {r["bag"]: r for r in rows(EVAL / f"{split}_core_table_optional.csv")}
    paired = []
    for bag in bags:
        a, b = old[bag], new[bag]
        if a["matched"] != b["matched"]:
            raise ValueError(f"table changed reference sample set: {bag}")
        if int(a["matched"]) > 0:
            paired.append({"bag": bag, "matched": a["matched"],
                           "physics_rmse_mps": a["core_rmse_mps"],
                           "table_rmse_mps": b["core_rmse_mps"]})
    def delta(data):
        return pooled_rmse(data, "table_rmse_mps") - pooled_rmse(data, "physics_rmse_mps")
    speed = {
        "bags": len(paired), "matched": sum(int(r["matched"]) for r in paired),
        "physics_pooled_rmse_mps": pooled_rmse(paired, "physics_rmse_mps"),
        "table_pooled_rmse_mps": pooled_rmse(paired, "table_rmse_mps"),
        "pooled_delta_mps": delta(paired),
        "bag_boot_pooled_delta_95_ci_mps": bootstrap_ci(paired, delta),
        "table_bag_wins": sum(f(r, "table_rmse_mps") < f(r, "physics_rmse_mps") for r in paired),
        "median_paired_bag_delta_mps": st.median(f(r, "table_rmse_mps") - f(r, "physics_rmse_mps") for r in paired),
    }
    old_dist = {r["bag"]: r for r in rows(EVAL / f"{split}_distance_rawfreeze_fix.csv")}
    new_dist = {r["bag"]: r for r in rows(EVAL / f"{split}_distance_table_optional.csv")}
    dist_pairs = []
    for bag in bags:
        if bag in old_dist and bag in new_dist:
            dist_pairs.append({"bag": bag,
                               "physics_drift_pct": old_dist[bag]["core_drift_pct"],
                               "table_drift_pct": new_dist[bag]["core_drift_pct"]})
    drift = signed_summary(dist_pairs, "physics_drift_pct", "table_drift_pct", abs)
    return {"speed": speed, "absolute_end_drift_pct": drift}


def speed_by_session(catalog):
    out = []
    source = {"train": EVAL / "audit_train_core_deployed.csv",
              "validation": EVAL / "validation_core_deployed.csv",
              "holdout": EVAL / "holdout_core_deployed.csv"}
    by_bag = {r["bag"]: r for r in catalog}
    for split, path in source.items():
        if not path.exists():
            continue
        grouped = defaultdict(list)
        for r in rows(path):
            if int(r["matched"]) > 0:
                c = by_bag[r["bag"]]
                grouped[(c["vehicle"], c["start_utc"][:10])].append(r)
        for (vehicle, date), group in sorted(grouped.items()):
            n = sum(int(r["matched"]) for r in group)
            out.append({"split": split, "vehicle": vehicle, "session_date": date,
                        "labeled_bags": len(group), "matched": n,
                        "baseline_rmse_mps": pooled_rmse(group, "baseline_rmse_mps"),
                        "core_rmse_mps": pooled_rmse(group, "core_rmse_mps"),
                        "baseline_bias_mps": sum(int(r["matched"]) * f(r, "baseline_bias_mps") for r in group) / n,
                        "core_bias_mps": sum(int(r["matched"]) * f(r, "core_bias_mps") for r in group) / n})
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-files", action="store_true", help="SHA-256 every SQLite file")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    catalog = rows(ROOT / "analysis" / "catalog" / "bags.csv")
    cat_by_bag = {r["bag"]: r for r in catalog}
    split = audit_split(catalog, manifest, args.verify_files)
    deployed_artifacts = build_deployed_artifacts(manifest)
    inputs = {
        "validation": {
            "speed": EVAL / "validation_core_deployed.csv",
            "distance": EVAL / "validation_distance_deployed.csv",
            "position": EVAL / "validation_position_table.csv",
        },
        "holdout": {
            "speed": EVAL / "holdout_core_deployed.csv",
            "distance": EVAL / "holdout_distance_deployed.csv",
            "position": EVAL / "holdout_position_deployed_proxy.csv",
        },
    }
    report = {"seed": SEED, "bag_bootstrap_draws": BOOT, "split": split,
              "deployed_artifacts": deployed_artifacts, "sets": {}}
    bag_export = []
    for set_name, paths in inputs.items():
        data = {kind: rows(path) for kind, path in paths.items()}
        reps = set(manifest["representatives"][set_name])
        issues = []
        if {r["bag"] for r in data["speed"]} != reps:
            issues.append("speed CSV bag set differs from split representatives")
        for kind in ("distance", "position"):
            if not {r["bag"] for r in data[kind]} <= reps:
                issues.append(f"{kind} CSV has bags outside the split")
        groups = {"all": reps}
        groups.update({v: {b for b in reps if b.startswith(v + "_")} for v in ("30618", "30639")})
        sessions = defaultdict(set)
        for bag in reps:
            c = cat_by_bag[bag]
            sessions[f"{c['vehicle']}/{c['start_utc'][:10]}"].add(bag)
        groups.update(sessions)
        summaries = {}
        for group, bags in groups.items():
            sr = analyse_speed(data["speed"], bags)
            dr = [r for r in data["distance"] if r["bag"] in bags]
            pr = [r for r in data["position"] if r["bag"] in bags]
            summaries[group] = {
                "speed": sr,
                "distance_abs_end_drift_pct": signed_summary(
                    dr, "baseline_drift_pct", "core_drift_pct", abs) if dr else None,
                "position_master_3d_rmse_m": signed_summary(
                    pr, "baseline_rmse_3d_m", "rmse_3d_m") if pr else None,
                "position_base_link_3d_rmse_m": signed_summary(
                    pr, "baseline_base_rmse_3d_m", "base_rmse_3d_m") if pr else None,
            }
        by_speed = {r["bag"]: r for r in data["speed"]}
        by_dist = {r["bag"]: r for r in data["distance"]}
        by_pos = {r["bag"]: r for r in data["position"]}
        for bag in sorted(reps):
            c = cat_by_bag[bag]
            s = by_speed[bag]
            d = by_dist.get(bag, {})
            p = by_pos.get(bag, {})
            bag_export.append({
                "split": set_name, "bag": bag, "vehicle": c["vehicle"],
                "session_date": c["start_utc"][:10], "duration_s": c["duration_s"],
                "gnss_messages": c["gnss_count"],
                "speed_matched": s["matched"], "core_outputs": s["core_outputs"],
                "speed_baseline_rmse_mps": s["baseline_rmse_mps"],
                "speed_core_rmse_mps": s["core_rmse_mps"],
                "distance_reference_m": d.get("reference_distance_m", ""),
                "distance_baseline_drift_pct": d.get("baseline_drift_pct", ""),
                "distance_core_drift_pct": d.get("core_drift_pct", ""),
                "position_matched": p.get("matched", ""),
                "position_baseline_base_rmse_m": p.get("baseline_base_rmse_3d_m", ""),
                "position_core_base_rmse_m": p.get("base_rmse_3d_m", ""),
            })
        report["sets"][set_name] = {"input_issues": issues, "sessions": sorted(sessions),
                                    "summaries": summaries}
    report["interpretation"] = {
        "independent_sessions_validation": 1,
        "independent_sessions_holdout": 2,
        "bag_bootstrap_limit": "Conditional on these recorded dates; bags within a date are correlated. No defensible session-level CI exists with one validation and two holdout sessions.",
        "holdout_selection_limit": "Final mode choice used holdout feedback; holdout is no longer untouched for model selection.",
        "time_order_limit": "The 30618 training set includes 2026-09-03, later than 2026-08-26 validation and 2026-08-10 holdout. This is a session split, not a prospective time-forward test.",
        "command_clock_limit": "Overlapping command and wheel header-time branches occur in holdout. A trial future-header guard suppressed genuine outputs and was reverted; see evaluation/audit_command_timing.md. Current results do not establish robustness to an isolated corrupt future header.",
        "reference_limit": "GNSS is an imperfect proxy and available only on a subset of outputs; no hidden jury fused reference is available.",
    }
    report["table_ablation_30618"] = {
        split: table_ablation(split, manifest) for split in ("validation", "holdout")}
    report["speed_by_session"] = speed_by_session(catalog)
    (EVAL / "audit_stats.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (EVAL / "audit_bags.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(bag_export[0]))
        writer.writeheader()
        writer.writerows(bag_export)
    write_markdown(report)
    print(EVAL / "audit_stats.md")
    print("split errors:", split["errors"])


def write_markdown(report):
    lines = ["# Независимый статистический аудит", "",
             "Скрипт: `python evaluation/audit_stats.py --verify-files`. Скрипт формирует `*_deployed.csv`: train-only таблица для 30618, исправленная физическая модель для 30639. Источник метрик — сохранённые per-bag CSV replay; группировки, парные сравнения и интервалы вычислены заново. `audit_bags.csv` содержит все bag, включая неоценённые.", "",
             "## Разбиение и единица независимости", "",
             f"Каталог: {report['split']['catalog_bags']} bag, {report['split']['sha_groups']} уникальных SQLite SHA, {report['split']['duplicate_bags']} дубликатов. Проверено файлов SHA: {report['split']['file_hashes_verified']}. Ошибки разбиения: {report['split']['errors']}. Совпадающих SHA разрешённых входов между split: {report['split']['cross_split_input_sha_groups']}.", "",
             "Сессии — сочетания трамвай/дата: validation 1 (только 30618), holdout 2 (по одной на трамвай). Bootstrap по bag даёт только условную изменчивость внутри этих дат. Интервал для новой сессии статистически не оценить. Train для 30618 включает 2026-09-03, что позже validation 2026-08-26 и holdout 2026-08-10: это разбиение по сессиям, но не хронологический прогноз на будущую дату.", "",
             "**Устаревшие артефакты:** файлы `*_final.csv` не представляют текущий deployed режим. В частности, `holdout_core_final.csv` содержит старый сбой `30639_3b3d9eb8` с RMSE 1,531 м/с вместо исправленного 0,225 м/с. В таблицах ниже используются только `*_deployed.csv`.", "",
             "## Скорость, парное сравнение на одинаковых GNSS-метках", "",
             "| Набор | Bag с метками / всего | Покрытие выходов | Pooled RMSE база → модель, м/с | Δ модель−база, м/с | 95% bag bootstrap Δ | Победы bag |", "|---|---:|---:|---:|---:|---:|---:|"]
    for set_name in ("validation", "holdout"):
        sm = report["sets"][set_name]["summaries"]
        for g in ("all", "30618", "30639"):
            s = sm[g]["speed"]
            if not s:
                continue
            ci = s["bag_boot_pooled_delta_95_ci_mps"]
            lines.append(f"| {set_name} {g} | {s['labeled_bags']}/{s['total_bags']} | {100*s['matched_fraction_of_all_outputs']:.1f}% | {s['pooled_baseline_rmse_mps']:.4f} → {s['pooled_core_rmse_mps']:.4f} | {s['pooled_delta_core_minus_baseline_mps']:+.4f} | [{ci[0]:+.4f}, {ci[1]:+.4f}] | {s['bag_wins']}/{s['labeled_bags']} |")
    lines += ["", "Интервалы выше ресэмплируют bag, а не независимые даты. Их нельзя использовать как доказательство переноса на новые сессии. Pooled RMSE взвешивает bag числом GNSS-меток.", "",
              "| Набор | Медиана абсолютного конечного дрейфа база → модель | Парная медиана Δ дрейфа (95% bag bootstrap) | Победы bag | Медиана 3D RMSE `base_link` база → модель | Парная медиана Δ 3D (95% bag bootstrap) | Победы bag |", "|---|---:|---:|---:|---:|---:|---:|"]
    for set_name in ("validation", "holdout"):
        for g in ("all", "30618", "30639"):
            sm = report["sets"][set_name]["summaries"][g]
            d, p = sm["distance_abs_end_drift_pct"], sm["position_base_link_3d_rmse_m"]
            if not d and not p:
                continue
            ds = f"{d['median_baseline']:.3f}% → {d['median_core']:.3f}% ({d['bags']} bag)" if d else "—"
            ps = f"{p['median_baseline']:.2f} → {p['median_core']:.2f} м ({p['bags']} bag)" if p else "—"
            dci = d["median_delta_bag_boot_ci"] if d else None
            pci = p["median_delta_bag_boot_ci"] if p else None
            dd = f"{d['median_paired_delta']:+.3f}% [{dci[0]:+.3f}, {dci[1]:+.3f}]" if d else "—"
            pd = f"{p['median_paired_delta']:+.2f} м [{pci[0]:+.2f}, {pci[1]:+.2f}]" if p else "—"
            dw = f"{d['wins']}/{d['bags']}" if d else "—"
            pw = f"{p['wins']}/{p['bags']}" if p else "—"
            lines.append(f"| {set_name} {g} | {ds} | {dd} | {dw} | {ps} | {pd} | {pw} |")
    lines += ["", "## Парная абляция таблицы привода на 30618", "",
              "Сравнение исправленной физической модели и той же модели с train-only таблицей; одни и те же bag и GNSS-метки.", "",
              "| Split | Pooled RMSE физика → таблица, м/с | Δ (95% bag bootstrap) | Победы таблицы по bag | Медиана абсолютного дрейфа физика → таблица | Победы по дрейфу |", "|---|---:|---:|---:|---:|---:|"]
    for split in ("validation", "holdout"):
        a = report["table_ablation_30618"][split]
        s, d = a["speed"], a["absolute_end_drift_pct"]
        ci = s["bag_boot_pooled_delta_95_ci_mps"]
        lines.append(f"| {split} | {s['physics_pooled_rmse_mps']:.4f} → {s['table_pooled_rmse_mps']:.4f} | {s['pooled_delta_mps']:+.4f} [{ci[0]:+.4f}, {ci[1]:+.4f}] | {s['table_bag_wins']}/{s['bags']} | {d['median_baseline']:.3f}% → {d['median_core']:.3f}% | {d['wins']}/{d['bags']} |")
    lines += ["", "## Скорость и систематическое смещение по сессиям", "",
              "Train измерен на данных, использованных для калибровки колёс и таблицы. Это описательная in-sample величина, а не независимый тест.", "",
              "| Split | Трамвай / дата | Bag | Метки | RMSE база → модель, м/с | Bias база → модель, м/с |", "|---|---|---:|---:|---:|---:|"]
    for s in report["speed_by_session"]:
        lines.append(f"| {s['split']} | {s['vehicle']} / {s['session_date']} | {s['labeled_bags']} | {s['matched']} | {s['baseline_rmse_mps']:.4f} → {s['core_rmse_mps']:.4f} | {s['baseline_bias_mps']:+.4f} → {s['core_bias_mps']:+.4f} |")
    lines += ["", "![Разница между датами и трамваями](audit_generalization.png)", "",
              "## Ограничения вывода", "",
              "- GNSS-прокси есть не у каждого bag; coverage в таблице посчитан относительно всех выходов split, включая bag без GNSS. Отбор меток может смещать оценку.",
              "- Holdout уже использовался при выборе финального режима. Это тест для диагностики, но не untouched оценка выбора модели.",
              "- [Аудит времени команд](audit_command_timing.md) выявил перемежающиеся ветки `header.stamp` в holdout. Пробный guard будущих меток подавлял настоящие выходы и был откатан; текущие метрики не доказывают защиту от одиночного ошибочного будущего заголовка.",
              "- В данных нет скрытого объединённого эталона жюри. RMSE GNSS-прокси не следует называть официальной точностью.",
              "- Все отсчёты в одном bag и bag одной даты коррелированы. Число совпавших GNSS-меток не равно числу независимых испытаний.", ""]
    (EVAL / "audit_stats.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
