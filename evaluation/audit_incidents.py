"""Focused slip and rear-freeze incident audit on two known difficult bags."""

from __future__ import annotations

import csv
import json
import math
import statistics as st
from pathlib import Path

from causal_baseline import FRONT, REAR, causal_wheel_baseline, make_reference, read_run
from core_benchmark import DEFAULT_EXE, reference_at, replay_cpp

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"
TABLE = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "drive_accel_table.csv"
BAGS = ["30618_33bec73f", "30639_3b3d9eb8"]


def metric(n, bss, css):
    return {"n": n, "baseline_rmse_mps": math.sqrt(bss / n) if n else None,
            "core_rmse_mps": math.sqrt(css / n) if n else None}


def freeze_runs(events):
    front = [(recv, val / 3.6) for recv, _, topic, val in events if topic == FRONT]
    rear = [(recv, val) for recv, _, topic, val in events if topic == REAR]
    grouped = []
    current = []
    for receive, value in rear:
        if value == 0:
            current.append(receive)
        elif current:
            grouped.append((current[0], current[-1], receive, len(current)))
            current = []
    if current:
        grouped.append((current[0], current[-1], current[-1], len(current)))
    candidates = []
    for start, last_zero, recovery, samples in grouped:
        speeds = [v for t, v in front if start <= t <= last_zero]
        if speeds and st.median(speeds) > 1 and (last_zero - start) > 2e9:
            candidates.append({"start_receive_ns": start, "last_zero_receive_ns": last_zero,
                               "first_nonzero_receive_ns": recovery,
                               "observed_zero_span_s": (last_zero - start) / 1e9,
                               "post_zero_silence_s": (recovery - last_zero) / 1e9,
                               "time_to_nonzero_s": (recovery - start) / 1e9,
                               "zero_samples": samples,
                               "front_median_during_mps": st.median(speeds),
                               "front_max_during_mps": max(speeds)})
    return sorted(candidates, key=lambda r: r["observed_zero_span_s"], reverse=True)


def score_windows(events, truth, baseline, core, freeze):
    start, end = freeze["start_receive_ns"], freeze["first_nonzero_receive_ns"]
    windows = {
        "before_5s": (start - 5_000_000_000, start),
        "rear_zero_or_absent": (start, end),
        "after_0_5s": (end, end + 5_000_000_000),
        "after_5_30s": (end + 5_000_000_000, end + 30_000_000_000),
    }
    accum = {key: [0, 0.0, 0.0] for key in windows}
    for stamp in core.keys() & baseline.keys():
        base = baseline[stamp]
        ref = reference_at(truth, stamp)
        if ref is None:
            continue
        for name, (a, b) in windows.items():
            if a <= base.receive_ns < b:
                be = base.speed_mps - ref
                ce = core[stamp]["velocity_mps"] - ref
                cell = accum[name]
                cell[0] += 1
                cell[1] += be * be
                cell[2] += ce * ce
                break
    return {name: metric(*values) for name, values in accum.items()}


def main():
    results = {}
    for bag in BAGS:
        events, raw_truth = read_run(bag)
        truth = make_reference(raw_truth)
        baseline = {r.stamp_ns: r for r in causal_wheel_baseline(events)}
        table = TABLE if bag.startswith("30618_") else None
        core = replay_cpp(DEFAULT_EXE.resolve(), bag, events, table)
        all_errors = []
        flagged = []
        for stamp in core.keys() & baseline.keys():
            ref = reference_at(truth, stamp)
            if ref is None:
                continue
            errors = (baseline[stamp].speed_mps - ref, core[stamp]["velocity_mps"] - ref)
            all_errors.append(errors)
            if core[stamp]["front_slip"] or core[stamp]["rear_slip"]:
                flagged.append(errors)
        def errors_metric(errors):
            return metric(len(errors), sum(x[0] ** 2 for x in errors),
                          sum(x[1] ** 2 for x in errors))
        nflag = len(flagged)
        bss_flag = sum(x[0] ** 2 for x in flagged)
        css_flag = sum(x[1] ** 2 for x in flagged)
        bss_all = sum(x[0] ** 2 for x in all_errors)
        css_all = sum(x[1] ** 2 for x in all_errors)
        nonflag = metric(len(all_errors) - nflag, bss_all - bss_flag, css_all - css_flag)
        freezes = freeze_runs(events)
        first_recv = min(x[0] for x in events)
        incident = freezes[0] if freezes else None
        if incident:
            incident["start_offset_s"] = (incident["start_receive_ns"] - first_recv) / 1e9
            incident["first_nonzero_offset_s"] = (incident["first_nonzero_receive_ns"] - first_recv) / 1e9
        results[bag] = {
            "all": errors_metric(all_errors), "slip_flagged": errors_metric(flagged),
            "slip_unflagged": nonflag,
            "moving_rear_zero_runs": len(freezes),
            "longest_moving_rear_zero_run": incident,
            "longest_run_window_scores": score_windows(events, truth, baseline, core, incident) if incident else None,
        }
        print(bag, results[bag]["all"], flush=True)
    (EVAL / "audit_incidents.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    lines = ["# Два инцидента: проскальзывание и нулевой rear", "",
             "Точный исправленный C++ replay на одинаковых метках с GNSS-прокси. Флаги slip взяты из оценивателя; они редки и не представляют всё движение. RMSE на флагах нельзя обобщать на новые эпизоды.", "",
             "| Bag | Подвыборка | Метки | RMSE база → модель, м/с |", "|---|---|---:|---:|"]
    for bag, record in results.items():
        for label in ("all", "slip_flagged", "slip_unflagged"):
            r = record[label]
            lines.append(f"| {bag} | {label} | {r['n']} | {r['baseline_rmse_mps']:.4f} → {r['core_rmse_mps']:.4f} |")
    target = results["30639_3b3d9eb8"]
    run = target["longest_moving_rear_zero_run"]
    lines += ["", f"В `30639_3b3d9eb8` самая длинная серия rear=0 при движении началась на {run['start_offset_s']:.2f} с: {run['zero_samples']} нулевых сообщений за {run['observed_zero_span_s']:.2f} с (медианная скорость front {run['front_median_during_mps']:.2f} м/с). Затем rear не поступал {run['post_zero_silence_s']:.2f} с; первый ненулевой rear получен на {run['first_nonzero_offset_s']:.2f} с, через {run['time_to_nonzero_s']:.2f} с от начала. Интервалы определены по времени получения SQLite, потому что в bag есть аномальные header timestamps.", "",
              "| Окно относительно серии | Метки | RMSE база → модель, м/с |", "|---|---:|---:|"]
    for label, r in target["longest_run_window_scores"].items():
        lines.append(f"| {label} | {r['n']} | {r['baseline_rmse_mps']:.4f} → {r['core_rmse_mps']:.4f} |" if r["n"] else f"| {label} | 0 | — |")
    lines += ["", "Показатель после серии — диагностический: изменение режима движения и GNSS-прокси тоже влияют на RMSE. Он не доказывает отдельную гарантию времени восстановления.", ""]
    (EVAL / "audit_incidents.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
