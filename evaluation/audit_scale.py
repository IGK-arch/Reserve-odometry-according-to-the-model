"""Session-level wheel/GNSS common-scale audit; diagnosis only, no tuning.

Run: python evaluation/audit_scale.py
"""

from __future__ import annotations

import csv
import json
import math
import statistics as st
from collections import defaultdict
from pathlib import Path

from calibrate_wheels import estimate_bag

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"


def float_or_none(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (ValueError, TypeError):
        return None


def main():
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    with (ROOT / "analysis" / "catalog" / "bags.csv").open(newline="", encoding="utf-8") as file:
        catalog = {r["bag"]: r for r in csv.DictReader(file)}
    with (ROOT / "analysis" / "signals" / "bag_summary.csv").open(newline="", encoding="utf-8") as file:
        quality = {r["bag"]: r for r in csv.DictReader(file)}
    out = []
    for split in ("train", "validation", "holdout"):
        for bag in manifest["representatives"][split]:
            if int(catalog[bag]["gnss_count"]) == 0:
                continue
            scored = estimate_bag(bag)
            q = quality.get(bag, {})
            out.append({
                "split": split, "bag": bag, "vehicle": catalog[bag]["vehicle"],
                "session_date": catalog[bag]["start_utc"][:10],
                "calibration_points": scored["n"] if scored else 0,
                "front_scale": scored["front_scale"] if scored else "",
                "rear_scale": scored["rear_scale"] if scored else "",
                "mean_scale": (scored["front_scale"] + scored["rear_scale"]) / 2 if scored else "",
                "rover_master_p95_abs_mps": q.get("rover_master_p95_abs", ""),
                "rover_master_gt_0p5_fraction": q.get("rover_master_gt_0p5_fraction", ""),
                "pair_coverage": q.get("pair_coverage", ""),
            })
            print(split, bag, scored["n"] if scored else 0, flush=True)
    with (EVAL / "audit_scale_bags.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(out[0]))
        writer.writeheader()
        writer.writerows(out)
    by_session = defaultdict(list)
    for row in out:
        by_session[(row["split"], row["vehicle"], row["session_date"])].append(row)
    sessions = []
    for (split, vehicle, date), items in sorted(by_session.items()):
        usable = [r for r in items if r["mean_scale"] != ""]
        scales = [r["mean_scale"] for r in usable]
        gnss_quality = [x for r in items if (x := float_or_none(r["rover_master_p95_abs_mps"])) is not None]
        pair = [x for r in items if (x := float_or_none(r["pair_coverage"])) is not None]
        sessions.append({
            "split": split, "vehicle": vehicle, "session_date": date,
            "bags_with_gnss": len(items), "bags_with_scale": len(usable),
            "median_mean_scale": st.median(scales) if scales else None,
            "min_mean_scale": min(scales) if scales else None,
            "max_mean_scale": max(scales) if scales else None,
            "median_rover_master_p95_abs_mps": st.median(gnss_quality) if gnss_quality else None,
            "median_pair_coverage": st.median(pair) if pair else None,
        })
    (EVAL / "audit_scale.json").write_text(json.dumps({"method":
        "Per-bag least-squares wheel multiplier on dual-GNSS agreement, 2-16 m/s and non-slip filters from calibrate_wheels.py; median by vehicle/date. Diagnostic only.",
        "sessions": sessions}, indent=2) + "\n", encoding="utf-8")
    lines = ["# Масштаб колёс и GNSS по сессиям", "",
             "Расчёт `evaluation/audit_scale.py` использует dual-GNSS согласие и фильтры `calibrate_wheels.py`; масштаб — дополнительный множитель к переводу raw km/h в м/с. Это диагностическая оценка на всех split, не новая калибровка модели.", "",
             "| Split | Трамвай / дата | Bag с масштабом / GNSS | Медиана масштаба (min–max bag) | Медиана p95 master/rover, м/с |", "|---|---|---:|---:|---:|"]
    for s in sessions:
        scale = (f"{s['median_mean_scale']:.5f} ({s['min_mean_scale']:.5f}–{s['max_mean_scale']:.5f})"
                 if s["median_mean_scale"] is not None else "—")
        q = f"{s['median_rover_master_p95_abs_mps']:.3f}" if s["median_rover_master_p95_abs_mps"] is not None else "—"
        lines.append(f"| {s['split']} | {s['vehicle']} / {s['session_date']} | {s['bags_with_scale']}/{s['bags_with_gnss']} | {scale} | {q} |")
    lines += ["", "Поскольку bag одной даты коррелированы, разница между сессиями важнее узкого разброса bag внутри даты. Holdout масштабы показаны только для диагностики переноса; коэффициенты модели по ним подбирать нельзя.", ""]
    (EVAL / "audit_scale.md").write_text("\n".join(lines), encoding="utf-8")
    print(EVAL / "audit_scale.md")


if __name__ == "__main__":
    main()
