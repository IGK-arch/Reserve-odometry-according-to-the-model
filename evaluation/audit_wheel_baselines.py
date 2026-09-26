"""Fixed train-calibrated wheel-only baselines on exact deployed scoring stamps.

The guard threshold and freshness age are predeclared here (0.3 m/s, 0.25 s),
without holdout optimisation. Run: python evaluation/audit_wheel_baselines.py
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

from causal_baseline import CMD, FRONT, REAR, causal_wheel_baseline, make_reference, read_run
from core_benchmark import DEFAULT_EXE, reference_at, replay_cpp

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"
TABLE = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "drive_accel_table.csv"
SCALES = json.loads((EVAL / "wheel_calibration_train.json").read_text(encoding="utf-8"))["vehicles"]
AGE_NS = 250_000_000
DIFFERENCE_MPS = 0.3


def candidates(events, vehicle):
    scales = {FRONT: SCALES[vehicle]["front_scale"], REAR: SCALES[vehicle]["rear_scale"]}
    wheels = {FRONT: (-1, math.nan), REAR: (-1, math.nan)}
    outputs = {}
    previous = {"front_only": 0.0, "rear_only": 0.0, "guarded": 0.0}
    last_stamp = -1
    for _, stamp, topic, raw in events:
        if stamp <= 0 or not math.isfinite(float(raw)):
            continue
        if topic in wheels:
            if 0 <= raw <= 150 and stamp > wheels[topic][0]:
                wheels[topic] = (stamp, raw / 3.6 * scales[topic])
            continue
        if topic != CMD or not -15 <= int(raw) <= 15 or stamp <= last_stamp:
            continue
        last_stamp = stamp
        valid = {key: value for key, (wheel_stamp, value) in wheels.items()
                 if 0 <= stamp - wheel_stamp <= AGE_NS and math.isfinite(value)}
        if FRONT in valid:
            previous["front_only"] = valid[FRONT]
        if REAR in valid:
            previous["rear_only"] = valid[REAR]
        if len(valid) == 2:
            front, rear = valid[FRONT], valid[REAR]
            if abs(front - rear) <= DIFFERENCE_MPS:
                previous["guarded"] = (front + rear) / 2
            else:
                previous["guarded"] = min((front, rear), key=lambda v: abs(v - previous["guarded"]))
        elif len(valid) == 1:
            previous["guarded"] = next(iter(valid.values()))
        outputs[stamp] = dict(previous)
    return outputs


def score_bag(split, bag):
    events, raw_truth = read_run(bag)
    truth = make_reference(raw_truth)
    baseline = {r.stamp_ns: r for r in causal_wheel_baseline(events)}
    wheel = candidates(events, bag.split("_", 1)[0])
    table = TABLE if bag.startswith("30618_") else None
    core = replay_cpp(DEFAULT_EXE.resolve(), bag, events, table)
    models = ["raw_average", "front_only", "rear_only", "guarded", "deployed_core"]
    accum = {m: [0, 0.0, 0.0] for m in models}
    for stamp in core.keys() & baseline.keys() & wheel.keys():
        ref = reference_at(truth, stamp)
        if ref is None:
            continue
        values = {"raw_average": baseline[stamp].speed_mps,
                  **wheel[stamp], "deployed_core": core[stamp]["velocity_mps"]}
        for name, val in values.items():
            e = val - ref
            a = accum[name]
            a[0] += 1
            a[1] += e * e
            a[2] += e
    return [{"split": split, "vehicle": bag.split("_", 1)[0], "bag": bag,
             "model": name, "matched": a[0], "sse": a[1], "error_sum": a[2],
             "rmse_mps": math.sqrt(a[1] / a[0]) if a[0] else math.nan}
            for name, a in accum.items()]


def main():
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    with (ROOT / "analysis" / "catalog" / "bags.csv").open(newline="", encoding="utf-8") as file:
        catalog = {r["bag"]: r for r in csv.DictReader(file)}
    per_bag = []
    for split in ("validation", "holdout"):
        for bag in manifest["representatives"][split]:
            if int(catalog[bag]["gnss_count"]) == 0:
                continue
            per_bag.extend(score_bag(split, bag))
            print(split, bag, flush=True)
    with (EVAL / "audit_wheel_baselines_bags.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(per_bag[0]))
        writer.writeheader()
        writer.writerows(per_bag)
    grouped = defaultdict(list)
    for r in per_bag:
        for vehicle in ("all", r["vehicle"]):
            grouped[(r["split"], vehicle, r["model"])].append(r)
    summary = []
    for (split, vehicle, model), rs in sorted(grouped.items()):
        n = sum(r["matched"] for r in rs)
        summary.append({"split": split, "vehicle": vehicle, "model": model,
                        "bags": len(rs), "matched": n,
                        "rmse_mps": math.sqrt(sum(r["sse"] for r in rs) / n),
                        "bias_mps": sum(r["error_sum"] for r in rs) / n})
    (EVAL / "audit_wheel_baselines.json").write_text(json.dumps({
        "policy": "Train-only per-vehicle wheel multipliers; front/rear hold-last when stale; guard uses 0.25s freshness, 0.3m/s wheel difference, otherwise wheel nearest previous guard output; no holdout tuning",
        "summary": summary}, indent=2) + "\n", encoding="utf-8")
    lines = ["# Фиксированные wheel-only альтернативы", "",
             "Все модели оценены на одних и тех же выходных метках C++ replay и доступного GNSS-прокси. `front_only`/`rear_only` используют train-only масштабы колёс и hold-last при возрасте >0,25 с. `guarded`: при разнице колёс ≤0,3 м/с усредняет, иначе выбирает колесо ближе к предыдущей оценке; при двух отсутствующих удерживает значение. Порог выбран заранее и не подбирался на validation/holdout.", "",
             "| Split | Трамвай | Метод | Bag | Метки | RMSE, м/с | Bias, м/с |", "|---|---|---|---:|---:|---:|---:|"]
    for r in summary:
        lines.append(f"| {r['split']} | {r['vehicle']} | {r['model']} | {r['bags']} | {r['matched']} | {r['rmse_mps']:.4f} | {r['bias_mps']:+.4f} |")
    lines += ["", "Эти альтернативы оценивают ценность модели относительно более сильных простых правил. Выбор лучшего метода по holdout был бы утечкой; ни одна альтернатива здесь не переключена в production.", ""]
    (EVAL / "audit_wheel_baselines.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
