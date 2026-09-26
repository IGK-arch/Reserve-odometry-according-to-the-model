"""Train-only observational drive table and validation model-only stress test.

Run: py -3.12 evaluation/calibrate_drive.py

The table is descriptive, not an identified motor torque curve. It is learned
only from bags assigned `train` in tools/split_manifest.json. Validation GNSS
is used exclusively for the optional 1/3/5 s synthetic wheel-blackout audit.
No holdout bag is read by this script.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "signals"))
from analyze_signals import DATA, latest, nearest, read_bag  # noqa: E402

MANIFEST = ROOT / "tools" / "split_manifest.json"
SUMMARY = ROOT / "analysis" / "signals" / "notch_speed_dwell_summary.csv"
OUT = ROOT / "evaluation" / "drive_calibration_train.json"
ASSET_CSV = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "drive_accel_table.csv"
SPEED_BINS = [(1.0, 2.0), (2.0, 3.0), (3.0, 5.0), (5.0, 8.0), (8.0, 12.0), (12.0, 20.0)]
VEHICLES = ("30618", "30639")


def weighted_median(values: list[float], weights: list[float]) -> float:
    ordered = sorted(zip(values, weights))
    halfway = 0.5 * sum(weights)
    cumulative = 0.0
    for value, weight in ordered:
        cumulative += weight
        if cumulative >= halfway:
            return float(value)
    return float(ordered[-1][0])


def build_table(train_bags: set[str]) -> dict:
    bins = defaultdict(list)
    with SUMMARY.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if raw["bag"] not in train_bags or float(raw["dwell_min"]) != 1.0:
                continue
            lo, hi = float(raw["speed_lo"]), float(raw["speed_hi"])
            if (lo, hi) not in SPEED_BINS:
                continue
            n = int(raw["n"])
            bins[(raw["train"], int(raw["notch"]), lo, hi)].append({
                "bag": raw["bag"], "n": n,
                "median_a": float(raw["median_a"]),
                "mean_a": float(raw["sum_a"]) / n,
                "sum_a": float(raw["sum_a"]),
                "sum_a2": float(raw["sum_a2"]),
            })
    table = {vehicle: {} for vehicle in VEHICLES}
    for (vehicle, notch, lo, hi), rows in sorted(bins.items()):
        values = [row["median_a"] for row in rows]
        weights = [min(row["n"], 100) for row in rows]
        center = weighted_median(values, weights)
        bag_mad = 1.4826 * weighted_median([abs(value - center) for value in values], weights)
        n = sum(row["n"] for row in rows)
        pooled_mean = sum(row["sum_a"] for row in rows) / n
        pooled_var = max(0.0, sum(row["sum_a2"] for row in rows) / n - pooled_mean**2)
        key = f"{lo:g}-{hi:g}"
        table[vehicle].setdefault(str(notch), {})[key] = {
            "speed_lo_mps": lo, "speed_hi_mps": hi,
            "sample_count": n, "bag_count": len(rows),
            "bag_capped_weighted_median_accel_mps2": round(center, 5),
            "pooled_mean_accel_mps2": round(pooled_mean, 5),
            "bag_mad_accel_mps2": round(bag_mad, 5),
            "sample_sd_accel_mps2": round(math.sqrt(pooled_var), 5),
            "usable_for_interpolation": bool(n >= 80 and len(rows) >= 3),
        }
    return table


def physics_accel(vehicle: str, notch: float, speed: float) -> float:
    """Current C++ grey-box acceleration with no adaptive bias or drive lag."""
    mass = 27500.0 if vehicle == "30639" else 27000.0
    torque = 590.0 if vehicle == "30639" else 600.0
    demand = max(-1.0, min(1.0, notch / 15.0))
    if demand > 0:
        torque_force = 4.0 * torque * 6.0 * 0.84 / 0.34
        rolloff = 1.0 / (1.0 + (speed / 23.0) ** 2)
        power_force = 200000.0 / max(1.5, speed)
        force = demand**0.75 * min(torque_force * rolloff, power_force)
    elif demand < 0:
        low_speed = 0.5 + 0.5 * max(0.0, min(1.0, speed / 1.5))
        force = -30000.0 * low_speed * (-demand)**0.72
    else:
        force = 0.0
    drag = 0.018 + 0.00055 * speed * speed if speed > 0.05 else 0.0
    return max(-4.0, min(3.0, force / mass - drag))


def empirical_accel(table: dict, vehicle: str, notch: int, speed: float) -> float:
    rows = table[vehicle].get(str(notch), {})
    supported = [row for row in rows.values() if row["usable_for_interpolation"]]
    if not supported:
        return physics_accel(vehicle, notch, speed)
    x = np.array([(row["speed_lo_mps"] + row["speed_hi_mps"]) / 2 for row in supported])
    y = np.array([row["bag_capped_weighted_median_accel_mps2"] for row in supported])
    order = np.argsort(x)
    return float(np.interp(speed, x[order], y[order]))


def prepare_validation_bag(bag: str) -> tuple[np.ndarray, ...] | None:
    arr = read_bag(next((DATA / bag).glob("*.db3")))
    master = arr.get("/sensing/gnss/master/vel", np.empty((0, 3)))
    rover = arr.get("/sensing/gnss/rover/vel", np.empty((0, 3)))
    if len(master) < 100 or len(rover) < 100:
        return None
    t = master[:, 0] / 1e9
    g = master[:, 2]
    rv, _ = nearest(t, rover, 0.15)
    front, _ = nearest(t, arr["/vehicle/front_bogie_velocity"], 0.15)
    rear, _ = nearest(t, arr["/vehicle/rear_bogie_velocity"], 0.15)
    cmd = arr["/vehicle/driver_position_cmd"]
    notch, _ = latest(t, cmd, 0.2)
    good = np.isfinite(g + rv + front + rear + notch) & (g < 20) & (rv < 20)
    good &= (np.abs(g - rv) < 0.3) & (np.abs(front - rear) < 0.25)
    return t, g, front, rear, cmd, good


def predict_window(table: dict, vehicle: str, t0: float, v0: float,
                   cmd_time: np.ndarray, cmd_value: np.ndarray,
                   duration: float, model: str) -> tuple[float, float]:
    step = 0.05
    speed, distance = v0, 0.0
    j = max(0, int(np.searchsorted(cmd_time, t0, side="right") - 1))
    start_notch = int(cmd_value[j])
    drive_state = start_notch / 15.0
    filtered_accel = empirical_accel(table, vehicle, start_notch, speed)
    for k in range(int(round(duration / step))):
        current_time = t0 + (k + 0.5) * step
        j = max(0, int(np.searchsorted(cmd_time, current_time, side="right") - 1))
        notch = int(cmd_value[j])
        if model == "hold":
            accel = 0.0
        elif model == "physics":
            desired = notch / 15.0
            tau = 0.27 if desired < drive_state else 0.45
            drive_state += (desired - drive_state) * (1.0 - math.exp(-step / tau))
            # The C++ force law operates on the filtered drive state, not an
            # integer notch. This continuous version matches its equations.
            accel = physics_accel(vehicle, drive_state * 15.0, speed)
        else:
            desired_accel = empirical_accel(table, vehicle, notch, speed)
            tau = 0.27 if desired_accel < filtered_accel else 0.45
            filtered_accel += (desired_accel - filtered_accel) * (1.0 - math.exp(-step / tau))
            accel = filtered_accel
        after = max(0.0, min(25.0, speed + accel * step))
        distance += 0.5 * (speed + after) * step
        speed = after
    return speed, distance


def validation_stress(table: dict, bags: list[str]) -> dict:
    rows = []
    for bag in bags:
        prepared = prepare_validation_bag(bag)
        if prepared is None:
            continue
        t, g, front, rear, cmd, good = prepared
        cmd_time = cmd[:, 0] / 1e9
        cmd_value = cmd[:, 2]
        start_times = np.arange(math.ceil(t[0] / 30.0) * 30.0, t[-1] - 5.1, 30.0)
        for start in start_times:
            i = int(np.searchsorted(t, start))
            if i >= len(t) or abs(t[i] - start) > 0.08 or not good[i] or g[i] < 0.5:
                continue
            # Require GNSS receiver agreement throughout the full synthetic
            # blackout, so a false GNSS spike cannot masquerade as model loss.
            j = int(np.searchsorted(t, t[i] + 5.0))
            if j >= len(t) or j - i < 45 or not np.all(good[i:j + 1]):
                continue
            if np.max(np.diff(t[i:j + 1])) > 0.25:
                continue
            v0 = 0.5 * (front[i] + rear[i])
            if abs(v0 - g[i]) > 0.30:
                continue
            for horizon in (1.0, 3.0, 5.0):
                end_t = t[i] + horizon
                truth_speed = float(np.interp(end_t, t[i:j + 1], g[i:j + 1]))
                mask = (t[i:j + 1] <= end_t)
                local_t = np.r_[t[i:j + 1][mask], end_t]
                local_g = np.r_[g[i:j + 1][mask], truth_speed]
                truth_distance = float(np.trapz(local_g, local_t))
                result = {"bag": bag, "vehicle": bag[:5], "horizon_s": horizon,
                          "start_speed_mps": v0, "truth_end_speed_mps": truth_speed,
                          "start_notch": int(cmd_value[max(0, np.searchsorted(cmd_time, t[i], side="right") - 1)])}
                for name in ("hold", "physics", "empirical"):
                    speed, distance = predict_window(table, bag[:5], t[i], v0,
                                                     cmd_time, cmd_value, horizon, name)
                    result[name + "_end_speed_error_mps"] = speed - truth_speed
                    result[name + "_distance_error_m"] = distance - truth_distance
                rows.append(result)
        print(f"stress {bag}: {sum(r['bag'] == bag for r in rows)} windows", flush=True)
    summary = {}
    for horizon in (1.0, 3.0, 5.0):
        for vehicle in (*VEHICLES, "all"):
            part = [r for r in rows if r["horizon_s"] == horizon and
                    (vehicle == "all" or r["vehicle"] == vehicle)]
            if not part:
                continue
            cell = {"windows": len(part), "bags": len({r["bag"] for r in part})}
            for name in ("hold", "physics", "empirical"):
                ve = np.asarray([r[name + "_end_speed_error_mps"] for r in part])
                de = np.asarray([r[name + "_distance_error_m"] for r in part])
                cell[name] = {
                    "speed_rmse_mps": round(float(np.sqrt(np.mean(ve**2))), 5),
                    "speed_mae_mps": round(float(np.mean(np.abs(ve))), 5),
                    "speed_bias_mps": round(float(np.mean(ve)), 5),
                    "distance_rmse_m": round(float(np.sqrt(np.mean(de**2))), 5),
                }
            summary[f"{vehicle}:{horizon:g}s"] = cell
    return {"protocol": "Validation-only, non-overlapping 30 s start grid, clean 5 s GNSS and both bogies, 0.05 s causal command simulation; each start evaluated at 1/3/5 s", "summary": summary, "windows": rows}


def main() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    train_bags = set(manifest["representatives"]["train"])
    table = build_table(train_bags)
    strong = {v: sum(row["usable_for_interpolation"] for notch in table[v].values()
                     for row in notch.values()) for v in VEHICLES}
    print("usable empirical cells:", strong)
    stress = validation_stress(table, manifest["representatives"]["validation"])
    result = {
        "schema_version": 1,
        "source": "train representatives only; per-bag notch_speed_dwell_summary rows filtered by frozen split",
        "training_bag_count": len(train_bags),
        "derivative": "1.1 s centered Savitzky-Golay of master GNSS speed; offline only",
        "selection": "master/rover speed agreement <0.3 m/s; bogies <0.25 m/s; average bogie/master <0.5 m/s; notch dwell >=1 s",
        "uncertainty": "Each cell reports sample SD and bag-to-bag robust MAD; sparse cells fall back to grey-box physics",
        "speed_bins_mps": [list(bin_) for bin_ in SPEED_BINS],
        "table": table,
        "usable_cells_by_vehicle": strong,
        "validation_synthetic_dropout": stress,
    }
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    ASSET_CSV.parent.mkdir(parents=True, exist_ok=True)
    with ASSET_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["vehicle", "notch", "speed_bin", "accel", "uncertainty", "count"])
        for vehicle in VEHICLES:
            for notch_text, speed_rows in sorted(table[vehicle].items(), key=lambda item: int(item[0])):
                notch = int(notch_text)
                for speed_bin, cell in speed_rows.items():
                    if not cell["usable_for_interpolation"]:
                        continue
                    uncertainty = max(0.2, cell["bag_mad_accel_mps2"],
                                      0.5 * cell["sample_sd_accel_mps2"])
                    if notch == -8:
                        uncertainty = max(uncertainty, 0.35)
                    if notch == -15:
                        uncertainty = max(uncertainty, 0.75)
                    writer.writerow([vehicle, notch_text, speed_bin,
                                     f"{cell['bag_capped_weighted_median_accel_mps2']:.5f}",
                                     f"{uncertainty:.5f}", cell["sample_count"]])
    print(json.dumps(stress["summary"], ensure_ascii=False, indent=2))
    print(OUT)
    print(ASSET_CSV)


if __name__ == "__main__":
    main()
