"""Empirically check the wheel-speed unit against clean GNSS on one bag.

Offline diagnostic only. GNSS is never a proposed runtime estimator input.
Usage: py -3.12 analysis/viewer/verify_units.py 30618_0e41eac3
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from rosbag_cdr import messages


ROOT = Path(__file__).resolve().parents[2]
TOPICS = {
    "/vehicle/front_bogie_velocity", "/vehicle/rear_bogie_velocity",
    "/sensing/gnss/master/vel", "/sensing/gnss/rover/vel",
}


def nearest(query_t: np.ndarray, samples: np.ndarray, max_dt: float = 0.15) -> np.ndarray:
    t = samples[:, 0]
    right = np.searchsorted(t, query_t)
    left = np.clip(right - 1, 0, len(t) - 1)
    right = np.clip(right, 0, len(t) - 1)
    idx = np.where(np.abs(t[left] - query_t) <= np.abs(t[right] - query_t), left, right)
    result = samples[idx, 1].copy()
    result[np.abs(t[idx] - query_t) > max_dt] = np.nan
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_id", nargs="?", default="30618_0e41eac3")
    args = parser.parse_args()
    rows: dict[str, list[tuple[float, float]]] = {name: [] for name in TOPICS}
    for topic, _, msg in messages(ROOT / "dataset" / "data" / args.bag_id, TOPICS):
        speed = (msg["velocity_raw"] if topic.startswith("/vehicle/")
                 else float(np.linalg.norm(msg["linear"])))
        rows[topic].append((msg["stamp_ns"] / 1e9, speed))
    samples = {name: np.array(sorted(pairs), dtype=float).reshape(-1, 2)
               for name, pairs in rows.items()}
    if any(not len(a) for a in samples.values()):
        raise ValueError("Choose a bag containing both wheel channels and both GNSS velocities")
    master = samples["/sensing/gnss/master/vel"]
    q = master[:, 0]
    gnss = master[:, 1]
    rover = nearest(q, samples["/sensing/gnss/rover/vel"])
    front = nearest(q, samples["/vehicle/front_bogie_velocity"])
    rear = nearest(q, samples["/vehicle/rear_bogie_velocity"])
    wheel_raw = (front + rear) / 2
    good = np.isfinite(rover + wheel_raw) & (gnss > 1) & (gnss < 25)
    good &= (np.abs(rover - gnss) < 0.3) & (np.abs(front - rear) < 0.9)
    if not np.any(good):
        raise ValueError("No aligned clean samples")
    raw, truth = wheel_raw[good], gnss[good]
    slope = float(np.dot(raw, truth) / np.dot(truth, truth))
    raw_rmse = float(np.sqrt(np.mean((raw - truth) ** 2)))
    converted_rmse = float(np.sqrt(np.mean((raw / 3.6 - truth) ** 2)))
    print(f"{args.bag_id}: n={len(raw):,}, raw/GNSS slope={slope:.4f}")
    print(f"Raw-as-m/s RMSE={raw_rmse:.3f} m/s; raw/3.6 RMSE={converted_rmse:.3f} m/s")


if __name__ == "__main__":
    main()
