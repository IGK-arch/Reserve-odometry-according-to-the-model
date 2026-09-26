"""Create read-only diagnostic plots from one ROS 2 bag without ROS installed.

Usage (from repository root):
    py -3.12 analysis/viewer/make_plots.py 30618_0e41eac3
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer

from rosbag_cdr import messages


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "dataset" / "data"
OUT = ROOT / "analysis" / "viewer" / "figures"
TOPICS = {
    "/vehicle/front_bogie_velocity",
    "/vehicle/rear_bogie_velocity",
    "/vehicle/driver_position_cmd",
    "/sensing/gnss/master/fix",
    "/sensing/gnss/master/vel",
    "/sensing/gnss/rover/fix",
    "/sensing/gnss/rover/vel",
}


def load(bag_id: str) -> dict[str, list[dict]]:
    result: dict[str, list[dict]] = defaultdict(list)
    for topic, receive_ns, msg in messages(DATA / bag_id, TOPICS):
        msg["receive_ns"] = receive_ns
        result[topic].append(msg)
    return result


def times_and_values(items: list[dict], key: str, origin_ns: int) -> tuple[np.ndarray, np.ndarray]:
    ordered = sorted(items, key=lambda m: m["stamp_ns"])
    t = np.array([(m["stamp_ns"] - origin_ns) / 1e9 for m in ordered], dtype=float)
    v = np.array([m[key] for m in ordered], dtype=float)
    return t, v


def chosen_gnss(data: dict[str, list[dict]], suffix: str) -> tuple[str, list[dict]]:
    for receiver in ("master", "rover"):
        topic = f"/sensing/gnss/{receiver}/{suffix}"
        if data.get(topic):
            return receiver, data[topic]
    return "none", []


def gnss_speed(items: list[dict], origin_ns: int) -> tuple[np.ndarray, np.ndarray]:
    ordered = sorted(items, key=lambda m: m["stamp_ns"])
    t = np.array([(m["stamp_ns"] - origin_ns) / 1e9 for m in ordered])
    v = np.array([np.linalg.norm(m["linear"]) for m in ordered])
    return t, v


def split_gaps(t: np.ndarray, v: np.ndarray, threshold_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Insert NaN at missing stretches so plotting does not invent measurements."""
    if len(t) < 2:
        return t, v
    gaps = np.flatnonzero(np.diff(t) > threshold_s)
    if not len(gaps):
        return t, v
    return (np.insert(t, gaps + 1, (t[gaps] + t[gaps + 1]) / 2),
            np.insert(v, gaps + 1, np.nan))


def nearest_age(t: np.ndarray, sample_t: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(sample_t, t)
    before = np.abs(t - sample_t[np.clip(idx - 1, 0, len(sample_t) - 1)])
    after = np.abs(t - sample_t[np.clip(idx, 0, len(sample_t) - 1)])
    return np.minimum(before, after)


def local_enu(items: list[dict]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ordered = sorted(items, key=lambda m: m["stamp_ns"])
    transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    lat0, lon0, h0 = 55.805, 37.425, 170.0
    x0, y0, z0 = transformer.transform(lon0, lat0, h0)
    lon = np.array([m["longitude"] for m in ordered])
    lat = np.array([m["latitude"] for m in ordered])
    h = np.array([m["altitude"] for m in ordered])
    x, y, z = transformer.transform(lon, lat, h)
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = np.deg2rad(lat0), np.deg2rad(lon0)
    east = -np.sin(lam) * dx + np.cos(lam) * dy
    north = -np.sin(phi) * np.cos(lam) * dx - np.sin(phi) * np.sin(lam) * dy + np.cos(phi) * dz
    up = np.cos(phi) * np.cos(lam) * dx + np.cos(phi) * np.sin(lam) * dy + np.sin(phi) * dz
    return east, north, up


def plot_bag(bag_id: str) -> None:
    data = load(bag_id)
    origins = [m["stamp_ns"] for topic in TOPICS if topic.startswith("/vehicle/") for m in data.get(topic, [])]
    if not origins:
        raise ValueError(f"No vehicle data in {bag_id}")
    origin_ns = min(origins)
    tf, vf_raw = times_and_values(data["/vehicle/front_bogie_velocity"], "velocity_raw", origin_ns)
    tr, vr_raw = times_and_values(data["/vehicle/rear_bogie_velocity"], "velocity_raw", origin_ns)
    tc, cmd = times_and_values(data["/vehicle/driver_position_cmd"], "position", origin_ns)
    vf, vr = vf_raw / 3.6, vr_raw / 3.6
    receiver, vel_msgs = chosen_gnss(data, "vel")
    tg, vg = gnss_speed(vel_msgs, origin_ns) if vel_msgs else (np.array([]), np.array([]))
    fix_receiver, fix_msgs = chosen_gnss(data, "fix")

    fig, axes = plt.subplots(4, 1, figsize=(15, 10), sharex=True, constrained_layout=True)
    pf_t, pf_v = split_gaps(tf, vf, 0.6)
    pr_t, pr_v = split_gaps(tr, vr, 0.6)
    axes[0].plot(pf_t / 60, pf_v, linewidth=0.8, label="front / 3.6")
    axes[0].plot(pr_t / 60, pr_v, linewidth=0.8, alpha=0.75, label="rear / 3.6")
    if len(tg):
        pg_t, pg_v = split_gaps(tg, vg, 0.6)
        axes[0].plot(pg_t / 60, pg_v, linewidth=0.8, alpha=0.8, label=f"GNSS {receiver} |v|")
    axes[0].set_ylabel("Speed, m/s")
    axes[0].legend(loc="upper right", ncol=3)

    pc_t, pc_v = split_gaps(tc, cmd.astype(float), 0.25)
    axes[1].step(pc_t / 60, pc_v, where="post", linewidth=0.8)
    axes[1].set_ylabel("Controller notch")
    axes[1].set_ylim(-16, 16)

    if len(tf) and len(tr):
        common = (tf >= tr[0]) & (tf <= tr[-1]) & (nearest_age(tf, tr) <= 0.3)
        delta = vf[common] - np.interp(tf[common], tr, vr)
        pd_t, pd_v = split_gaps(tf[common], delta, 0.6)
        axes[2].plot(pd_t / 60, pd_v, linewidth=0.8)
        axes[2].axhline(0, color="gray", linewidth=0.5)
    axes[2].set_ylabel("Front − rear, m/s")

    if len(tg) and len(tf) and len(tr):
        valid = (tg >= max(tf[0], tr[0])) & (tg <= min(tf[-1], tr[-1])) & (vg > 0.5)
        valid &= (nearest_age(tg, tf) <= 0.3) & (nearest_age(tg, tr) <= 0.3)
        wheel_mean = 0.5 * (np.interp(tg[valid], tf, vf) + np.interp(tg[valid], tr, vr))
        error = wheel_mean - vg[valid]
        pe_t, pe_v = split_gaps(tg[valid], error, 0.6)
        axes[3].plot(pe_t / 60, pe_v, linewidth=0.7)
        axes[3].axhline(0, color="gray", linewidth=0.5)
        axes[3].set_ylabel("Wheels − GNSS, m/s")
        rmse = float(np.sqrt(np.mean(error**2))) if len(error) else float("nan")
        mae = float(np.mean(np.abs(error))) if len(error) else float("nan")
        subtitle = f"raw wheel−GNSS RMSE {rmse:.3f} m/s; MAE {mae:.3f} m/s; n={len(error):,}"
    else:
        subtitle = "No GNSS velocity to compare"
        axes[3].set_ylabel("No GNSS")
    axes[3].set_xlabel("Time from first vehicle header, min")
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.suptitle(f"{bag_id} · {subtitle}")
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{bag_id}_signals.png", dpi=160)
    plt.close(fig)

    if fix_msgs:
        east, north, up = local_enu(fix_msgs)
        fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
        axes[0].plot(east, north, linewidth=1)
        axes[0].scatter(east[0], north[0], marker="o", label="start")
        axes[0].scatter(east[-1], north[-1], marker="x", label="finish")
        axes[0].set_aspect("equal", adjustable="box")
        axes[0].set_xlabel("East from (55.805°, 37.425°), m")
        axes[0].set_ylabel("North, m")
        axes[0].grid(alpha=0.2)
        axes[0].legend()
        axes[1].plot(np.arange(len(up)), up, linewidth=0.8)
        axes[1].set_xlabel("GNSS fix sample index")
        axes[1].set_ylabel("Up from 170 m ellipsoid height, m")
        axes[1].grid(alpha=0.2)
        fig.suptitle(f"{bag_id} · GNSS {fix_receiver} trajectory")
        fig.savefig(OUT / f"{bag_id}_route.png", dpi=160)
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_ids", nargs="+", help="Bag directory IDs under dataset/data")
    args = parser.parse_args()
    for bag_id in args.bag_ids:
        plot_bag(bag_id)
        print(bag_id)


if __name__ == "__main__":
    main()
