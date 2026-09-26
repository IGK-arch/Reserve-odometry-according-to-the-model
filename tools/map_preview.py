"""Render a static overview of the packaged route map (offline QA only)."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "route_map.csv"
OUT = ROOT / "docs" / "route_map_preview.png"


def main() -> None:
    rows = {"out": [], "return": []}
    with MAP.open(newline="", encoding="utf-8") as handle:
        for item in csv.DictReader(handle):
            rows[item["direction"]].append([float(item[field]) for field in ("s", "x", "y", "z")])
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), gridspec_kw={"height_ratios": [2.1, 1]}, dpi=150)
    palette = {"out": "#2159b5", "return": "#db6a25"}
    for direction, values in rows.items():
        a = np.asarray(values)
        axes[0].plot(a[:, 1], a[:, 2], color=palette[direction], lw=1.6,
                     label=f"{direction}: {a[-1, 0]:.0f} m")
        axes[0].scatter(a[0, 1], a[0, 2], s=60, color=palette[direction], marker="o", zorder=5)
        axes[0].scatter(a[-1, 1], a[-1, 2], s=70, color=palette[direction], marker="X", zorder=5)
        axes[1].plot(a[:, 0], a[:, 3], color=palette[direction], lw=1.4, label=direction)
    axes[0].set(title="Offline route map: master GNSS antenna, WGS84 ENU",
                xlabel="East (m)", ylabel="North (m)")
    axes[0].axis("equal")
    axes[1].set(title="Elevation profile relative to fixed ENU origin",
                xlabel="Distance along direction (m)", ylabel="Up (m)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, bbox_inches="tight")
    plt.close(fig)
    print(OUT)


if __name__ == "__main__":
    main()
