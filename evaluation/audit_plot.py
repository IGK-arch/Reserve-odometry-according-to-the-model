"""Plot the recorded-session generalization gap from audited deployed data.

Run with Python 3.12 plus matplotlib: py -3.12 evaluation/audit_plot.py
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"
REPORT = json.loads((EVAL / "audit_stats.json").read_text(encoding="utf-8"))


def main():
    sessions = REPORT["speed_by_session"]
    order = ["train", "validation", "holdout"]
    sessions.sort(key=lambda x: (order.index(x["split"]), x["vehicle"], x["session_date"]))
    labels = [f"{s['vehicle']}\n{s['session_date'][5:]}\n{s['split']}" for s in sessions]
    base = [s["baseline_rmse_mps"] for s in sessions]
    core = [s["core_rmse_mps"] for s in sessions]
    x = np.arange(len(sessions))
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), height_ratios=[1.25, 1], layout="constrained")
    width = .37
    ax[0].bar(x - width / 2, base, width, color="#64748b", label="Wheel average")
    ax[0].bar(x + width / 2, core, width, color="#0f766e", label="Deployed estimator")
    ax[0].set_xticks(x, labels)
    ax[0].set_ylabel("Speed RMSE vs GNSS proxy, m/s")
    ax[0].set_title("Speed accuracy by vehicle/date session")
    ax[0].legend(loc="upper left")
    ax[0].grid(axis="y", alpha=.2)
    ax[0].set_axisbelow(True)
    for i, s in enumerate(sessions):
        if s["split"] == "train":
            ax[0].axvspan(i - .49, i + .49, color="#f8fafc", zorder=-2)
        ax[0].text(i + width / 2, core[i] + .002, f"{core[i]:.3f}", ha="center", va="bottom", fontsize=8)
    drift = []
    for split in ("validation", "holdout"):
        summaries = REPORT["sets"][split]["summaries"]
        for vehicle in ("30618", "30639"):
            d = summaries[vehicle]["distance_abs_end_drift_pct"]
            if d:
                drift.append((f"{vehicle}\n{split}", d["median_baseline"], d["median_core"], d["bags"]))
    y = np.arange(len(drift))
    ax[1].bar(y - width / 2, [r[1] for r in drift], width, color="#64748b")
    ax[1].bar(y + width / 2, [r[2] for r in drift], width, color="#0f766e")
    ax[1].set_xticks(y, [f"{r[0]}\n({r[3]} bags)" for r in drift])
    ax[1].set_ylabel("Median absolute final distance drift, %")
    ax[1].set_title("Integrated distance proxy on scored bags")
    ax[1].grid(axis="y", alpha=.2)
    ax[1].set_axisbelow(True)
    out = EVAL / "audit_generalization.png"
    fig.savefig(out, dpi=160)
    print(out)


if __name__ == "__main__":
    main()
