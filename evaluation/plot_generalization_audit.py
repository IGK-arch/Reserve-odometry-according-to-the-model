"""Render the frozen session-split audit without reading any raw rosbag.

Usage: python evaluation/plot_generalization_audit.py
Inputs: evaluation/audit_stats.json; output: docs/audit_generalization.png.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
AUDIT = ROOT / "evaluation" / "audit_stats.json"
OUT = ROOT / "docs" / "audit_generalization.png"


def main() -> None:
    data = json.loads(AUDIT.read_text(encoding="utf-8"))["sets"]
    cohorts = [
        ("Валидация\n30618", data["validation"]["summaries"]["30618"]),
        ("Holdout\n30618", data["holdout"]["summaries"]["30618"]),
        ("Holdout\n30639", data["holdout"]["summaries"]["30639"]),
    ]
    measures = [
        ("speed", "pooled_baseline_rmse_mps", "pooled_core_rmse_mps",
         "RMSE скорости, м/с"),
        ("distance_abs_end_drift_pct", "median_baseline", "median_core",
         "Медианный |конечный дрейф|, %"),
        ("position_base_link_3d_rmse_m", "median_baseline", "median_core",
         "Медианный 3D RMSE, м"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 5.4))
    fig.subplots_adjust(top=0.82, bottom=0.25, left=0.055, right=0.985,
                        wspace=0.20)
    labels = [name for name, _ in cohorts]
    x = np.arange(len(cohorts))
    for ax, (section, baseline_key, core_key, title) in zip(axes, measures):
        base = [summary[section][baseline_key] for _, summary in cohorts]
        core = [summary[section][core_key] for _, summary in cohorts]
        ax.bar(x - 0.19, base, width=0.37, color="#64748b", label="Среднее колёс")
        ax.bar(x + 0.19, core, width=0.37, color="#0f766e", label="Модель")
        ax.set_title(title, fontsize=11, fontweight="bold")
        ax.set_xticks(x, labels)
        ax.tick_params(axis="x", labelsize=9)
        ax.grid(axis="y", alpha=0.22)
        ax.set_axisbelow(True)
        ax.set_ylim(0, max(base + core) * 1.23)
        for i, (b, c) in enumerate(zip(base, core)):
            digits = 3 if section == "speed" else 2
            ax.text(i - 0.19, b + max(base + core) * 0.018, f"{b:.{digits}f}",
                    ha="center", va="bottom", fontsize=8, color="#334155")
            ax.text(i + 0.19, c + max(base + core) * 0.018, f"{c:.{digits}f}",
                    ha="center", va="bottom", fontsize=8, color="#115e59")
    handles, names = axes[0].get_legend_handles_labels()
    fig.legend(handles, names, loc="lower center", ncol=2,
               bbox_to_anchor=(0.5, 0.115), frameon=False)
    fig.suptitle("Проверка переноса по датам и трамваям: меньше лучше",
                 fontsize=13, y=0.94)
    fig.text(0.5, 0.04,
             "Локальный GNSS-прокси; независимых сессий: 1 validation, 2 holdout. "
             "Holdout участвовал в выборе режима.",
             ha="center", fontsize=8, color="#475569")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(OUT)


if __name__ == "__main__":
    main()
