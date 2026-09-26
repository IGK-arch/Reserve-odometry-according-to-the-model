"""Make an honest, reproducible summary of validation proxy metrics.

These are local GNSS-based proxies, not the organisers' hidden score.
Run after core_benchmark.py and position_proxy.py have written their CSVs.
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluation"


def rows(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return {row["bag"]: row for row in csv.DictReader(handle)}


def main() -> None:
    physics = rows(EVAL / "validation_core_rawfreeze_fix.csv")
    table = rows(EVAL / "validation_core_table_optional.csv")
    positions = rows(EVAL / "validation_position_table.csv")
    bags = sorted(positions)
    labels = [bag.split("_", 1)[1] for bag in bags]
    base_speed = np.array([float(physics[bag]["baseline_rmse_mps"]) for bag in bags])
    physics_speed = np.array([float(physics[bag]["core_rmse_mps"]) for bag in bags])
    table_speed = np.array([float(table[bag]["core_rmse_mps"]) for bag in bags])
    base_pos = np.array([float(positions[bag]["baseline_base_rmse_3d_m"]) for bag in bags])
    table_pos = np.array([float(positions[bag]["base_rmse_3d_m"]) for bag in bags])

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True,
                             gridspec_kw={"hspace": 0.15})
    x = np.arange(len(bags))
    w = 0.27
    axes[0].bar(x - w, base_speed, w, color="#8b9aab", label="Среднее колёс")
    axes[0].bar(x, physics_speed, w, color="#6d8fa2", label="Физическая модель")
    axes[0].bar(x + w, table_speed, w, color="#078e8e", label="Модель + train-таблица")
    axes[0].set_ylabel("RMSE скорости, м/с")
    axes[0].set_title("Validation 30618: локальный GNSS-прокси, 13 уникальных bag")
    axes[0].grid(axis="y", alpha=0.2)
    axes[0].legend(ncol=3, loc="upper right", frameon=False)

    axes[1].bar(x - w / 2, base_pos, w, color="#8b9aab", label="Среднее колёс + карта")
    axes[1].bar(x + w / 2, table_pos, w, color="#078e8e", label="Модель + train-таблица + карта")
    axes[1].set_ylabel("3D RMSE base_link, м")
    axes[1].grid(axis="y", alpha=0.2)
    axes[1].legend(ncol=2, loc="upper right", frameon=False)
    axes[1].set_xticks(x, labels, rotation=50, ha="right")
    axes[1].set_xlabel("Идентификатор bag (суффикс)")
    fig.text(0.01, 0.01, "GNSS и две антенны используются только отдельным кодом оценки после replay. Скрытый эталон жюри недоступен.",
             fontsize=9, color="#3f4d58")
    fig.subplots_adjust(bottom=0.16, top=0.91, left=0.08, right=0.98)
    output = ROOT / "docs" / "validation_metrics.png"
    fig.savefig(output, dpi=170)
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
