#!/usr/bin/env python3
"""Render a static result figure from archived reports; never fit a metric."""
from pathlib import Path
import argparse
import json
import os
import tempfile

os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'tram-matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT / 'docs/round2_comparison.png')
    args = parser.parse_args()
    results = ROOT / 'evaluation/results'
    original = load(results / 'ros_original_1x_run_summary.json')
    previous = load(results / 'ros_final_1x_run_summary.json')
    final = load(results / 'round2_final/ros_run_summary.json')
    if any(r['status'] != 'completed' for r in (original, previous, final)):
        raise ValueError('Only completed official ROS runs may enter the figure')
    speed = load(results / 'speed_study_final.json')['runs']['validation']['session_summaries']
    by_variant = {row['variant']: row for row in speed}
    gnss = load(results / 'round2_final/navigation_train_validation.json')['sessions']['2026-08-26']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle('Резервная одометрия • проверенный результат', x=.06, ha='left', fontsize=21, weight='bold')
    gray, blue = '#8e9bad', '#176fbd'
    ax = axes[0, 0]
    values = [r['official_checker']['position']['distance']['rmse'] for r in (original, previous, final)]
    bars = ax.bar(['Исходный код', 'Первый раунд', 'Итог'], values, color=[gray, '#74afd6', blue], width=.58)
    ax.bar_label(bars, labels=[f'{v:.3f}' for v in values], padding=5)
    ax.set_ylim(0, max(values)*1.2)
    ax.set_ylabel('3D RMSE, м')
    ax.set_title('Положение • официальный ROS checker, 1×', loc='left', pad=14, weight='bold')
    ax.grid(axis='y', alpha=.18); ax.set_axisbelow(True)
    ax = axes[0, 1]
    for key, label, color in [('baseline', 'Первый раунд', gray), ('candidate', 'Итог', blue)]:
        values = [by_variant[key][f'blackout{t}']['speed_rmse_mps'] for t in [1, 3, 5]]
        ax.plot([1, 3, 5], values, 'o-', label=label, color=color, linewidth=2.5)
    ax.set_xticks([1, 3, 5]); ax.set_ylim(0, .65)
    ax.set_xlabel('Время после потери обоих колёс, с'); ax.set_ylabel('RMSE скорости, м/с')
    ax.set_title('Обрыв колёс • 284 окна validation', loc='left', pad=14, weight='bold')
    ax.legend(frameon=False); ax.grid(alpha=.18)
    ax = axes[1, 0]
    x = np.arange(2)
    for key, label, color, offset in [('baseline', 'Первый раунд', gray, -.18), ('candidate', 'Итог', blue, .18)]:
        values = [gnss[mode][f'{key}_pooled_rmse_m'] for mode in ['sparse', 'startup']]
        bars = ax.bar(x+offset, values, .34, color=color, label=label)
        ax.bar_label(bars, labels=[f'{v:.3f}' for v in values], padding=4)
    ax.set_xticks(x, ['Редкие GNSS', 'GNSS только первые 30 с'])
    ax.set_ylim(0, 3.5); ax.set_ylabel('3D RMSE, м')
    ax.set_title('GNSS-прокси • validation, включая регрессию', loc='left', pad=14, weight='bold')
    ax.legend(frameon=False); ax.grid(axis='y', alpha=.18); ax.set_axisbelow(True)
    ax = axes[1, 1]; ax.axis('off')
    resource = final['runtime']['resources']
    latency = final['runtime']['latency']['position']
    velocity = [r['official_checker']['velocity']['rmse'] for r in (previous, final)]
    lines = [
        ('Среда', 'Ubuntu 22.04 / ROS 2 Humble, arm64'),
        ('Лимит контейнера', '2 CPU / 512 MiB'),
        ('Пиковая память ноды', f"{resource['memoryPeakKB']/1024:.2f} MiB"),
        ('CPU ноды, среднее', f"{resource['cpuPercentOneCoreMean']:.2f}% одного ядра"),
        ('Вход → положение, p95', f"{latency['p95_ms']:.2f} мс (наблюдатель DDS)"),
        ('Скорость, ROS RMSE', f'{velocity[0]:.6f} → {velocity[1]:.6f} м/с'),
    ]
    ax.text(0, 1, 'Ресурсы и скорость • итоговый прогон', va='top', weight='bold', fontsize=12)
    for i, (name, value) in enumerate(lines):
        y=.83-i*.135
        ax.text(0, y, name, color='#4b5563', fontsize=10)
        ax.text(.51, y, value, fontsize=10, weight='medium')
    fig.text(.06, .03, 'Один открытый эталон уже использован для диагностики. GNSS-прокси и искусственные обрывы не являются скрытым score.', color='#555', fontsize=10)
    fig.subplots_adjust(left=.075, right=.97, bottom=.11, top=.88, hspace=.5, wspace=.29)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160, facecolor='white')
    plt.close(fig)
    print(args.out)


if __name__ == '__main__':
    main()
