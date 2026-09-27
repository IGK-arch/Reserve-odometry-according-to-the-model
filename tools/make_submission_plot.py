#!/usr/bin/env python3
"""Plot all valid nearest-header matches of the offline C++ output, without fitting."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'evaluation'))
from reference_benchmark import ReferenceIndex, finite_position, read_bag, read_candidate


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--metrics', type=Path, required=True)
    args = parser.parse_args()
    _, truth, _ = read_bag(args.bag)
    index = ReferenceIndex(truth)
    position, velocity = [], []
    for sample in read_candidate(args.candidate):
        ref = index.nearest(sample.stamp_ns)
        if ref is None:
            continue
        if finite_position(sample.position) and finite_position(ref.position):
            position.append((sample.stamp_ns, np.linalg.norm(np.subtract(sample.position, ref.position))))
        if sample.velocity_present and np.isfinite(sample.velocity_mps) and np.isfinite(ref.velocity_mps):
            velocity.append((sample.stamp_ns, sample.velocity_mps - ref.velocity_mps))
    metrics = json.loads(args.metrics.read_text())['candidate']
    p, v = np.array(position), np.array(velocity)
    p_rmse, v_rmse = float(np.sqrt(np.mean(p[:, 1]**2))), float(np.sqrt(np.mean(v[:, 1]**2)))
    assert len(p) == metrics['position']['matched']
    assert len(v) == metrics['velocity']['matched']
    assert abs(p_rmse - metrics['position']['rmse_3d_m']) < 1e-9
    assert abs(v_rmse - metrics['velocity']['rmse_mps']) < 1e-9
    start_ns = min(position[0][0], velocity[0][0])
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.2), sharex=True)
    fig.suptitle('Ошибка на полном контрольном прогоне 30618', fontsize=16, x=.09, ha='left')
    fig.text(.09, .905, 'Офлайн-пересчёт C++-ядра · ближайшая метка эталона ≤50 мс · прямое сравнение', fontsize=10, color='#526070')
    axes[0].plot((p[:, 0]-start_ns)/60e9, p[:, 1], color='#1766a1', linewidth=.7)
    axes[0].axhline(p_rmse, color='#b76b19', linestyle='--', linewidth=1.1,
                   label=f'Офлайн RMSE = {p_rmse:.3f} м')
    axes[0].set_ylabel('Пространственная ошибка, м')
    axes[0].set_ylim(bottom=0)
    axes[0].legend(loc='upper left', frameon=False)
    axes[1].plot((v[:, 0]-start_ns)/60e9, v[:, 1], color='#1766a1', linewidth=.55,
                 label=f'Офлайн RMSE = {v_rmse:.4f} м/с')
    axes[1].axhline(0, color='#64748b', linewidth=.6)
    axes[1].set_ylabel('Ошибка скорости, м/с')
    axes[1].set_xlabel('Время от первого сопоставленного отсчёта, мин')
    axes[1].legend(loc='upper left', frameon=False)
    for ax in axes:
        ax.grid(alpha=.18)
        ax.set_axisbelow(True)
        ax.set_xlim(0, max((p[-1, 0]-start_ns)/60e9, (v[-1, 0]-start_ns)/60e9))
    fig.subplots_adjust(left=.09, right=.98, top=.85, bottom=.10, hspace=.19)
    out = ROOT / 'docs/submission/reference_errors.png'
    fig.savefig(out, dpi=170, facecolor='white')
    plt.close(fig)
    metadata = {'method': 'nearest reference <=50 ms; every valid match; no alignment or filtering',
                'position_n': len(p), 'velocity_n': len(v), 'position_rmse_m': p_rmse,
                'velocity_rmse_mps': v_rmse, 'candidate_sha256': sha(args.candidate),
                'metrics_sha256': sha(args.metrics), 'script_sha256': sha(__file__),
                'plot_sha256': sha(out), 'command': sys.argv}
    out.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
