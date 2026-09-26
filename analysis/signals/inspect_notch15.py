"""Inspect a few atypical -15 episodes using both GNSS and both axles."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from analyze_signals import DATA, latest, nearest, read_bag

OUT = Path(__file__).resolve().parent
CASES = [
    ('30618_27e994fc', 1795, 1828),
    ('30618_bab2fe58', 15, 32),
    ('30639_c31df386', 1327, 1338),
]

fig, axes = plt.subplots(3, 1, figsize=(12, 8), layout='constrained')
for ax, (bag, lo, hi) in zip(axes, CASES):
    arr = read_bag(next((DATA / bag).glob('*.db3')))
    base = arr['/sensing/gnss/master/vel'][0, 0] / 1e9
    raw_cmd = arr['/vehicle/driver_position_cmd']
    raw_cmd = raw_cmd[(raw_cmd[:, 0] / 1e9 - base >= lo) &
                      (raw_cmd[:, 0] / 1e9 - base <= hi)]
    gaps = np.diff(raw_cmd[:, 0]) / 1e9
    print(bag, 'command messages:', len(raw_cmd),
          'median gap:', np.median(gaps), 'max gap:', max(gaps))
    grid = base + np.arange(lo, hi, .1)
    for topic, label, style in [
        ('/sensing/gnss/master/vel', 'GNSS master', '-'),
        ('/sensing/gnss/rover/vel', 'GNSS rover', '--'),
        ('/vehicle/front_bogie_velocity', 'Front wheel', '-'),
        ('/vehicle/rear_bogie_velocity', 'Rear wheel', '--'),
    ]:
        vals, _ = nearest(grid, arr[topic], .16)
        ax.plot(grid-base, vals, style, label=label, linewidth=1)
    cmd, _ = latest(grid, arr['/vehicle/driver_position_cmd'], .3)
    ax2 = ax.twinx()
    ax2.step(grid-base, cmd, where='post', color='black', alpha=.25,
             label='Driver command')
    ax2.set_ylim(-17, 3)
    ax2.set_ylabel('Notch')
    ax.set_ylabel('Speed, m/s')
    ax.set_title(bag)
    ax.grid(alpha=.2)
    ax.legend(loc='upper right', ncol=2, fontsize=8)
axes[-1].set_xlabel('Seconds from first GNSS master velocity sample')
fig.savefig(OUT / 'minus15_examples.png', dpi=170)
