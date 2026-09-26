"""Export targeted windows from bag messages and make comparison plots."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from analyze_signals import DATA, OUT, latest, nearest, read_bag


WINDOWS = [
    ("30618_33bec73f", 99, 115, "rear_spin"),
    ("30618_2050d396", 396, 408, "rear_brake_slide"),
    ("30618_2050d396", 440, 447, "both_brake_slide"),
    ("30639_50956d6e", 76, 103, "front_spin"),
    ("30618_4d487b0d", 97, 170, "shared_wheel_vs_gnss"),
    ("30639_e4379d7f", 525, 565, "gnss_spike"),
]


def main():
    for bag, low, high, label in WINDOWS:
        arr = read_bag(next((DATA / bag).glob('*.db3')))
        origin = min(x[0, 0] for x in arr.values() if len(x)) / 1e9
        master = arr.get('/sensing/gnss/master/vel', np.empty((0, 3)))
        rover = arr.get('/sensing/gnss/rover/vel', np.empty((0, 3)))
        front = arr['/vehicle/front_bogie_velocity']
        rear = arr['/vehicle/rear_bogie_velocity']
        cmd = arr['/vehicle/driver_position_cmd']
        t = master[:, 0] / 1e9 if len(master) else front[:, 0] / 1e9
        mask = (t - origin >= low) & (t - origin <= high)
        t = t[mask]
        g = master[:, 2][mask] if len(master) else np.full(len(t), np.nan)
        rv, _ = nearest(t, rover)
        f, _ = nearest(t, front)
        r, _ = nearest(t, rear)
        c, _ = latest(t, cmd)
        rel = t - origin
        rows = [dict(t_rel_s=round(float(a), 4), front_mps=b, rear_mps=d,
                     cmd=int(e) if np.isfinite(e) else '', master_mps=h, rover_mps=i)
                for a, b, d, e, h, i in zip(rel, f, r, c, g, rv)]
        csv_path = OUT / f'{bag}_{label}.csv'
        with csv_path.open('w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        fig, ax = plt.subplots(figsize=(12, 4.8))
        ax.plot(rel, g, lw=1.6, label='GNSS master')
        ax.plot(rel, rv, lw=1.1, label='GNSS rover')
        ax.plot(rel, f, lw=1.2, label='front /3.6')
        ax.plot(rel, r, lw=1.2, label='rear /3.6')
        ax.set_ylabel('Speed, m/s')
        ax.set_xlabel('Seconds from first message stamp')
        ax.grid(alpha=.3)
        ax2 = ax.twinx()
        ax2.step(rel, c, color='gray', where='post', lw=.7, alpha=.6, label='controller')
        ax2.set_ylabel('Controller notch')
        ax2.set_ylim(-16, 16)
        handles, labels = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(handles + h2, labels + l2, loc='upper right', fontsize=8)
        ax.set_title(f'{bag}: {label}')
        fig.tight_layout()
        fig.savefig(OUT / f'{bag}_{label}.png', dpi=140)
        plt.close(fig)
        print(bag, label, len(rows), f'cmd {np.nanmin(c):.0f}..{np.nanmax(c):.0f}')


if __name__ == '__main__':
    main()
