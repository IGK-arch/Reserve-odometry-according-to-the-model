"""Explore controller notch response with GNSS validity, speed and dwell controls."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

from analyze_signals import DATA, latest, nearest, read_bag

OUT = Path(__file__).resolve().parent
UNIQUE = pd.read_csv(OUT / 'duplicates.csv').drop_duplicates('file_sha256')
SPEED_BINS = [0, .5, 1, 2, 3, 5, 8, 12, 20]


def run_lengths(value: np.ndarray, dt: float) -> np.ndarray:
    last = 0
    dwell = np.zeros(len(value))
    for i in range(1, len(value)):
        if value[i] != value[i-1] or not np.isfinite(value[i]):
            last = i
        dwell[i] = (i-last)*dt
    return dwell


records = []
episodes = []
for _, row in UNIQUE.iterrows():
    bag = row.bag
    arr = read_bag(next((DATA / bag).glob('*.db3')))
    m = arr.get('/sensing/gnss/master/vel', np.empty((0,3)))
    rv = arr.get('/sensing/gnss/rover/vel', np.empty((0,3)))
    if len(m) < 25 or len(rv) < 25:
        continue
    mt = m[:,0]/1e9
    grid = np.arange(mt[0]+1, mt[-1]-1, .1)
    if len(grid) < 25:
        continue
    g = np.interp(grid, mt, m[:,2])
    # Offline derivative only; the live estimator must use causal histories.
    smooth = savgol_filter(g, 11, 2)
    accel = np.gradient(smooth, .1)
    rover, _ = nearest(grid, rv, .15)
    front, _ = nearest(grid, arr['/vehicle/front_bogie_velocity'], .15)
    rear, _ = nearest(grid, arr['/vehicle/rear_bogie_velocity'], .15)
    cmd, _ = latest(grid, arr['/vehicle/driver_position_cmd'], .2)
    dwell = run_lengths(cmd, .1)
    mean_wheel = (front+rear)/2
    valid = np.isfinite(g+rover+front+rear+cmd)
    valid &= (g < 20) & (rover < 20) & (abs(g-rover) < .3)
    valid &= (abs(front-rear) < .25) & (abs(mean_wheel-g) < .5)
    train = bag[:5]
    for notch in range(-15,16):
        sel = valid & (cmd == notch)
        for low, high in zip(SPEED_BINS[:-1], SPEED_BINS[1:]):
            for minimum_dwell in [0, .5, 1.0, 2.0]:
                mask = sel & (g >= low) & (g < high) & (dwell >= minimum_dwell)
                if np.any(mask):
                    records.append(dict(bag=bag,train=train,notch=notch,speed_lo=low,
                                        speed_hi=high,dwell_min=minimum_dwell,n=int(mask.sum()),
                                        sum_a=float(np.sum(accel[mask])),
                                        sum_a2=float(np.sum(accel[mask]**2)),
                                        mean_speed=float(np.mean(g[mask])),
                                        median_a=float(np.median(accel[mask]))))

    # Identify each contiguous command episode at -15 from raw command stamps.
    cmd_raw = arr['/vehicle/driver_position_cmd']
    vals = cmd_raw[:,2]
    starts = np.flatnonzero((vals == -15) & np.r_[True,vals[:-1] != -15])
    for start in starts:
        end = start
        while end+1 < len(vals) and vals[end+1] == -15:
            end += 1
        ts = cmd_raw[start,0]/1e9
        te = cmd_raw[end,0]/1e9
        if te-ts < .2:
            continue
        entry = float(np.interp(ts,mt,m[:,2]))
        exit_ = float(np.interp(te,mt,m[:,2]))
        rover_entry, _ = nearest(np.array([ts]),rv,.2)
        rover_exit, _ = nearest(np.array([te]),rv,.2)
        episodes.append(dict(bag=bag,train=train,start_rel=ts-mt[0],duration=te-ts,
                             entry_speed=entry,exit_speed=exit_,delta_speed=exit_-entry,
                             previous_notch=vals[start-1] if start>0 else np.nan,
                             gnss_consistent=bool(np.isfinite(rover_entry[0]+rover_exit[0])
                               and abs(rover_entry[0]-entry)<.3 and abs(rover_exit[0]-exit_)<.3)))

pd.DataFrame(records).to_csv(OUT/'notch_speed_dwell_summary.csv',index=False)
pd.DataFrame(episodes).to_csv(OUT/'minus15_episodes.csv',index=False)

df = pd.DataFrame(records)
fig, axes = plt.subplots(1,2,figsize=(13,5.5),sharey=True)
for ax, train in zip(axes,['30618','30639']):
    sub = df[(df.train==train) & (df.dwell_min==1.0)]
    for speed_lo,speed_hi,color in [(0,1,'tab:gray'),(1,3,'tab:orange'),(3,8,'tab:blue'),(8,20,'tab:green')]:
        part = sub[(sub.speed_lo>=speed_lo)&(sub.speed_hi<=speed_hi)]
        agg = part.groupby('notch').agg(n=('n','sum'),sa=('sum_a','sum'))
        agg = agg[agg.n>=100]
        # Keep absent positions as gaps; joining distant notches implies data we
        # do not have, especially in the sparse negative-command range.
        mean_a = (agg.sa/agg.n).reindex(np.arange(-15,16))
        ax.plot(mean_a.index,mean_a,marker='o',markersize=3,lw=1.4,
                color=color,label=f'{speed_lo}–{speed_hi} m/s')
    ax.axhline(0,color='black',lw=.6)
    ax.set_title(f'Tram {train}')
    ax.set_xlabel('Controller position')
    ax.set_xticks([-15,-10,-5,0,5,10,15])
    ax.grid(alpha=.25)
axes[0].set_ylabel('Mean GNSS-derived acceleration, m/s²')
axes[1].legend(loc='lower right',fontsize=8)
fig.suptitle('Notch response, GNSS-consistent periods; notch held ≥1 s; ≥100 samples/point')
fig.tight_layout()
fig.savefig(OUT/'notch_response_speed_dwell.png',dpi=170)
plt.close(fig)

print('rows',len(records),'episodes',len(episodes))
epi = pd.DataFrame(episodes)
for train,part in epi.groupby('train'):
    good = part[part.gnss_consistent]
    print(train,'minus15 episodes',len(part),'consistent',len(good),
          'entry speed median',good.entry_speed.median(),
          'entry <1 fraction',np.mean(good.entry_speed<1),
          'duration median',good.duration.median(),
          'duration p90',good.duration.quantile(.9),
          'mean acceleration median',(good.delta_speed/good.duration).median())
