"""Robust baseline with GNSS agreement gate and exact event diagnostics."""

from pathlib import Path

import numpy as np
import pandas as pd

from analyze_signals import DATA, latest, nearest, read_bag

dup = pd.read_csv(Path(__file__).resolve().parent / 'duplicates.csv')
unique = dup.drop_duplicates('file_sha256')
all_rows = []
for _, info in unique.iterrows():
    bag = info.bag
    arr = read_bag(next((DATA / bag).glob('*.db3')))
    master = arr.get('/sensing/gnss/master/vel', np.empty((0, 3)))
    rover = arr.get('/sensing/gnss/rover/vel', np.empty((0, 3)))
    if not len(master) or not len(rover):
        continue
    t = master[:, 0] / 1e9
    gnss = master[:, 2]
    rv, _ = nearest(t, rover)
    front, _ = nearest(t, arr['/vehicle/front_bogie_velocity'])
    rear, _ = nearest(t, arr['/vehicle/rear_bogie_velocity'])
    cmd, _ = latest(t, arr['/vehicle/driver_position_cmd'])
    good = np.isfinite(front + rear + gnss + rv) & (gnss < 25) & (rv < 25)
    good &= (abs(rv - gnss) < .3)
    # A standstill agreement at zero is acceptable; a single zeroed GNSS is not.
    avg = (front + rear) / 2
    error = avg - gnss
    for mode, mask in [('all_agree', good), ('nominal', good & (abs(front - rear) < .25) & (abs(error) < .5))]:
        if np.any(mask):
            all_rows.append(dict(bag=bag,train=bag[:5],mode=mode,n=int(mask.sum()),
                                 total=int(len(t)),sum_e=float(np.sum(error[mask])),
                                 sum_abs=float(np.sum(abs(error[mask]))),
                                 sum_e2=float(np.sum(error[mask] ** 2)),
                                 sum_front_e2=float(np.sum((front[mask]-gnss[mask]) ** 2)),
                                 sum_rear_e2=float(np.sum((rear[mask]-gnss[mask]) ** 2))))

df = pd.DataFrame(all_rows)
df.to_csv(Path(__file__).resolve().parent / 'robust_baseline_by_bag.csv', index=False)
for (train, mode), part in df.groupby(['train','mode']):
    n = part.n.sum()
    print(train, mode, 'bags',len(part),'n',n,'coverage',n/part.total.sum(),
          'RMSE',np.sqrt(part.sum_e2.sum()/n),'MAE',part.sum_abs.sum()/n,
          'bias',part.sum_e.sum()/n,
          'front_RMSE',np.sqrt(part.sum_front_e2.sum()/n),
          'rear_RMSE',np.sqrt(part.sum_rear_e2.sum()/n))

# Check whether the long apparent shared-wheel discrepancy resembles a lag.
bag = '30618_4d487b0d'
arr = read_bag(next((DATA / bag).glob('*.db3')))
origin = min(x[0, 0] for x in arr.values()) / 1e9
master = arr['/sensing/gnss/master/vel']
rover = arr['/sensing/gnss/rover/vel']
front = arr['/vehicle/front_bogie_velocity']
rear = arr['/vehicle/rear_bogie_velocity']
cmd = arr['/vehicle/driver_position_cmd']
t = master[:, 0] / 1e9
g = master[:, 2]
rv, _ = nearest(t, rover)
fg, _ = nearest(t, front)
rg, _ = nearest(t, rear)
cg, _ = latest(t, cmd)
for lo, hi in [(101.467,116.667),(127.267,132.967),(147.867,158.867),(135,170)]:
    m = (t-origin >= lo) & (t-origin <= hi)
    print('window',lo,hi,'n',m.sum(),'master',np.nanmean(g[m]),'rover',np.nanmean(rv[m]),
          'front',np.nanmean(fg[m]),'rear',np.nanmean(rg[m]),
          'cmd',np.nanmin(cg[m]),np.nanmax(cg[m]),np.nanmean(cg[m]),
          'master-rover MAE',np.nanmean(abs(g[m]-rv[m])))

lo, hi = 147.867, 158.867
mask = (t-origin >= lo) & (t-origin <= hi)
wm_t = (front[:,0] / 1e9)
wm_v = front[:,2]
rm_t = (rear[:,0] / 1e9)
rm_v = rear[:,2]
shifts = np.arange(-2.0, 2.001, .01)
errs = []
for shift in shifts:
    w = (np.interp(t[mask]+shift,wm_t,wm_v)+np.interp(t[mask]+shift,rm_t,rm_v))/2
    errs.append(np.sqrt(np.mean((w-g[mask])**2)))
idx = int(np.argmin(errs))
print('lag-fit',bag,lo,hi,'best_shift_s',shifts[idx],'rmse_best',errs[idx],
      'rmse_zero',errs[int(np.argmin(abs(shifts)))])

bag = '30639_e4379d7f'
arr = read_bag(next((DATA / bag).glob('*.db3')))
origin = min(x[0,0] for x in arr.values())/1e9
for name in ['/sensing/gnss/master/vel','/sensing/gnss/rover/vel']:
    x = arr.get(name,np.empty((0,3)))
    if len(x):
        ix = np.argmax(x[:,2])
        print('gnss-spike',bag,name,'n',len(x),'t',x[ix,0]/1e9-origin,'speed',x[ix,2])
    else:
        print('gnss-spike',bag,name,'absent')
