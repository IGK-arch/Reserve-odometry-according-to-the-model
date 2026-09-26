"""Reproducible GNSS audit of the supplied ROS 2 SQLite bags.

Run: py -3.12 analysis/routes/analyze_gnss.py
Reads only dataset/data; writes summaries, compressed trajectory samples and PNGs here.
"""

from __future__ import annotations

import csv
import json
import math
import sqlite3
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer
from scipy.spatial import cKDTree

from inspect_cdr import fix, vel


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DATA = ROOT / 'dataset' / 'data'
TOPICS = {
    '/sensing/gnss/master/fix': 'mf',
    '/sensing/gnss/rover/fix': 'rf',
    '/sensing/gnss/master/vel': 'mv',
    '/sensing/gnss/rover/vel': 'rv',
}
ORIGIN = (37.462266845, 55.810367065, 168.3794)  # lon, lat, altitude
ecef = Transformer.from_crs(4979, 4978, always_xy=True)
origin_ecef = np.array(ecef.transform(*ORIGIN))
lon0, lat0, _ = ORIGIN
phi, lam = math.radians(lat0), math.radians(lon0)
ROT = np.array([
    [-math.sin(lam), math.cos(lam), 0],
    [-math.sin(phi)*math.cos(lam), -math.sin(phi)*math.sin(lam), math.cos(phi)],
    [math.cos(phi)*math.cos(lam), math.cos(phi)*math.sin(lam), math.sin(phi)],
])


def enu(lat, lon, alt):
    x, y, z = ecef.transform(lon, lat, alt)
    return (ROT @ (np.array([x, y, z]) - origin_ecef[:, None])).T


def finite_positions(a):
    if not len(a):
        return a
    return a[np.isfinite(a[:, 2:5]).all(axis=1) &
             (a[:, 2] > 50) & (a[:, 2] < 60) &
             (a[:, 3] > 30) & (a[:, 3] < 45)]


def quant(a, qs=(0.0, .5, .9, .95, .99, 1.0)):
    return dict(zip((str(q) for q in qs), map(float, np.quantile(a, qs)))) if len(a) else {}


def extract(path):
    db = sqlite3.connect(path)
    topic_ids = {ident: TOPICS[name] for ident, name in db.execute('SELECT id,name FROM topics') if name in TOPICS}
    records = {k: [] for k in TOPICS.values()}
    for tid, recorded_ns, raw in db.execute('SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp' %
                                    ','.join(map(str, topic_ids))):
        typ = topic_ids[tid]
        if typ.endswith('f'):
            h, status, service, lat, lon, alt, cov, cov_type, *_ = fix(raw)
            records[typ].append((recorded_ns / 1e9, h[0] + h[1] / 1e9, lat, lon, alt,
                                 status, service, cov_type, cov[0], cov[4], cov[8]))
        else:
            h, twist, *_ = vel(raw)
            records[typ].append((recorded_ns / 1e9, h[0] + h[1] / 1e9, *twist))
    begin, end = db.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()
    db.close()
    arrays = {k: np.asarray(v, dtype=float).reshape(-1, 11 if k.endswith('f') else 8)
              for k,v in records.items()}
    return begin / 1e9, end / 1e9, arrays


def path_stats(a):
    if len(a) < 2:
        return {}
    p = enu(a[:, 2], a[:, 3], a[:, 4])
    dt = np.diff(a[:, 1]); ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
    good = (dt > 0.01) & (dt < 5) & (ds < 50)
    return {
        'start_enu': p[0].tolist(), 'end_enu': p[-1].tolist(),
        'enu_min': p.min(axis=0).tolist(), 'enu_max': p.max(axis=0).tolist(),
        'length_m_clipped_steps': float(ds[good].sum()),
        'net_m': float(np.linalg.norm(p[-1]-p[0])),
        'step_m': quant(ds[dt > 0]),
        'derived_speed_mps': quant(ds[good] / dt[good]),
        'height_range_m': float(np.ptp(p[:, 2])),
        'alt_range_m': [float(a[:,4].min()), float(a[:,4].max())],
    }


def topic_stats(a, begin, end, is_fix):
    if not len(a):
        return {'count': 0}
    lag = a[:,0]-a[:,1]
    interval = np.diff(a[:,1])
    valid_interval = interval[interval > 0]
    result = {
        'count': int(len(a)),
        'first_offset_from_bag_s': float(a[0,0]-begin),
        'last_offset_from_bag_end_s': float(end-a[-1,0]),
        'header_first_offset_from_bag_s': float(a[0,1]-begin),
        'header_last_offset_from_bag_end_s': float(end-a[-1,1]),
        'header_span_s': float(a[-1,1]-a[0,1]),
        'record_minus_header_s': quant(lag),
        'header_interval_s': quant(valid_interval),
        'nonpositive_header_intervals': int(np.count_nonzero(interval <= 0)),
        'gaps_gt_1s': int(np.count_nonzero(interval > 1)),
        'max_gap_s': float(interval.max()) if len(interval) else None,
    }
    if is_fix:
        result['status_counts'] = {str(int(k)): int(v) for k,v in zip(*np.unique(a[:,5],return_counts=True))}
        result['service_counts'] = {str(int(k)): int(v) for k,v in zip(*np.unique(a[:,6],return_counts=True))}
        result['cov_type_counts'] = {str(int(k)): int(v) for k,v in zip(*np.unique(a[:,7],return_counts=True))}
        result['nonzero_cov_xyz_count'] = int(np.count_nonzero(np.any(a[:,8:11] != 0, axis=1)))
        result['valid_position_count'] = int(len(finite_positions(a)))
    else:
        result['speed_norm_mps'] = quant(np.linalg.norm(a[:,2:5],axis=1))
        result['nonzero_angular_count'] = int(np.count_nonzero(np.any(a[:,5:8] != 0,axis=1)))
    return result


def pair_stats(a, b):
    if len(a) < 2 or len(b) < 2:
        return {}
    ia = np.searchsorted(b[:,1], a[:,1]); ia = np.clip(ia, 1, len(b)-1)
    ia -= np.abs(b[ia-1,1]-a[:,1]) < np.abs(b[ia,1]-a[:,1])
    dt = b[ia,1] - a[:,1]
    close = np.abs(dt) <= .2
    if not close.any():
        return {'paired_count': 0, 'time_offset_s': quant(dt)}
    out = {'paired_count': int(close.sum()), 'time_offset_s': quant(dt[close])}
    if a.shape[1] == 11:
        am, bm = a[close], b[ia[close]]
        delta = enu(bm[:,2],bm[:,3],bm[:,4]) - enu(am[:,2],am[:,3],am[:,4])
        out['offset_enu_m_median'] = np.median(delta, axis=0).tolist()
        out['offset_enu_m_mean'] = delta.mean(axis=0).tolist()
        out['offset_3d_m'] = quant(np.linalg.norm(delta,axis=1))
        out['offset_horizontal_m'] = quant(np.linalg.norm(delta[:,:2],axis=1))
        out['delta'] = delta
    else:
        delta = b[ia[close],2:5]-a[close,2:5]
        out['velocity_delta_norm_mps'] = quant(np.linalg.norm(delta,axis=1))
    return out


def main():
    paths = sorted(DATA.glob('*/*.db3'))
    summaries = []
    all_m = []
    all_r = []
    all_offsets = []
    per_bag_points = {}
    for i,path in enumerate(paths,1):
        begin,end,a = extract(path)
        bag = path.parent.name
        mf,rf,mv,rv = [a[k] for k in ('mf','rf','mv','rv')]
        mvalid, rvalid = finite_positions(mf),finite_positions(rf)
        ps=pair_stats(mvalid,rvalid)
        vs=pair_stats(mv,rv)
        if 'delta' in ps:
            all_offsets.append(ps.pop('delta'))
        summary = {
            'bag': bag, 'vehicle': bag.split('_')[0], 'bag_duration_s': end-begin,
            'topics': {k: topic_stats(a[k],begin,end,k.endswith('f')) for k in a},
            'master_path': path_stats(mvalid), 'rover_path': path_stats(rvalid),
            'master_rover_fix': ps, 'master_rover_vel': vs,
        }
        summaries.append(summary)
        if len(mvalid):
            p=enu(mvalid[:,2],mvalid[:,3],mvalid[:,4]); idx=np.arange(0,len(p),max(1,len(p)//2000))
            per_bag_points[bag] = {'p':p[idx], 't':mvalid[idx,1], 'status':mvalid[idx,5]}
            all_m.append(p[::max(1,len(p)//5000)])
        if len(rvalid):
            p=enu(rvalid[:,2],rvalid[:,3],rvalid[:,4]); all_r.append(p[::max(1,len(p)//5000)])
        print(f'{i}/{len(paths)} {bag} mf={len(mf)} rf={len(rf)} mv={len(mv)} rv={len(rv)}',flush=True)
    with (OUT/'summaries.json').open('w',encoding='utf-8') as f:
        json.dump(summaries,f,indent=2,ensure_ascii=False)
    np.savez_compressed(OUT/'sampled_tracks.npz', **{k:v['p'] for k,v in per_bag_points.items()})
    with (OUT/'bag_summary.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f)
        w.writerow(['bag','vehicle','duration_s','master_fix_count','rover_fix_count','master_vel_count','rover_vel_count','master_length_m','net_m','start_e','start_n','end_e','end_n','alt_min','alt_max','master_rover_median_m'])
        for s in summaries:
            p=s['master_path']; mr=s['master_rover_fix']
            w.writerow([s['bag'],s['vehicle'],s['bag_duration_s'],*[s['topics'][k]['count'] for k in ('mf','rf','mv','rv')],
                        p.get('length_m_clipped_steps'),p.get('net_m'),
                        *(p.get('start_enu',[None,None])[:2]),*(p.get('end_enu',[None,None])[:2]),
                        *(p.get('alt_range_m',[None,None])),mr.get('offset_horizontal_m',{}).get('0.5')])

    positive=[s for s in summaries if s['topics']['mf']['count']]
    master=np.concatenate(all_m) if all_m else np.zeros((0,3))
    offsets=np.concatenate(all_offsets) if all_offsets else np.zeros((0,3))
    global_result = {
        'bag_count':len(summaries),'master_fix_bag_count':len(positive),
        'master_fix_count':int(sum(s['topics']['mf']['count'] for s in summaries)),
        'rover_fix_count':int(sum(s['topics']['rf']['count'] for s in summaries)),
        'master_vel_count':int(sum(s['topics']['mv']['count'] for s in summaries)),
        'rover_vel_count':int(sum(s['topics']['rv']['count'] for s in summaries)),
        'overall_enu_min':master.min(axis=0).tolist() if len(master) else None,
        'overall_enu_max':master.max(axis=0).tolist() if len(master) else None,
        'overall_master_rover_offset_enu_median_m':np.median(offsets,axis=0).tolist() if len(offsets) else None,
        'overall_master_rover_offset_m':quant(np.linalg.norm(offsets,axis=1)),
        'overall_master_rover_offset_horizontal_m':quant(np.linalg.norm(offsets[:,:2],axis=1)),
        'status_total':dict(Counter(str(k) for s in summaries for k in [])),
    }
    for k in ('mf','rf'):
        c=Counter()
        for s in summaries:
            c.update({status:int(n) for status,n in s['topics'][k].get('status_counts',{}).items()})
        # Counter.update(mapping) adds counts, correctly aggregating bag summaries.
        global_result[k+'_status_counts']=dict(c)
    with (OUT/'global_summary.json').open('w',encoding='utf-8') as f:
        json.dump(global_result,f,indent=2)

    fig,ax=plt.subplots(figsize=(11,8)); cmap={'30618':'#1463b8','30639':'#d35d27'}
    for s in summaries:
        bag=s['bag']
        if bag in per_bag_points:
            p=per_bag_points[bag]['p']; ax.plot(p[:,0],p[:,1],color=cmap[s['vehicle']],alpha=.12,lw=.5)
    for vehicle,color in cmap.items():
        ax.plot([],[],color=color,label=vehicle)
    ax.set(title='GNSS master: all runs (ENU, origin at first example fix)',xlabel='East, m',ylabel='North, m')
    ax.axis('equal'); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(OUT/'all_routes.png',dpi=170); plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,4))
    counts=np.array([s['topics']['mf']['count'] for s in summaries]); axs[0].hist(counts,bins=25,color='#1463b8'); axs[0].set(xlabel='Master fix count per bag',ylabel='Bags',title='GNSS availability')
    if len(offsets):
        axs[1].hist(np.linalg.norm(offsets[:,:2],axis=1),bins=120,range=(0,60),color='#d35d27'); axs[1].set(xlabel='Master–rover horizontal offset, m',ylabel='Pairs',title='Receiver separation (0–60 m)')
        axs[1].axvline(12.44,color='#212a36',lw=1,ls='--',label='median ≈ 12.44 m');axs[1].set_yscale('log');axs[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(OUT/'gnss_quality.png',dpi=170); plt.close(fig)
    print(json.dumps(global_result,indent=2))


if __name__ == '__main__':
    main()
