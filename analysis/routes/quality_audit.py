"""Quantify GNSS anomalies, timing and repeated route geometry."""
import csv
import json
from collections import Counter

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from analyze_gnss import DATA, OUT, enu, extract, finite_positions


def nearest(a,b):
    j=np.searchsorted(b[:,1],a[:,1]);j=np.clip(j,1,len(b)-1)
    j-=np.abs(b[j-1,1]-a[:,1])<np.abs(b[j,1]-a[:,1])
    return j,np.abs(b[j,1]-a[:,1])


def quant(x):
    return [float(z) for z in np.quantile(x,[.01,.1,.5,.9,.99])] if len(x) else []


def analyze_one(path):
    _,_,a=extract(path)
    mf,rf,mv=[a[k] for k in ('mf','rf','mv')]
    result={'bag':path.parent.name,'master_fix_n':len(mf),'rover_fix_n':len(rf)}
    if len(mf)<2:return result
    pm=enu(mf[:,2],mf[:,3],mf[:,4])
    dt=np.diff(mf[:,1]);ds=np.linalg.norm(np.diff(pm,axis=0),axis=1)
    good_dt=(dt>.05)&(dt<.2)
    rate=np.zeros(len(dt));rate[good_dt]=ds[good_dt]/dt[good_dt]
    result['step_count']=int(good_dt.sum())
    result['apparent_speed_gt_20mps_count']=int(np.count_nonzero(good_dt&(rate>20)))
    result['apparent_speed_gt_30mps_count']=int(np.count_nonzero(good_dt&(rate>30)))
    result['apparent_speed_gt_50mps_count']=int(np.count_nonzero(good_dt&(rate>50)))
    result['max_apparent_speed_mps']=float(rate.max())
    result['status_at_gt_30mps']=dict(Counter(int(x) for x in mf[1:][good_dt&(rate>30),5]))
    if len(mv)>1:
        jm,dvm=nearest(mf,mv)
        speed=np.linalg.norm(mv[jm,2:5],axis=1)
        result['master_vel_speed_gt_20mps_count']=int(np.count_nonzero((dvm<.06)&(speed>20)))
        result['master_vel_speed_mps_q']=quant(speed[dvm<.06])
    if len(rf)>1:
        jr,dtr=nearest(mf,rf)
        good=dtr<.06
        pr=enu(rf[jr[good],2],rf[jr[good],3],rf[jr[good],4]);delta=pr-pm[good]
        distance=np.linalg.norm(delta,axis=1);err=np.abs(distance-12.44)
        result['paired_n']=int(good.sum())
        result['pair_distance_m_q']=quant(distance)
        result['pair_sep_err_gt_1m_count']=int(np.count_nonzero(err>1))
        result['pair_sep_err_gt_5m_count']=int(np.count_nonzero(err>5))
        result['pair_sep_err_gt_20m_count']=int(np.count_nonzero(err>20))
        result['pair_sep_err_gt_5m_status']=dict(Counter(int(x) for x in mf[good,5][err>5]))
        result['pair_sep_err_gt_5m_rover_status']=dict(Counter(int(x) for x in rf[jr[good],5][err>5]))
    return result


def main():
    paths=sorted(DATA.glob('*/*.db3'))
    results=[]
    for i,p in enumerate(paths,1):
        if i%10==0:print(i,'/',len(paths),flush=True)
        results.append(analyze_one(p))
    with (OUT/'quality_summary.json').open('w') as f:json.dump(results,f,indent=2)
    total=lambda k:sum(r.get(k,0) for r in results)
    ag={k:total(k) for k in ['step_count','apparent_speed_gt_20mps_count','apparent_speed_gt_30mps_count','apparent_speed_gt_50mps_count','master_vel_speed_gt_20mps_count','paired_n','pair_sep_err_gt_1m_count','pair_sep_err_gt_5m_count','pair_sep_err_gt_20m_count']}
    ag['jump_status_counts']=dict(sum((Counter(r.get('status_at_gt_30mps',{})) for r in results),Counter()))
    ag['worst_bags_by_sep5m']=sorted([[r['bag'],r.get('pair_sep_err_gt_5m_count',0),r.get('paired_n',0)] for r in results],key=lambda x:-x[1])[:12]
    ag['worst_bags_by_speedjump']=sorted([[r['bag'],r.get('apparent_speed_gt_30mps_count',0),r.get('step_count',0)] for r in results],key=lambda x:-x[1])[:12]
    with (OUT/'quality_global.json').open('w') as f:json.dump(ag,f,indent=2)
    print(json.dumps(ag,indent=2))


if __name__=='__main__':main()
