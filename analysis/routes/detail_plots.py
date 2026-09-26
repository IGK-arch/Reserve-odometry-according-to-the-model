"""Focused GNSS route visualizations and antenna geometry; run with py -3.12."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from analyze_gnss import DATA, OUT, enu, extract, finite_positions


BAGS = [
    '30618_0e41eac3',  # east to west, mostly good master
    '30618_0f120b35',  # west to east
    '30639_584b6e32',  # other vehicle
    '30618_0686195f',  # degraded master, long path outliers
]


def load(bag):
    _,_,a=extract(DATA/bag/f'{bag}_0.db3')
    for k in ('mf','rf'):
        a[k]=finite_positions(a[k])
    return a


def nearest(a,b):
    j=np.searchsorted(b[:,1],a[:,1]); j=np.clip(j,1,len(b)-1)
    j-=np.abs(b[j-1,1]-a[:,1]) < np.abs(b[j,1]-a[:,1])
    return j, np.abs(b[j,1]-a[:,1])


def smooth_track(p,half=10):
    # Median filter removes short GNSS multipath spikes for visualization only.
    from scipy.ndimage import median_filter
    return median_filter(p,size=(2*half+1,1),mode='nearest')


def antenna(a):
    mf,rf,mv=a['mf'],a['rf'],a['mv']
    im,dt=nearest(mf,rf)
    iv,dv=nearest(mf,mv)
    good=(dt<.06)&(dv<.06)
    if not np.any(good): return None
    p1=enu(mf[good,2],mf[good,3],mf[good,4])
    p2=enu(rf[im[good],2],rf[im[good],3],rf[im[good],4])
    d=p2-p1
    v=mv[iv[good],2:5]
    speed=np.linalg.norm(v[:,:2],axis=1)
    moving=speed>1
    along=np.sum(d[moving,:2]*(v[moving,:2]/speed[moving,None]),axis=1)
    cross=(d[moving,0]*v[moving,1]-d[moving,1]*v[moving,0])/speed[moving]
    return {'n':int(good.sum()),'n_moving':int(moving.sum()),
            'along_m':np.quantile(along,[.01,.1,.5,.9,.99]).tolist(),
            'cross_m':np.quantile(cross,[.01,.1,.5,.9,.99]).tolist(),
            'vertical_m':np.quantile(d[:,2],[.01,.1,.5,.9,.99]).tolist(),
            'distance_m':np.quantile(np.linalg.norm(d,axis=1),[.01,.1,.5,.9,.99]).tolist(),
            'master_status0_frac':float(np.mean(mf[good,5]==0)),
            'rover_status0_frac':float(np.mean(rf[im[good],5]==0))}


def main():
    tracks={b:load(b) for b in BAGS}
    fig,axs=plt.subplots(2,1,figsize=(14,8),height_ratios=(3,1),sharex=True)
    colors=['#1875bd','#db5c22','#2a9655','#9233aa']
    for bag,color in zip(BAGS,colors):
        a=tracks[bag]; p=enu(a['mf'][:,2],a['mf'][:,3],a['mf'][:,4]); pn=smooth_track(p)
        axs[0].plot(pn[::20,0],pn[::20,1],color=color,lw=1,label=bag)
        axs[1].plot(pn[::20,0],pn[::20,2],color=color,lw=1,label=bag)
    axs[0].set(ylabel='North, m',title='Representative master GNSS routes in local ENU'); axs[0].legend(loc='lower right',fontsize=8)
    axs[1].set(xlabel='East, m',ylabel='Up, m');
    for ax in axs: ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(OUT/'representative_routes.png',dpi=170);plt.close(fig)

    fig,axs=plt.subplots(1,2,figsize=(13,5))
    for bag,color in zip(BAGS[:3],colors):
        a=tracks[bag];p=enu(a['mf'][:,2],a['mf'][:,3],a['mf'][:,4]);pn=smooth_track(p)
        axs[0].plot(pn[::10,0],pn[::10,1],color=color,lw=1,label=bag)
        axs[1].plot(pn[::10,0],pn[::10,1],color=color,lw=1,label=bag)
    axs[0].set(xlim=(-2600,-2000),ylim=(-850,-650),title='Middle route: parallel track comparison')
    axs[1].set(xlim=(-4700,-4400),ylim=(-1300,-1050),title='Western terminal geometry')
    for ax in axs: ax.grid(alpha=.3); ax.set(xlabel='East, m',ylabel='North, m');ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(OUT/'route_zoom.png',dpi=180);plt.close(fig)

    report={bag:antenna(a) for bag,a in tracks.items()}
    with (OUT/'antenna_examples.json').open('w') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
