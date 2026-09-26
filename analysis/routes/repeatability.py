"""Track reuse and route branching from sampled GNSS master positions."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter
from scipy.spatial import cKDTree
from analyze_gnss import OUT


def main():
    arr=np.load(OUT/'sampled_tracks.npz')
    summaries=json.loads((OUT/'summaries.json').read_text())
    bybag={x['bag']:x for x in summaries}
    refs={
        'out':arr['30618_0e41eac3'],
        'return':arr['30618_0f120b35'],
    }
    trees={}
    for direction,p in refs.items():
        p=median_filter(p,size=(5,1),mode='nearest')
        corridor=(p[:,0]>-4300)&(p[:,0]<-200)
        trees[direction]=cKDTree(p[corridor,:2])
    rows=[]
    for bag in arr.files:
        s=bybag[bag]['master_path']
        if s['net_m']<4400:continue
        direction='out' if s['end_enu'][0]<s['start_enu'][0] else 'return'
        p=median_filter(arr[bag],size=(5,1),mode='nearest')
        corridor=(p[:,0]>-4300)&(p[:,0]<-200)
        if corridor.sum()<100:continue
        d,_=trees[direction].query(p[corridor,:2])
        rows.append({'bag':bag,'direction':direction,'vehicle':s and bag.split('_')[0],
                     'n':int(len(d)),'median_m':float(np.median(d)),'p90_m':float(np.quantile(d,.9)),
                     'p99_m':float(np.quantile(d,.99)),'frac_gt_10m':float(np.mean(d>10)),
                     'frac_gt_20m':float(np.mean(d>20))})
    print('full_bags',len(rows),'directions',{k:sum(r['direction']==k for r in rows) for k in ['out','return']})
    print('median repeatability q',np.quantile([r['median_m'] for r in rows],[0,.1,.5,.9,1]))
    print('p90 q',np.quantile([r['p90_m'] for r in rows],[0,.5,.9,1]))
    print('worst median',sorted([(r['bag'],r['median_m'],r['p90_m'],r['frac_gt_20m']) for r in rows],key=lambda x:-x[1])[:12])
    outp=median_filter(refs['out'],size=(5,1),mode='nearest');retp=median_filter(refs['return'],size=(5,1),mode='nearest')
    c=(outp[:,0]>-4300)&(outp[:,0]<-200)
    rt=cKDTree(retp[(retp[:,0]>-4300)&(retp[:,0]<-200),:2])
    dist,_=rt.query(outp[c,:2])
    print('out_vs_return_nearest_distance',np.quantile(dist,[.01,.1,.5,.9,.99]))
    (OUT/'repeatability.json').write_text(json.dumps(rows,indent=2))
    fig,axs=plt.subplots(2,1,figsize=(12,8))
    compare=['30618_0e41eac3','30639_9c362687','30639_44226bde','30639_253671cc']
    for bag in compare:
        p=median_filter(arr[bag],size=(5,1),mode='nearest')
        for ax in axs:ax.plot(p[:,0],p[:,1],lw=1,label=bag)
    axs[0].set(xlim=(-4400,-200),ylim=(-1300,-400),title='Selected route variants')
    axs[1].set(xlim=(-2600,-1800),ylim=(-850,-550),title='Middle segment zoom')
    for ax in axs:ax.grid(alpha=.3);ax.legend(fontsize=8);ax.set(xlabel='East, m',ylabel='North, m')
    fig.tight_layout();fig.savefig(OUT/'variant_routes.png',dpi=170);plt.close(fig)

if __name__=='__main__':main()
