#!/usr/bin/env python3
"""Train-only official XY compatibility and isolated baseline replay experiment.

No fused truth is read in this program. Official geometry is base_link; map
coordinates are GNSS master. Paired-GNSS proxy is a diagnostic, not true rails.
"""
from pathlib import Path
import sys,json,csv,hashlib,argparse,subprocess
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import median_filter,gaussian_filter1d
R=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(R/'evaluation'),str(R/'tools')]
import navigation_gnss_stress as g
from route_map_io import RouteMap

def stats(x):
 x=np.asarray(x);return {'n':len(x),'rmse':float(np.sqrt(np.mean(x*x))),'median':float(np.median(x)),'p90':float(np.quantile(x,.9)),'p95':float(np.quantile(x,.95)),'max':float(np.max(x))} if len(x) else {'n':0}
def project(p,line):
 tree=cKDTree(line[:,:2]);_,ix=tree.query(p[:,:2],k=3)
 ids=np.clip(np.concatenate([ix,ix-1],axis=1),0,len(line)-2)
 a=line[ids,:2];d=line[ids+1,:2]-a
 f=np.clip(np.sum((p[:,None,:2]-a)*d,axis=2)/np.sum(d*d,axis=2),0,1)
 q=a+f[:,:,None]*d;dist=np.linalg.norm(p[:,None,:2]-q,axis=2);best=np.argmin(dist,axis=1);rows=np.arange(len(p))
 j=ids[rows,best];return q[rows,best],dist[rows,best],j,f[rows,best]

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--outdir',type=Path,default=Path('/private/tmp/round3-xy'));ap.add_argument('--train',action='store_true');ap.add_argument('--pathgraph',type=Path,default=R/'context/таллинская - щукинская - Pathgraph для задачи.json');args=ap.parse_args()
 O=args.outdir;O.mkdir(parents=True,exist_ok=True);proj=g.build_projection(O,'c++')
 assets=R/'ros2_ws/src/tram_odometry/assets';official=np.array([[float(p[k]) for k in ['x','y','z']] for p in json.loads(args.pathgraph.read_text())['points']]);os=np.r_[0,np.cumsum(np.linalg.norm(np.diff(official[:,:2],axis=0),axis=1))]
 maps={};report={'protocol':'No truth or held-out bag read; map base_link positions use fixed physical heading lookahead 6.098m, paired GNSS proxy TF 9.873m, no registration/position/time fitting.', 'official':{'n':len(official),'length_xy_m':float(os[-1])},'maps':{}}
 for name in ['route_map.csv','route_map_branch_a.csv']:
  route=RouteMap.from_csv(assets/name);new=[]
  for direction in ['out','return']:
   a=np.array(route.rows[direction]);base=g.project([route.master_to_base_enu(direction,s) for s in a[:,0]],'E',proj);q,dist,j,f=project(base,official);along=os[j]+f*(os[j+1]-os[j]);od=official[j+1,:2]-official[j,:2];md=np.gradient(base[:,:2],axis=0);cos=np.sum(od*md,axis=1)/np.linalg.norm(od,axis=1)/np.linalg.norm(md,axis=1)
   covered=(along>50)&(along<os[-1]-50)&(dist<8)
   report['maps'][name+':'+direction]={'nodes':len(a),'distance_all':stats(dist),'covered_nodes':int(covered.sum()),'covered_s_m':[float(a[covered,0].min()),float(a[covered,0].max())],'covered_distance':stats(dist[covered]),'official_tangent_cos_median':float(np.median(cos[covered]))}
   maps[name+':'+direction]=(a,base)
 np.savez(O/'geometry.npz',official=official,**{k.replace(':','_'):v[1] for k,v in maps.items()})
 if args.train:
  report['train']=[];manifest=json.loads((R/'tools/split_manifest.json').read_text())
  for bag in manifest['representatives']['train']:
   inputs=g.read_permitted(R/'dataset/data'/bag)
   try:t,proxy=g.scoring_proxy(inputs,proj)
   except ValueError as e:report['train'].append({'bag':bag,'skip':str(e)});continue
   # Retain one in ten proxy samples to avoid dense temporal weighting.
   t=t[::10];proxy=proxy[::10];delta=np.gradient(proxy[:,:2],axis=0);moving=np.linalg.norm(delta,axis=1)>.5;heading=delta/np.maximum(np.linalg.norm(delta,axis=1),1e-9)[:,None]
   best=np.full(len(proxy),np.inf);directions=np.full(len(proxy),'',dtype='U6')
   for key,(a,base) in maps.items():
    q,d,j,f=project(proxy,base);tan=base[j+1,:2]-base[j,:2];align=np.sum(tan*heading,axis=1)/np.linalg.norm(tan,axis=1);good=(align>.8)&(d<best);best[good]=d[good];directions[good]=key.split(':')[1]
   q,d,j,f=project(proxy,official);along=os[j]+f*(os[j+1]-os[j]);covered=(along>50)&(along<os[-1]-50)&moving&(best<5)&(d<8)
   row={'bag':bag,'proxy_samples':len(t),'moving_samples':int(moving.sum()),'directions':{}}
   for direction in ['out','return']:
    m=covered&(directions==direction);row['directions'][direction]={'n':int(m.sum()),'official_xy':stats(d[m]),'existing_base_xy':stats(best[m])}
   report['train'].append(row);print(bag,row['directions'],flush=True)
   np.savez(O/(bag+'_proxy.npz'),t=t,proxy=proxy,moving=moving,directions=directions,covered=covered)
 (O/'geometry_report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report['maps'],indent=2))
if __name__=='__main__':main()
