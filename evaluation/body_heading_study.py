#!/usr/bin/env python3
"""Offline body-heading geometry diagnostic on paired GNSS only.

The 6.098 m shift is fixed from organizer dimensions (9.873 - 7.55/2), not
optimized. Zero,3,and9m are sensitivity ablations. Oracle nearest-route matching
and rover interpolation use offline GNSS solely to isolate heading error;
these are NOT full odometry accuracy or contest scores. Geometry maps remain
train-only. The production algorithm never receives these interpolated points.
Uses both branch maps and directions, rejects pairs outside physical baseline
range, strong coordinate outliers, low speed and >2m route mismatch. Reports
lever-arm error caused only by heading, not absolute position error.
"""
from pathlib import Path
import sys,json,csv
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import median_filter
R=Path(__file__).resolve().parents[1]
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--split',choices=('train','validation','holdout'),default='train')
parser.add_argument('--outdir',type=Path,required=True)
args=parser.parse_args()
sys.path.insert(0,str(R/'evaluation'))
import navigation_gnss_stress as g
O=args.outdir.resolve();O.mkdir(parents=True,exist_ok=False)
p=g.build_projection(O,'c++')
manifest=json.loads((R/'tools/split_manifest.json').read_text())
routes=[]
for filename in ['route_map.csv','route_map_branch_a.csv']:
 rows=list(csv.DictReader((R/'ros2_ws/src/tram_odometry/assets'/filename).open()))
 for direction in ['out','return']:
  a=np.array([[float(r[k]) for k in ['s','x','y','z']] for r in rows if r['direction']==direction])
  routes.append((filename,direction,a,cKDTree(a[:,1:3])))
results=[]
for bag in manifest['representatives'][args.split]:
 if not bag.startswith('30618'):continue
 rows=g.read_permitted(R/'dataset/data'/bag)
 fs={}
 for code in ('MF','RF'):
  a=np.array([[r[1],*r[3:6]] for r in rows if r[2]==code and r[6]>=0],dtype=float)
  if len(a)<2:break
  a=a[np.argsort(a[:,0])];a=a[np.r_[True,np.diff(a[:,0])>0]]
  fs[code]=(a[:,0],g.project(a[:,1:],'G',p))
 if len(fs)!=2:continue
 tm,pm=fs['MF'];tr,pr=fs['RF']
 idx=np.clip(np.searchsorted(tr,tm),1,len(tr)-1)
 age=np.minimum(np.abs(tr[idx]-tm),np.abs(tr[idx-1]-tm))
 pair=np.array([np.interp(tm-tr[0],tr-tr[0],pr[:,j]) for j in range(3)]).T
 delta=pair-pm;dist=np.linalg.norm(delta[:,:2],axis=1)
 speed=np.linalg.norm(pm[10:,:2]-pm[:-10,:2],axis=1)/(np.maximum(tm[10:]-tm[:-10],1)*1e-9)
 moving=np.r_[np.zeros(10),speed]>1
 good=(age<80e6)&(dist>11.5)&(dist<13.4)&moving&(np.linalg.norm(pm-median_filter(pm,size=(9,1),mode='nearest'),axis=1)<.5)
 indices=np.flatnonzero(good)[::10]
 sums={str(q):[] for q in [0,3,6.098,9]};curves={str(q):[] for q in [0,3,6.098,9]}
 for i in indices:
  true=delta[i,:2]/dist[i];best=None
  for filename,direction,a,tree in routes:
   ds,ix=tree.query(pm[i,:2],k=3)
   for j in {max(0,min(len(a)-2,int(k))) for k in list(ix)+list(ix-1)}:
    d=a[j+1,1:3]-a[j,1:3];n=d@d
    if n<1e-8:continue
    f=np.clip((pm[i,:2]-a[j,1:3])@d/n,0,1)
    err=np.linalg.norm(pm[i,:2]-(a[j,1:3]+f*d))
    if d@true<=0:continue
    if best is None or err<best[0]:best=(err,a,a[j,0]+f*(a[j+1,0]-a[j,0]))
  if best is None or best[0]>2:continue
  err,a,s=best
  def tangent(s):
   s=np.clip(s,a[0,0],a[-1,0])
   u=np.array([np.interp(s+1,a[:,0],a[:,j])-np.interp(s-1,a[:,0],a[:,j]) for j in (1,2)])
   return u/np.linalg.norm(u)
  c=abs(np.linalg.det(np.stack((tangent(s-6),tangent(s+6)))))>.05
  for q in sums:
   e=9.873*np.linalg.norm(tangent(s+float(q))-true)
   sums[q].append(float(e))
   if c:curves[q].append(float(e))
 def stats(es):return {'n':len(es),'rmse':float(np.sqrt(np.mean(np.square(es)))),'p95':float(np.quantile(es,.95))} if len(es) else None
 r={'bag':bag,'heading_only_base_offset_error':{k:stats(v) for k,v in sums.items()},'curves':{k:stats(v) for k,v in curves.items()}}
 results.append(r);print(bag,r['curves'],flush=True)
 (O/(args.split+'.json')).write_text(json.dumps(results,indent=2))
