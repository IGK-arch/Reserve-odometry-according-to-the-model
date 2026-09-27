#!/usr/bin/env python3
"""Train-only 120 s startup study. Diagnostic paired GNSS proxy shares sensors."""
from pathlib import Path
import subprocess,json,csv,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=Path('/private/tmp/round4-startup');BASE=Path('/private/tmp/odometry-round4-source');ASSETS=BASE/'ros2_ws/src/tram_odometry/assets'
VERSIONS=['baseline','mixed','rigid','joint','pure']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def executable(v):return Path('/private/tmp/odometry-round4-baseline/navigation_replay') if v=='baseline' else OUT/v/'navigation_replay'
def read(p):
 r=list(csv.DictReader(p.open()));return np.array([int(x['stamp_ns']) for x in r]),np.array([[float(x[k]) for k in ['x','y','z']] for x in r]),np.array([x['position_valid']=='1' and x['frame_id']=='mgrs_37UCB' for x in r]),np.array([[float(x[k]) for k in ['velocity_mps','distance_m']] for x in r])
def nearest(t,q):
 i=np.searchsorted(t,q).clip(0,len(t)-1);p=(i-1).clip(0,len(t)-1);return np.where(abs(t[p]-q)<abs(t[i]-q),p,i)
def main():
 report={'protocol':__doc__,'selection_data':'Synthetic truth and all 42 train bags only; no validation, holdout or public reads. Inputs use cached sparse GNSS, dense in first30s. Scores one in ten clean paired proxy timestamps, nearest output <=50ms, common mask; XY only. No registration. Windows 0-30,30-60,60-120s.','versions':{v:sha(executable(v)) for v in VERSIONS},'bags':[]}
 manifest=json.loads((ROOT/'tools/split_manifest.json').read_text());groups={b:g for g in manifest['groups'] for b in g['bags']}
 for bag in manifest['representatives']['train']:
  source=Path('/private/tmp/round3-xy')/(bag+'_input.csv');inp=OUT/(bag+'_input.csv')
  rows=[];t0=None
  for r in csv.reader(source.open()):
   if t0 is None:t0=int(r[0])
   if int(r[0])-t0<=120e9:rows.append(r)
   else:break
  with inp.open('w') as f:csv.writer(f).writerows(rows)
  data={}
  for v in VERSIONS:
   p=OUT/(bag+'_'+v+'.csv');subprocess.run([str(executable(v)),'--input',str(inp),'--output',str(p),'--vehicle',bag.split('_')[0],'--map',str(ASSETS/'route_map.csv'),'--alternate-map',str(ASSETS/'route_map_branch_a.csv'),'--elevation',str(ASSETS/'official_elevation.csv'),'--drive-table',str(ASSETS/'drive_accel_table.csv'),'--gnss-mode','corrections'],check=True);data[v]=read(p)
  a=data['baseline'];z=np.load(Path('/private/tmp/round3-xy')/(bag+'_proxy.npz'));qt=z['t'];qp=z['proxy'];m=(qt>=a[0][0])&(qt<=a[0][-1]);qt=qt[m];qp=qp[m];ix=nearest(a[0],qt);valid=(abs(a[0][ix]-qt)<=50e6)
  row={'bag':bag,'session':groups[bag]['date'],'input_sha256':sha(inp),'proxy_sha256':sha(Path('/private/tmp/round3-xy')/(bag+'_proxy.npz')),'n_proxy':len(qt),'versions':{}}
  for v in VERSIONS:
   b=data[v];assert np.array_equal(a[0],b[0]);assert np.array_equal(a[3],b[3]);common=valid&a[2][ix]&b[2][ix];out={'common_n':int(common.sum()),'lost_absolute_matches':int((valid&a[2][ix]&~b[2][ix]).sum()),'gained_absolute_matches':int((valid&~a[2][ix]&b[2][ix]).sum()),'absolute_outputs':int(b[2].sum()),'max_shift_m':float(np.max(np.linalg.norm(b[1][:,:2]-a[1][:,:2],axis=1))),'windows':{},'same_velocity_distance':True}
   for lo,hi in [(0,30),(30,60),(60,120),(0,120)]:
    mask=common&((qt-a[0][0])/1e9>=lo)&((qt-a[0][0])/1e9<hi);errors=np.linalg.norm(b[1][ix[mask],:2]-qp[mask,:2],axis=1);out['windows'][f'{lo}-{hi}']={'n':len(errors),'rmse':float(np.sqrt(np.mean(errors**2))) if len(errors) else None,'max':float(np.max(errors)) if len(errors) else None}
   row['versions'][v]=out
  report['bags'].append(row);print(bag,' '.join(f'{v}:{row["versions"][v]["windows"]["0-120"]["rmse"]}' for v in VERSIONS),flush=True)
 report['summary']={}
 for v in VERSIONS:
  report['summary'][v]={}
  for w in ['0-30','30-60','60-120','0-120']:
   rr=[r['versions'][v]['windows'][w] for r in report['bags']];n=sum(r['n'] for r in rr);report['summary'][v][w]={'n':n,'rmse':(sum(r['n']*r['rmse']**2 for r in rr if r['n'])/n)**.5}
 (ROOT/'evaluation/results/round4/startup_train.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report['summary'],indent=2))
if __name__=='__main__':main()
