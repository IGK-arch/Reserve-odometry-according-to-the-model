#!/usr/bin/env python3
"""Known straight-track synthetic position bias and GNSS fault study."""
from pathlib import Path
import numpy as np,json,csv,subprocess,sys
from round4_position_build import S,O,VARIANTS,sha
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'evaluation'));import navigation_gnss_stress as g

def metrics(x):return {'n':len(x),'rmse':float(np.sqrt(np.mean(x*x))),'p95':float(np.quantile(x,.95)),'max':float(np.max(x)),'last':float(x[-1])}
def main():
 D=O/'synthetic';D.mkdir(exist_ok=True);projection=g.build_projection(D,'c++')
 source=D/'inverse.cpp';source.write_text('#include <iostream>\n#include <iomanip>\n#include "tram_odometry/geo_projection.hpp"\nint main(){tram_odometry::geo::EnuDatum d({55.810367065,37.462266845,168.3794});double x,y,z;std::cout<<std::setprecision(17);while(std::cin>>x>>y>>z){auto p=d.fromEnu({x,y,z});std::cout<<p.latitude_deg<<" "<<p.longitude_deg<<" "<<p.height_m<<"\\n";}}')
 subprocess.run(['c++','-std=c++17','-O2','-I'+str(S/'ros2_ws/src/tram_odometry/include'),str(source),'-o',str(D/'inverse')],check=True)
 (D/'map.csv').write_text('direction,s,x,y,z\nout,0,0,0,3\nout,10000,10000,0,3\n')
 t=np.arange(0,361,.1);speed=np.minimum(np.maximum(t-2,0),10);speed[t>=260]=np.maximum(0,10-(t[t>=260]-260));distance=np.r_[0,np.cumsum((speed[:-1]+speed[1:])*.05)]+100
 report={'cases':{},'protocol':'Known straight ENU trajectory,10m/s cruise,1% wheel scale, sparseGNSS, stationary after270s; unchanged vehicle inputs pervariant.'}
 for case in ['scale_sparse','scale_bad_shared','scale_frozen','scale_delayed','healthy_sparse']:
  positions=[];fixmeta=[]
  for i,tm in enumerate(t):
   window=int((tm-120)//120);phase=(tm-120)%120
   if tm>30 and not(window>=0 and phase<2):continue
   # First startup window both antennas, subsequent two-second windows one antenna.
   antennas=[0,1] if tm<=30 else [window%2]
   for rover in antennas:
    x=distance[i]+rover*12.436
    if tm>30 and case=='scale_bad_shared':x+=20
    if tm>30 and case=='scale_frozen':x=float(np.interp(120*(window+1),t,distance))+rover*12.436
    positions.append((x,0,3));fixmeta.append((i,rover))
  inv=subprocess.run([str(D/'inverse')],input=''.join(f'{x} {y} {z}\n' for x,y,z in positions),text=True,capture_output=True,check=True);llh=np.loadtxt(inv.stdout.splitlines());rows=[]
  for i,tm in enumerate(t):
   ns=int(round((100+tm)*1e9));wheel=speed[i]*(1 if case=='healthy_sparse' else 1.01)*3.6
   rows.extend([(ns,ns,'F',wheel),(ns,ns,'R',wheel),(ns,ns,'C',0)])
  for (i,rover),p in zip(fixmeta,llh):
   ns=int(round((100+t[i])*1e9));receive=ns+(1000000000 if t[i]>30 and case=='scale_delayed' else 0);rows.append((receive,ns,'RF' if rover else 'MF',*p,2))
  rows.sort(key=lambda r:r[0]);inp=D/(case+'.csv')
  with inp.open('w') as f:csv.writer(f).writerows(rows)
  report['cases'][case]={};before=None
  for name,exe in [('baseline',Path('/private/tmp/odometry-round4-baseline/navigation_replay'))]+[(n,O/n/'navigation_replay') for n in VARIANTS]:
   out=D/(case+'_'+name+'.csv');subprocess.run([str(exe),'--input',str(inp),'--output',str(out),'--map',str(D/'map.csv'),'--table','0','--direction','out'],check=True)
   with out.open() as f:a=list(csv.DictReader(f))
   ts=np.array([int(r['stamp_ns']) for r in a]);xy=np.array([[float(r[k]) for k in ['x','y']] for r in a]);state=np.array([[float(r[k]) for k in ['velocity_mps','distance_m']] for r in a]);elapsed=(ts-ts[0])*1e-9;truthx=np.interp(elapsed,t,distance)+9.873;truth=g.project(np.c_[truthx,np.zeros(len(ts)),np.zeros(len(ts))],'E',projection);valid=np.array([r['position_valid']=='1' and r['frame_id']=='mgrs_37UCB' for r in a]);mask=valid&(elapsed>=30);err=np.linalg.norm(xy-truth[:,:2],axis=1)
   if before is None:before=(ts,state)
   assert np.array_equal(ts,before[0]) and np.array_equal(state,before[1])
   report['cases'][case][name]={'moving_and_stop_after30s':metrics(err[mask]),'stopped_after280s':metrics(err[valid&(elapsed>=280)]),'stopped_position_range_m':float(np.linalg.norm(np.ptp(xy[elapsed>=280],axis=0))),'stopped_blackout_280_350_position_range_m':float(np.linalg.norm(np.ptp(xy[(elapsed>=280)&(elapsed<350)],axis=0))),'corrections':int(a[-1]['gnss_corrections']),'rejected':int(a[-1]['gnss_rejected']),'same_velocity_distance_stamps':True,'output_sha256':sha(out)}
  print(case,{n:r['moving_and_stop_after30s']['rmse'] for n,r in report['cases'][case].items()},flush=True)
 (O/'synthetic_report.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()
