#!/usr/bin/env python3
"""Independent analytic fixtures for round4 filter research; no runtime truth input."""
import hashlib,json,math,random
from pathlib import Path
import wheel_fault_benchmark as w
from round4_kalman import VARIANTS
ROOT=Path(__file__).resolve().parents[1]

def truth(t,profile):
 if profile=='constant_accel':return 2+.3*t,2*t+.15*t*t,4
 if profile=='command_step':
  u=max(0,t-5);return 5+.5*u,5*t+.25*u*u,0 if t<5 else 4
 if profile=='stop':
  u=min(t,10);return max(0,5-.5*t),5*u-.25*u*u,-5 if t<10 else 0
 raise ValueError(profile)

def main():
 rows=[];scratch=Path('/private/tmp/round4-kalman');baseline=Path('/private/tmp/odometry-round4-baseline/replay_cli')
 for vehicle in [30618,30639]:
  scales=(1.000295,1.000195) if vehicle==30618 else (1.003512,1.003187)
  table=ROOT/'ros2_ws/src/tram_odometry/assets/drive_accel_table.csv' if vehicle==30618 else None
  for profile in ['constant_accel','command_step','stop']:
   for dropout in [False,True]:
    rng=random.Random(41);events=[]
    for i in range(401):
     t=i*.05;s=w.nanoseconds(t);events.append((s+2000000,s,'C',truth(t,profile)[2]))
     if i%2 or (dropout and 10<=t<15):continue
     for channel,scale in zip('FR',scales):events.append((s+40000000,s,channel,max(0,truth(t,profile)[0]+rng.gauss(0,.008))*3.6/scale))
    payload=w.serialize_events(sorted(events));expected=[w.nanoseconds(i*.05) for i in range(40,401)]
    for name in ['baseline',*VARIANTS]:
     exe=baseline if name=='baseline' else scratch/name/'replay_cli';out=w.replay(exe,payload,vehicle,table)
     valid=[s for s in expected if s in out and all(math.isfinite(out[s][k]) for k in ['v','s'])];errors=[out[s]['v']-truth(w.seconds(s),profile)[0] for s in valid]
     assert len(valid)==len(expected),(profile,name,'coverage')
     assert all(0<=out[s]['v']<=25 for s in valid),(profile,name,'bounds')
     rows.append({'vehicle':vehicle,'profile':profile,'dropout_10_15s':dropout,'variant':name,'input_sha256':hashlib.sha256(payload.encode()).hexdigest(),'n':len(valid),'speed_rmse_mps':w.error_metrics(errors)['rmse'],'final_speed_mps':out[valid[-1]]['v'],'end_distance_error_m':out[valid[-1]]['s']-truth(20,profile)[1]})
 report={'protocol':__doc__,'results':rows};outdir=ROOT/'evaluation/results/round4';outdir.mkdir(parents=True,exist_ok=True);(outdir/'kalman_fixtures.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(rows,indent=2))
if __name__=='__main__':main()
