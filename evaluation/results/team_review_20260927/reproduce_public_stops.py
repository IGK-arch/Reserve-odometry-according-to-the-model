"""Paired offline stop-landmark check on the open reference bag. No fitting."""
from pathlib import Path
import argparse,csv,hashlib,json,subprocess,sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'evaluation'))
import reference_benchmark as bench
p=argparse.ArgumentParser()
p.add_argument('--bag',type=Path,required=True)
p.add_argument('--exe',type=Path,required=True)
a=p.parse_args()
assets=ROOT/'ros2_ws/src/tram_odometry/assets'
work=ROOT/'evaluation/runs/team_review_public_stops';work.mkdir(parents=True,exist_ok=True)
events,truth,inputs=bench.read_bag(a.bag)
t0=min(x[0] for x in inputs)
report={'method':'Paired offline nearest reference <=50ms, original stamps, no alignment. Current Navigation with stop landmarks disabled/enabled. Not ROS ATS.','reference':str(a.bag),'runtime_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'profiles':{}}
for mode,seconds in [('all_available',None),('first_1_5s',1.5),('first_30s',30.)]:
    selected=inputs if seconds is None else [x for x in inputs if x[2] in ('C','F','R') or x[1]<=t0+round(seconds*1e9)]
    src=work/(mode+'_input.csv')
    with src.open('w',newline='') as f:csv.writer(f).writerows(selected)
    profiles={};states=[]
    for name,enabled in [('without_stops',False),('with_stops',True)]:
        dst=work/(mode+'_'+name+'.csv')
        cmd=[str(a.exe.resolve()),'--input',str(src),'--output',str(dst),'--vehicle','30618','--map',str(assets/'route_map.csv'),'--alternate-map',str(assets/'route_map_branch_a.csv'),'--elevation',str(assets/'official_elevation.csv'),'--drive-table',str(assets/'drive_accel_table.csv'),'--gnss-mode','corrections']
        if enabled:cmd+=['--stops',str(assets/'stops.csv')]
        subprocess.run(cmd,check=True)
        rows=list(csv.DictReader(dst.open()))
        states.append([(r['stamp_ns'],r['velocity_mps'],r['distance_m'],r['position_valid']) for r in rows])
        metrics=bench.evaluate(bench.read_candidate(dst),truth,[],50_000_000)
        profiles[name]={'candidate':metrics['candidate'],'stop_corrections':int(rows[-1]['stop_corrections']),'command':cmd}
    assert states[0]==states[1]
    report['profiles'][mode]={'input_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'speed_distance_and_stamps_identical':True,**profiles}
report['hashes']={str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [a.exe.resolve(),Path(__file__).resolve(),ROOT/'evaluation/reference_benchmark.py',ROOT/'ros2_ws/src/tram_odometry/src/navigation.cpp',ROOT/'ros2_ws/src/tram_odometry/src/estimator.cpp',ROOT/'ros2_ws/src/tram_odometry/config/default.yaml',*assets.glob('*.csv')]}
out=Path(__file__).with_name('public_stop_comparison.json');out.write_text(json.dumps(report,indent=2)+'\n')
for mode,r in report['profiles'].items():
    print(mode,[(key,r[key]['candidate']['position']['rmse_3d_m'],r[key]['candidate']['velocity']['rmse_mps'],r[key]['stop_corrections']) for key in ['without_stops','with_stops']],flush=True)
