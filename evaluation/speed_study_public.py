import argparse,json,sys,math,hashlib
from pathlib import Path
parser=argparse.ArgumentParser(description='Post-freeze public speed diagnostic; original timestamps; no tuning.')
parser.add_argument('--root',required=True,type=Path)
parser.add_argument('--bag',required=True,type=Path)
parser.add_argument('--before',required=True,type=Path)
parser.add_argument('--after',required=True,type=Path)
parser.add_argument('--out',required=True,type=Path)
args=parser.parse_args()
root=args.root.resolve();sys.path.insert(0,str(root/'evaluation'))
from causal_baseline import read_run,make_reference,causal_wheel_baseline,ROOT
from core_benchmark import reference_at,replay_cpp
from reference_benchmark import read_bag,ReferenceIndex
bag=args.bag.resolve()
events,reference,_=read_bag(bag);ref=ReferenceIndex(reference)
# read_run also accepts absolute bag path; only its evaluator truth is retained.
_,truth=read_run(str(bag));gnss=make_reference(truth)
table=root/'ros2_ws/src/tram_odometry/assets/drive_accel_table.csv'
values={'wheel_mean':{r.stamp_ns:r.speed_mps for r in causal_wheel_baseline(events)}}
for name,exe in [('baseline',args.before.resolve()),('candidate',args.after.resolve())]:
 values[name]={s:v['velocity_mps'] for s,v in replay_cpp(exe,bag.name,events,table).items()}
common=sorted(set.intersection(*(set(v) for v in values.values())))
result={'bag':str(bag),'protocol':'post-freeze diagnostic; original timestamps, nearest fused signed-x reference within 50ms; common mask; no parameter choice','outputs':{k:len(v) for k,v in values.items()},'common_outputs':len(common),'mask_sha256':hashlib.sha256('\n'.join(map(str,common)).encode()).hexdigest(),'metrics':{}}
for truth_name,fn in [('fused',lambda s:None if ref.nearest(s) is None else ref.nearest(s).velocity_mps),('gnss',lambda s:reference_at(gnss,s))]:
 matches=[(s,v)for s in common if(v:=fn(s))is not None];result['metrics'][truth_name]={}
 for n,vs in values.items():
  errors=[vs[s]-v for s,v in matches]
  result['metrics'][truth_name][n]={'n':len(errors),'rmse_mps':math.sqrt(sum(e*e for e in errors)/len(errors)),'bias_mps':sum(errors)/len(errors)}
args.out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
