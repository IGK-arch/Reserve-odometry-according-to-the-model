"""All 97 unique C/F/R replays: preserve timestamp/count checks and per-field deltas."""
import sys,json,hashlib,subprocess,io,argparse
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser();parser.add_argument('--before',type=Path,required=True);parser.add_argument('--after',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'evaluation'))
from causal_baseline import read_run,CMD,FRONT,REAR
before=args.before.resolve();after=args.after.resolve();table=root/'ros2_ws/src/tram_odometry/assets/drive_accel_table.csv'
manifest=json.loads((root/'tools/split_manifest.json').read_text()); rows=[]
for split in ['train','validation','holdout']:
 for bag in manifest['representatives'][split]:
  events,_=read_run(bag);codes={CMD:'C',FRONT:'F',REAR:'R'}
  payload=''.join(f'{a},{b},{codes[c]},{d:.17g}\n' for a,b,c,d in events).encode(); outputs=[]
  for exe in [before,after]:
   command=[str(exe),bag[:5]]+([str(table)] if bag.startswith('30618') else [])
   outputs.append(subprocess.run(command,input=payload,stdout=subprocess.PIPE,check=True).stdout)
  before_rows=outputs[0].splitlines()[1:];after_rows=outputs[1].splitlines()[1:]
  same_stamps=len(before_rows)==len(after_rows) and all(a.split(b',',2)[:2]==b.split(b',',2)[:2] for a,b in zip(before_rows,after_rows))
  assert same_stamps,bag
  a,b=[np.loadtxt(io.BytesIO(v),delimiter=',',skiprows=1,usecols=range(2,12),ndmin=2) for v in outputs]
  row={'bag':bag,'outputs':len(a),'same_timestamps':same_stamps,'byte_equal':outputs[0]==outputs[1], 'max_abs_delta_by_field':dict(zip(outputs[0].splitlines()[0].decode().split(',')[2:],np.max(np.abs(a-b),axis=0).tolist()))}
  rows.append(row);print(bag,row['max_abs_delta_by_field']['velocity_mps'],row['max_abs_delta_by_field']['distance_m'],flush=True)
report={'before':str(before),'after':str(after),'binary_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (before,after)},'bags':len(rows),'rows':rows,'max_abs_delta_by_field':{k:max(r['max_abs_delta_by_field'][k] for r in rows) for k in rows[0]['max_abs_delta_by_field']}}
args.out.write_text(json.dumps(report,indent=2))
print(json.dumps(report['max_abs_delta_by_field'],indent=2),flush=True)
