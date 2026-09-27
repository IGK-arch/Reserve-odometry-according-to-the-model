import pathlib,csv,json,math,sys,hashlib,collections
ROOT=pathlib.Path('/Users/infibiss/Desktop/Reserve-odometry-according-to-the-model');sys.path.insert(0,str(ROOT/'evaluation'))
from reference_benchmark import read_bag,read_candidate,ReferenceIndex,evaluate
report3=json.load(open(ROOT/'evaluation/runs/round3_reference/ablation.json'))
_,refs,_=read_bag(pathlib.Path(report3['provenance']['bag'])); index=ReferenceIndex(refs)
run=ROOT/'evaluation/runs/round3_public_delta_baseline';result={'protocol':'Post-freeze diagnosis only; no runtime/parameter selection. Same permitted input CSV, frozen50bd771 binary and round3v1 outputs; nearest-reference <=50ms, not ROS ATS.','profiles':{}}
def norm(v):return math.sqrt(sum(x*x for x in v))
for name in ('startup','corrections','branch','full'):
 b=read_candidate(run/(name+'.csv')); c=read_candidate(ROOT/'evaluation/runs/round3_reference'/(name+'.csv'))
 pairs=[]
 for x,y in zip(b,c):
  assert x.stamp_ns==y.stamp_ns
  ref=index.nearest(x.stamp_ns)
  if x.position is not None and y.position is not None and ref:
   e0=sum((a-z)**2 for a,z in zip(x.position,ref.position));e1=sum((a-z)**2 for a,z in zip(y.position,ref.position))
   pairs.append((x,y,ref,e0,e1))
 summary={'baseline_position_outputs':sum(s.position is not None for s in b),'candidate_position_outputs':sum(s.position is not None for s in c),'common_matched':len(pairs),'baseline_rmse_3d_m':math.sqrt(sum(p[3] for p in pairs)/len(pairs)),'candidate_rmse_3d_m':math.sqrt(sum(p[4] for p in pairs)/len(pairs)),'velocity_rows_changed':sum(x.velocity_mps!=y.velocity_mps for x,y in zip(b,c)),'distance_rows_changed':sum(x.distance_m!=y.distance_m for x,y in zip(b,c))}
 if name=='full':
  t0=b[0].stamp_ns
  def stats(rows):
   return {'matched':len(rows),'baseline_rmse_m':math.sqrt(sum(p[3] for p in rows)/len(rows)) if rows else None,'candidate_rmse_m':math.sqrt(sum(p[4] for p in rows)/len(rows)) if rows else None,'delta_squared_error_sum_m2':sum(p[4]-p[3] for p in rows),'max_output_separation_m':max((norm([v-w for v,w in zip(p[0].position,p[1].position)]) for p in rows),default=0)}
  summary['time_bins_from_first_vehicle_s']={str((lo,hi)):stats([p for p in pairs if lo<=(p[0].stamp_ns-t0)/1e9<hi]) for lo,hi in ((0,10),(10,30),(30,60),(60,120),(120,300),(300,600),(600,1400))}
  summary['distance_bins_m']={str((lo,hi)):stats([p for p in pairs if lo<=p[0].distance_m<hi]) for lo,hi in ((0,.01),(.01,10),(10,50),(50,100),(100,300),(300,10000))}
  deltas=[(norm([v-w for v,w in zip(x.position,y.position)]),(x.stamp_ns-t0)/1e9,x.distance_m) for x,y,_,_,_ in pairs]
  summary['last_difference_over_1cm']=max((v for v in deltas if v[0]>.01),key=lambda v:v[1]);summary['last_difference_over_1mm']=max((v for v in deltas if v[0]>.001),key=lambda v:v[1])
 result['profiles'][name]=summary
base=list(csv.DictReader(open(run/'diagnostic_baseline.csv')));candidate=list(csv.DictReader(open(run/'diagnostic_rtk.csv')))
b0=next(r for r in base if r['anchored']=='1');c0=next(r for r in candidate if r['anchored']=='1')
result['startup']={'baseline_first_anchor_output':b0,'candidate_first_anchor_output':c0,'delay_s':(int(c0['stamp_ns'])-int(b0['stamp_ns']))/1e9,'initial_yaw_delta_deg':math.degrees(float(c0['yaw'])-float(b0['yaw'])),'initial_position_separation_m':norm([float(c0[k])-float(b0[k]) for k in ('x','y','z')]),'same_reference_match_count':True}
L=9.873
lever=[L*(math.cos(float(c0['yaw']))-math.cos(float(b0['yaw']))),L*(math.sin(float(c0['yaw']))-math.sin(float(b0['yaw'])))];delta=[float(c0[k])-float(b0[k]) for k in ('x','y')]
result['startup']['horizontal_unit_slope_lever_arm_approximation']={'lever_delta_xy_m':lever,'lever_delta_norm_m':norm(lever),'remaining_inferred_master_anchor_delta_xy_m':[a-b for a,b in zip(delta,lever)],'note':'Descriptive approximate decomposition in projected XY, assumes horizontal body, not a counterfactual score or surveyed truth.'}
rows=list(csv.reader(open(ROOT/'evaluation/runs/round3_reference/permitted_inputs.csv')));t0=int(rows[0][1])
result['startup']['fix_status_counts_first_5s']={sensor:dict(collections.Counter(row[6] for row in rows if row[2]==sensor and int(row[1])<=t0+5_000_000_000)) for sensor in ('MF','RF')}
result['startup']['first_RTK_since_run_start_s']={sensor:next(((int(row[1])-t0)/1e9 for row in rows if row[2]==sensor and row[6]=='2'),None) for sensor in ('MF','RF')}
result['files_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [ROOT/'evaluation/runs/round3_reference/permitted_inputs.csv',*[run/(name+'.csv') for name in ('startup','corrections','branch','full')],run/'diagnostic_baseline.csv',run/'diagnostic_rtk.csv']}
out=ROOT/'evaluation/results/round3/public_startup_delta.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
