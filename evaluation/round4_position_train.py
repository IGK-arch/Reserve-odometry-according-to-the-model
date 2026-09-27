#!/usr/bin/env python3
"""Train-only immutable-binary study of position correction variants.

Only manifest train bags are opened. Downsampled offline paired-GNSS proxy is
reused from round3 (one in ten source proxy samples). Output nearest proxy stamp
within50ms; identical masks/stamps/speed/distance required. Never reads public.
"""
from pathlib import Path
import json,csv,sys,subprocess,hashlib
import numpy as np
from round4_position_build import S,O,VARIANTS,sha
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'evaluation'));import navigation_gnss_stress as g
ASSETS=S/'ros2_ws/src/tram_odometry/assets';BASE=Path('/private/tmp/odometry-round4-baseline/navigation_replay');CACHE=Path('/private/tmp/gnss-round2-baseline')
def metrics(errors):
 if not len(errors):return {'n':0}
 return {'n':len(errors),'rmse_m':float(np.sqrt(np.mean(errors*errors))),'p95_m':float(np.quantile(errors,.95)),'max_m':float(np.max(errors)),'last_m':float(errors[-1])}
def replay(exe,inp,out):
 subprocess.run([str(exe),'--input',str(inp),'--output',str(out),'--vehicle',inp.name[:5],'--map',str(ASSETS/'route_map.csv'),'--alternate-map',str(ASSETS/'route_map_branch_a.csv'),'--elevation',str(ASSETS/'official_elevation.csv'),'--drive-table',str(ASSETS/'drive_accel_table.csv'),'--gnss-mode','corrections'],check=True)
 with out.open() as f:rows=list(csv.DictReader(f))
 t=np.array([int(r['stamp_ns']) for r in rows]);p=np.array([[float(r[k]) for k in ['x','y','z']] for r in rows]);state=np.array([[float(r[k]) for k in ['velocity_mps','distance_m']] for r in rows]);valid=np.array([r['position_valid']=='1' and r['frame_id']=='mgrs_37UCB' for r in rows]);return t,p,state,valid,int(rows[-1]['gnss_corrections']),int(rows[-1]['gnss_rejected'])
def aggregate(rows):
 result={}
 for session in ['all']+sorted({r['session'] for r in rows}):
  result[session]={}
  for mode in g.MODES:
   selected=[r['modes'][mode] for r in rows if session=='all' or r['session']==session];n=sum(x['baseline']['xy']['n'] for x in selected);result[session][mode]={'n':n,'bags':len(selected)}
   for name in ['baseline',*VARIANTS]:
    valid=[x for x in selected if x[name]['xy']['n']];rm=[x[name]['xy']['rmse_m'] for x in valid];br=[x['baseline']['xy']['rmse_m'] for x in valid]
    result[session][mode][name]={'pooled_xy_rmse_m':float(np.sqrt(sum(x[name]['xy']['n']*x[name]['xy']['rmse_m']**2 for x in valid)/n)) if n else None,'median_bag_xy_rmse_m':float(np.median(rm)) if rm else None,'p90_bag_xy_rmse_m':float(np.quantile(rm,.9)) if rm else None,'max_bag_xy_rmse_m':max(rm) if rm else None,'worse_bags_gt_0_1m':sum(a>b+.1 for a,b in zip(rm,br)),'better_bags_gt_0_1m':sum(a<b-.1 for a,b in zip(rm,br)),'worst_bag_delta_rmse_m':max([a-b for a,b in zip(rm,br)],default=0),'best_bag_delta_rmse_m':min([a-b for a,b in zip(rm,br)],default=0)}
 return result

def main():
 D=O/'train';D.mkdir(exist_ok=True);manifest=json.loads((R/'tools/split_manifest.json').read_text());groups={b:g for g in manifest['groups'] for b in g['bags']};bags=[b for b in manifest['representatives']['train'] if b.startswith('30618_')]
 report={'protocol':__doc__,'baseline_binary_sha256':sha(BASE),'candidate_build':json.loads((O/'build.json').read_text()),'train_bags':len(bags),'bags':[],'no_public_or_validation_read':True}
 for bag in bags:
  z=np.load(Path('/private/tmp/round3-xy')/(bag+'_proxy.npz'));row={'bag':bag,'session':groups[bag]['date'],'proxy_count':len(z['t']),'proxy_sha256':sha(Path('/private/tmp/round3-xy')/(bag+'_proxy.npz')),'modes':{}}
  for mode in g.MODES:
   inp=CACHE/(bag+'_'+mode+'_input.csv')
   if not inp.exists():
    inp=D/(bag+'_'+mode+'_input.csv');inputs=g.read_permitted(R/'dataset/data'/bag)
    with inp.open('w') as f:csv.writer(f).writerows(g.select_inputs(inputs,mode))
   original=None;mr={'input_sha256':sha(inp)}
   for name,exe in [('baseline',BASE)]+[(n,O/n/'navigation_replay') for n in VARIANTS]:
    out=D/(name+'_scratch.csv');a=replay(exe,inp,out);t,p,state,valid,corrections,rejected=a;ix,age=g.nearest(t,z['t']);m=(age<=50e6)&valid[ix]
    if original is None:original=a;mask=m
    else:
     assert np.array_equal(t,original[0]) and np.array_equal(state,original[2]),(bag,mode,name,'vehicle changed')
     assert np.array_equal(valid,original[3]) and np.array_equal(m,mask),(bag,mode,name,'coverage changed')
    err=p[ix[m]]-z['proxy'][m];mr[name]={'xy':metrics(np.linalg.norm(err[:,:2],axis=1)),'xyz':metrics(np.linalg.norm(err,axis=1)),'proxy_count':len(ix),'matched_count':int((age<=50e6).sum()),'absolute_output_count':int(valid.sum()),'outputs':len(t),'same_velocity_distance_stamps_mask':True,'max_position_delta_m':float(np.linalg.norm(p-original[1],axis=1).max()),'corrections':corrections,'rejected':rejected,'output_sha256':sha(out),'common_stamp_mask_sha256':hashlib.sha256(z['t'][m].tobytes()).hexdigest()}
   row['modes'][mode]=mr
  report['bags'].append(row);report['summary']=aggregate(report['bags']);(O/'train_report.json').write_text(json.dumps(report,indent=2)+'\n');print(bag,{m:{k:round(row['modes'][m][k]['xy'].get('rmse_m',0),3) for k in ['baseline',*VARIANTS]} for m in g.MODES},flush=True)
if __name__=='__main__':main()
