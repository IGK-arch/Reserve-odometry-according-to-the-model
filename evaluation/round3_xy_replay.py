#!/usr/bin/env python3
"""Isolated scratch-map XY ablation, using immutable 50bd771 navigation replay.

The candidate is intentionally restricted to the surveyed RETURN rail. Existing
outbound geometry and terminal loops stay intact. This evaluates a prototype;
it does not install the maps or select parameters from fused public truth.
"""
from pathlib import Path
import sys,json,csv,subprocess,argparse,hashlib
import numpy as np
from scipy.ndimage import gaussian_filter1d
from round3_official_xy import R,project,stats,g
EXE=Path('/private/tmp/odometry-pull-50bd771/navigation_replay')
ASSETS=R/'ros2_ws/src/tram_odometry/assets'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def candidate(O):
 geom=np.load(O/'geometry.npz');official=geom['official'];base=geom['route_map.csv_return'];q,d,j,f=project(base,official);os=np.r_[0,np.cumsum(np.linalg.norm(np.diff(official[:,:2],axis=0),axis=1))];s=os[j]+f*np.diff(os)[j]
 # Predetermined guards: no >0.5m move, 50m endpoint exclusion, 100m taper,
 # 10m Gaussian smoothing. Preserve metric s from corrected 3D geometry.
 weight=np.clip(np.minimum(s-50,os[-1]-s-50)/100,0,1)*(d<.5)
 delta=gaussian_filter1d((q-base[:,:2])*weight[:,None],5,axis=0,mode='nearest')
 delta[weight==0]=0
 pp=g.project([[0,0,0],[1,0,0],[0,1,0]],'E',O/'projection_cli');jac=(pp[1:,:2]-pp[0,:2]).T
 enudelta=delta@np.linalg.inv(jac).T
 checks={}
 for name in ['route_map.csv','route_map_branch_a.csv']:
  rows=list(csv.DictReader((ASSETS/name).open()));ret=np.array([[float(r[k]) for k in ['s','x','y','z']] for r in rows if r['direction']=='return']);new=ret.copy();new[:,1:3]+=enudelta;new[:,0]=np.r_[0,np.cumsum(np.linalg.norm(np.diff(new[:,1:],axis=0),axis=1))];ix=0
  with (O/name).open('w') as h:
   h.write('direction,s,x,y,z\n')
   for row in rows:
    if row['direction']=='return':h.write('return,'+','.join(f'{v:.9f}' for v in new[ix])+'\n');ix+=1
    else:h.write(','.join(row[k] for k in ['direction','s','x','y','z'])+'\n')
  checks[name]={'max_master_displacement_m':float(np.linalg.norm(enudelta,axis=1).max()),'s_length_change_m':float(new[-1,0]-ret[-1,0]),'unchanged_out':True,'unchanged_return_endpoint_xyz':bool(np.array_equal(new[[0,-1],1:],ret[[0,-1],1:])),'strictly_increasing_s':bool(np.all(np.diff(new[:,0])>0)),'max_step_change_m':float(np.max(np.linalg.norm(np.diff(new[:,1:3],axis=0)-np.diff(ret[:,1:3],axis=0),axis=1))),'sha256':sha(O/name)}
 return checks

def run(O,bag,inputs,version):
 inp=O/(bag+'_input.csv')
 if not inp.exists():
  with inp.open('w') as f:csv.writer(f).writerows(g.select_inputs(inputs,'sparse'))
 out=O/(bag+'_'+version+'.csv');maps=ASSETS if version=='baseline' else O
 subprocess.run([str(EXE),'--input',str(inp),'--output',str(out),'--vehicle',bag.split('_')[0],'--map',str(maps/'route_map.csv'),'--alternate-map',str(maps/'route_map_branch_a.csv'),'--elevation',str(ASSETS/'official_elevation.csv'),'--drive-table',str(ASSETS/'drive_accel_table.csv'),'--gnss-mode','corrections'],check=True)
 with out.open() as f:rows=list(csv.DictReader(f))
 t=np.array([int(r['stamp_ns']) for r in rows]);p=np.array([[float(r[k]) for k in ['x','y','z']] for r in rows]);valid=np.array([r['position_valid']=='1' and r['frame_id']=='mgrs_37UCB' for r in rows]);state=np.array([[float(r[k]) for k in ['velocity_mps','distance_m']] for r in rows]);return t,p,valid,state,sha(out)
def main():
 global EXE
 ap=argparse.ArgumentParser();ap.add_argument('--outdir',type=Path,default=Path('/private/tmp/round3-xy'));ap.add_argument('--baseline-exe',type=Path,default=EXE);args=ap.parse_args();O=args.outdir;EXE=args.baseline_exe.resolve()
 report={'decision_before_public':'Reject production XY change unless substantial train improvement; fixed bounded return-only scratch candidate, no fitted registration.', 'executable':str(EXE),'executable_sha256':sha(EXE),'candidate_checks':candidate(O),'protocol':'Sparse GNSS: both first30s, then alternating antenna2s every120s. Same immutable binary, inputs and output stamps. Score one in ten cleaned paired-GNSS proxy stamps using nearest output <=50ms, common absolute-valid mask, no alignment. 3D proxy altitude has unverified datum; XY is relevant.', 'bags':[]}
 manifest=json.loads((R/'tools/split_manifest.json').read_text())
 for bag in manifest['representatives']['train']:
  proxyfile=O/(bag+'_proxy.npz')
  if not proxyfile.exists():continue
  z=np.load(proxyfile);inputs=g.read_permitted(R/'dataset/data'/bag);a=run(O,bag,inputs,'baseline');b=run(O,bag,inputs,'candidate');assert np.array_equal(a[0],b[0]);assert np.array_equal(a[3],b[3]);ix,age=g.nearest(a[0],z['t']);m=(age<=50e6)&a[2][ix]&b[2][ix];row={'bag':bag,'proxy_count':len(ix),'n':int(m.sum()),'same_stamps_speed_distance':True,'same_absolute_mask':bool(np.array_equal(a[2],b[2])),'max_output_shift_m':float(np.linalg.norm(a[1]-b[1],axis=1).max()),'input_sha256':sha(O/(bag+'_input.csv'))}
  for key,out in [('baseline',a),('candidate',b)]:
   err=out[1][ix[m]]-z['proxy'][m];row[key]={'xy':stats(np.linalg.norm(err[:,:2],axis=1)),'xyz':stats(np.linalg.norm(err,axis=1)),'sha256':out[4]}
  report['bags'].append(row);(O/'replay_report.json').write_text(json.dumps(report,indent=2)+'\n');print(bag,row['n'],row['baseline']['xy'].get('rmse'),row['candidate']['xy'].get('rmse'),flush=True)
if __name__=='__main__':main()
