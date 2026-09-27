#!/usr/bin/env python3
"""Post-freeze validation/holdout report; never changes the selected estimator."""
from pathlib import Path
import json,math,hashlib,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[1];SCRATCH=Path('/private/tmp/round4-kalman/long_guarded');OUT=ROOT/'evaluation/results/round4'
FROZEN_SHA='b37a73245bfc141b2975f4b65482ee6c736cd19e44ebdecf8c5e581e4da2c143'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rmse(rows,key):return math.sqrt(sum(r[key]**2 for r in rows)/len(rows)) if rows else None

def summarize(d,vehicle=None):
 out={}
 for v in ['baseline','candidate']:
  rows=[r for r in d['nominal'] if r['variant']==v and (vehicle is None or r['vehicle']==vehicle)];n=sum(r['n'] for r in rows)
  out[v]={'bags':len(rows),'scored_bags':sum(r['n']>0 for r in rows),'n':n,'nominal_rmse_mps':math.sqrt(sum(r['n']*r['rmse_mps']**2 for r in rows if r['n'])/n) if n else None,'blackouts':{}}
  for h in [1,3,5]:
   f=[r for r in d['blackouts'] if r['variant']==v and r['horizon_s']==h and (vehicle is None or r['vehicle']==vehicle)]
   out[v]['blackouts'][h]={'n':len(f),'speed_rmse_mps':rmse(f,'speed_error_mps'),'distance_rmse_m':rmse(f,'distance_error_m')}
   for metric in ['speed_error_mps','distance_error_m']:
    values=[abs(r[metric]) for r in f]
    out[v]['blackouts'][h][metric+'_abs_p95']=float(np.percentile(values,95)) if values else None
    out[v]['blackouts'][h][metric+'_abs_max']=max(values) if values else None
 return out

def main():
 assert sha(SCRATCH/'replay_cli')==FROZEN_SHA
 for split in ['validation','holdout']:
  d=json.loads((SCRATCH/split/'results.json').read_text());sessions=json.loads((SCRATCH/split/'summary.json').read_text());assert d['provenance']['candidate']['binary']==FROZEN_SHA
  b={r['bag']:r for r in d['nominal'] if r['variant']=='baseline'};a={r['bag']:r for r in d['nominal'] if r['variant']=='candidate'}
  tails=[{'bag':k,'vehicle':b[k]['vehicle'],'date':b[k]['date'],'baseline':b[k]['rmse_mps'],'candidate':a[k]['rmse_mps'],'delta':a[k]['rmse_mps']-b[k]['rmse_mps'],'ratio':a[k]['rmse_mps']/b[k]['rmse_mps'] if b[k]['rmse_mps'] else None} for k in b if b[k]['n']]
  tails.sort(key=lambda r:r['delta'],reverse=True)
  key=lambda r:(r['bag'],r['start_s'],r['horizon_s'])
  fb={key(r):r for r in d['blackouts'] if r['variant']=='baseline'};fa={key(r):r for r in d['blackouts'] if r['variant']=='candidate'};assert fb.keys()==fa.keys()
  fault_tails={}
  for h in [1,3,5]:
   fault_tails[h]={}
   for metric in ['speed_error_mps','distance_error_m']:
    items=[{'bag':k[0],'vehicle':fb[k]['vehicle'],'date':fb[k]['date'],'start_s':k[1],'baseline_error':fb[k][metric],'candidate_error':fa[k][metric],'absolute_error_delta':abs(fa[k][metric])-abs(fb[k][metric])} for k in fb if k[2]==h]
    fault_tails[h][metric]=sorted(items,key=lambda r:r['absolute_error_delta'],reverse=True)[:10]
  report={'protocol':__doc__,'split':split,'selected_variant':'long_guarded','candidate_frozen_sha256':FROZEN_SHA,'pooled':summarize(d),'vehicles':{v:summarize(d,v) for v in sorted({r['vehicle'] for r in d['nominal']})},'sessions':sessions,'nominal_bag_tails':tails,'blackout_window_tails':fault_tails,'coverage':{'identical_masks_and_output_counts':all(all(b[k][f]==a[k][f] for f in ['n','outputs','common_outputs','mask_sha256']) for k in b),'unscored_bags':[k for k in b if not b[k]['n']],'identical_blackout_endpoints':True},'provenance':d['provenance']}
  report['provenance']['additional_source_hashes']={s:sha(ROOT/s) for s in ['evaluation/round4_kalman.py','evaluation/round4_kalman_generalization.py','evaluation/causal_baseline.py','evaluation/core_benchmark.py','evaluation/table_blackout_ablation.py','evaluation/calibrate_drive.py']}
  report['provenance']['frozen_candidate_sources']={s:sha(SCRATCH/s) for s in ['estimator.cpp','include/tram_odometry/estimator.hpp','candidate.patch']}
  for name in ['results','summary']:shutil.copyfile(SCRATCH/split/(name+'.json'),OUT/f'kalman_{split}_{name}.json')
  (OUT/f'kalman_{split}_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
  print(split,json.dumps(report['pooled']));print('vehicles',json.dumps(report['vehicles']));print('worst bags',json.dumps(tails[:4]));print('coverage',report['coverage'])
if __name__=='__main__':main()
