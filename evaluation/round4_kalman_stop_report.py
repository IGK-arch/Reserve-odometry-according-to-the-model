#!/usr/bin/env python3
"""Report the standstill-only structural correction without changing older research."""
from pathlib import Path
import json,hashlib,shutil,math
import numpy as np
from round4_kalman_generalization import summarize
ROOT=Path(__file__).resolve().parents[1];P=Path('/private/tmp/round4-kalman/long_guarded_stop');OUT=ROOT/'evaluation/results/round4'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 freeze=json.loads((P/'freeze.json').read_text());assert sha(P/'replay_cli')==freeze['candidate']['replay_cli']
 for name in ['freeze.json','candidate.patch','standstill.patch','existing_tests.json']:shutil.copyfile(P/name,OUT/('kalman_stop_'+name))
 for suffix in ['red','green','baseline']:shutil.copyfile(P.parent/f'standstill-{suffix}.log',OUT/f'kalman_stop_regression_{suffix}.log')
 report={'protocol':__doc__,'freeze':freeze,'tests':json.loads((P/'existing_tests.json').read_text()),'splits':{},'synthetic':{},'recommendation':'Retain legacy production. The structural standstill defect is fixed, but a longer-lived residual changes dropout tails; pooled improvement is not a universal fault guarantee.'}
 for split in ['train','validation','holdout']:
  d=json.loads((P/split/'results.json').read_text());assert d['provenance']['candidate']['binary']==freeze['candidate']['replay_cli'];summary=json.loads((P/split/'summary.json').read_text())
  b={r['bag']:r for r in d['nominal'] if r['variant']=='baseline'};a={r['bag']:r for r in d['nominal'] if r['variant']=='candidate'}
  tails=sorted([{'bag':k,'baseline_rmse_mps':b[k]['rmse_mps'],'candidate_rmse_mps':a[k]['rmse_mps'],'delta':a[k]['rmse_mps']-b[k]['rmse_mps']} for k in b if b[k]['n']],key=lambda r:r['delta'],reverse=True)
  key=lambda r:(r['bag'],r['start_s'],r['horizon_s']);fb={key(r):r for r in d['blackouts'] if r['variant']=='baseline'};fa={key(r):r for r in d['blackouts'] if r['variant']=='candidate'};assert fb.keys()==fa.keys()
  ft={}
  for metric in ['speed_error_mps','distance_error_m']:
   ft[metric]=sorted([{'bag':k[0],'start_s':k[1],'horizon_s':k[2],'baseline_error':fb[k][metric],'candidate_error':fa[k][metric],'absolute_error_delta':abs(fa[k][metric])-abs(fb[k][metric])} for k in fb],key=lambda r:r['absolute_error_delta'],reverse=True)[:10]
  s={'pooled':summarize(d),'vehicles':{v:summarize(d,v) for v in sorted({r['vehicle'] for r in d['nominal']})},'sessions':summary,'nominal_bag_tails':tails,'blackout_window_tails':ft,'coverage':{'identical_masks_and_output_counts':all(all(b[k][f]==a[k][f] for f in ['n','outputs','common_outputs','mask_sha256']) for k in b),'unscored_bags':[k for k in b if not b[k]['n']],'identical_blackout_endpoint_sets':True},'provenance':d['provenance']}
  report['splits'][split]=s
  for name in ['results','summary']:shutil.copyfile(P/split/(name+'.json'),OUT/f'kalman_stop_{split}_{name}.json')
  (OUT/f'kalman_stop_{split}_comparison.json').write_text(json.dumps(s,indent=2)+'\n')
 for vehicle in [30618,30639]:
  path=P/f'synthetic_{vehicle}.json';d=json.loads(path.read_text());assert d['provenance']['executables']['after']['sha256']==freeze['candidate']['replay_cli'];rows=d['scenarios'];reg=[]
  for r in rows:
   a=r['before']['recovery_to_0p1_mps_for_1s_s'];b=r['after']['recovery_to_0p1_mps_for_1s_s']
   if a is not None and (b is None or b>a+.2):reg.append({'profile':r['profile'],'fault':r['fault'],'seed':r['seed'],'before':a,'after':b})
  report['synthetic'][vehicle]={'cases':len(rows),'full_coverage':all(r['before']['coverage']==1 and r['after']['coverage']==1 for r in rows),'recovery_regressions_over_0p2s':reg}
  shutil.copyfile(path,OUT/f'kalman_stop_synthetic_{vehicle}.json')
 report['evaluator_source_sha256']={s:sha(ROOT/s) for s in ['evaluation/round4_kalman_stop.py','evaluation/round4_kalman_stop_test.cpp','evaluation/round4_kalman_stop_report.py','evaluation/round4_kalman_generalization.py','evaluation/speed_candidate_study.py','evaluation/wheel_fault_benchmark.py','evaluation/causal_baseline.py','evaluation/core_benchmark.py','evaluation/table_blackout_ablation.py','evaluation/calibrate_drive.py']}
 (OUT/'kalman_stop_decision.json').write_text(json.dumps(report,indent=2)+'\n')
 print('SHA',freeze['candidate']['replay_cli'])
 for split,s in report['splits'].items():
  print(split,s['pooled']);print('vehicle30618',s['vehicles'].get('30618'));print('worst windows',s['blackout_window_tails']['speed_error_mps'][:2],s['blackout_window_tails']['distance_error_m'][:2])
if __name__=='__main__':main()
