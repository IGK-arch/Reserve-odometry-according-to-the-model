#!/usr/bin/env python3
"""Freeze train-only Kalman research recommendation with explicit rejection gates."""
from pathlib import Path
import json,math,hashlib,shutil
from round4_kalman import VARIANTS
ROOT=Path(__file__).resolve().parents[1];SCRATCH=Path('/private/tmp/round4-kalman');OUT=ROOT/'evaluation/results/round4'
def rmse(rows,key):return math.sqrt(sum(r[key]**2 for r in rows)/len(rows)) if rows else None

def main():
 tests=json.loads((OUT/'kalman_existing_tests.json').read_text());reports=[]
 for name in VARIANTS:
  folder=SCRATCH/name;d=json.loads((folder/'train/results.json').read_text());summary=json.loads((folder/'train/summary.json').read_text());r={'variant':name,'parameters':VARIANTS[name],'pooled':{},'sessions':summary,'failed_existing_tests':[x for x in tests if x['variant']==name and x['returncode']], 'synthetic':{}}
  for variant in ['baseline','candidate']:
   rows=[x for x in d['nominal'] if x['variant']==variant];n=sum(x['n'] for x in rows)
   r['pooled'][variant]={'bags':len(rows),'matched_n':n,'nominal_rmse_mps':math.sqrt(sum(x['n']*x['rmse_mps']**2 for x in rows if x['n'])/n),'blackouts':{}}
   for h in [1,3,5]:
    f=[x for x in d['blackouts'] if x['variant']==variant and x['horizon_s']==h];r['pooled'][variant]['blackouts'][h]={'n':len(f),'speed_rmse_mps':rmse(f,'speed_error_mps'),'distance_rmse_m':rmse(f,'distance_error_m')}
  b={x['bag']:x for x in d['nominal'] if x['variant']=='baseline'};a={x['bag']:x for x in d['nominal'] if x['variant']=='candidate'}
  r['bag_tails']=sorted([{'bag':k,'baseline':b[k]['rmse_mps'],'candidate':a[k]['rmse_mps'],'delta':a[k]['rmse_mps']-b[k]['rmse_mps']} for k in b],key=lambda x:x['delta'],reverse=True)
  r['identical_nominal_coverage_masks']=all(all(b[k][f]==a[k][f] for f in ['n','outputs','common_outputs','mask_sha256']) for k in b)
  for vehicle in [30618,30639]:
   data=json.loads((folder/f'synthetic_{vehicle}.json').read_text())['scenarios'];items=[]
   for x in data:
    q={'profile':x['profile'],'fault':x['fault'],'seed':x['seed']}
    for metric in ['speed_rmse_mps','distance_rmse_m','recovery_to_0p1_mps_for_1s_s']:
     q[metric]={v:x[v][metric] for v in ['before','after']}
    items.append(q)
   recovery_regressions=[x for x in items if x['recovery_to_0p1_mps_for_1s_s']['before'] is not None and (x['recovery_to_0p1_mps_for_1s_s']['after'] is None or x['recovery_to_0p1_mps_for_1s_s']['after']>x['recovery_to_0p1_mps_for_1s_s']['before']+.2)]
   r['synthetic'][vehicle]={'scenarios':len(items),'full_coverage':all(x['before']['coverage']==1 and x['after']['coverage']==1 for x in data),'recovery_regressions_over_0p2s':recovery_regressions,'worst_speed_deltas':sorted(items,key=lambda x:x['speed_rmse_mps']['after']-x['speed_rmse_mps']['before'],reverse=True)[:5],'worst_distance_deltas':sorted(items,key=lambda x:x['distance_rmse_m']['after']-x['distance_rmse_m']['before'],reverse=True)[:5]}
   shutil.copyfile(folder/f'synthetic_{vehicle}.json',OUT/f'kalman_{name}_synthetic_{vehicle}.json')
  r['eligible_for_validation']=not r['failed_existing_tests'] and r['identical_nominal_coverage_masks'] and all(s['full_coverage'] and not s['recovery_regressions_over_0p2s'] for s in r['synthetic'].values()) and r['pooled']['candidate']['nominal_rmse_mps']<r['pooled']['baseline']['nominal_rmse_mps'] and all(r['pooled']['candidate']['blackouts'][h]['speed_rmse_mps']<=r['pooled']['baseline']['blackouts'][h]['speed_rmse_mps'] for h in [1,3,5])
  r['decision']='eligible train-only candidate' if r['eligible_for_validation'] else 'rejected: existing behavioral test failures and/or train degradation'
  reports.append(r)
  for f in ['results','summary']:shutil.copyfile(folder/'train'/f'{f}.json',OUT/f'kalman_{name}_train_{f}.json')
  shutil.copyfile(folder/'candidate.patch',OUT/f'kalman_{name}.patch')
 eligible=[x for x in reports if x['eligible_for_validation']];selected=min(eligible,key=lambda x:x['pooled']['candidate']['nominal_rmse_mps'])['variant'] if eligible else None
 result={'protocol':__doc__,'selected_for_validation':selected,'selection_basis':'Train only, after analytic fixtures. Existing estimator behavior tests are a hard gate; synthetic coverage and recovery delays are checked. No claim that all synthetic error metrics improve.','provenance':json.loads((SCRATCH/'provenance.json').read_text()),'variants':reports,'limitations':['This is an experimental train-only recommendation, not a production deployment decision.','Three original noise/time-constant variants preceded measurement. A fourth structural repair was added after existing behavior tests failed: reset acceleration variance1, residual bound±4, legacy velocity uncertainty during unhealthy-wheel fallback.','Finite covariance is propagated jointly, but clamped state innovations and residual saturation make Gaussian covariance approximate.','Synthetic trip pair_freeze_transition and some pair_freeze_brake/distance errors increase; inspect per-case tails.','Existing original fault gates are preserved syntactically, but changing predictor trajectories can change their decisions.','No validation, holdout, or public results were inspected for this experiment.']}
 (OUT/'kalman_decision.json').write_text(json.dumps(result,indent=2)+'\n');print('selected_for_validation',selected)
 for x in reports:print(x['variant'],x['decision'],x['pooled'])
if __name__=='__main__':main()
