#!/usr/bin/env python3
"""Freeze/reject the three position prototypes and retain exact patches/evidence."""
from pathlib import Path
import json,hashlib,difflib,shutil
from round4_position_build import S,O,VARIANTS,sha,ROOT
D=ROOT/'evaluation/results/round4'
def main():
 D.mkdir(parents=True,exist_ok=True);train=json.loads((O/'train_report.json').read_text());assert len(train['bags'])==33
 syn=json.loads((O/'synthetic_report.json').read_text());build=json.loads((O/'build.json').read_text())
 # Verify source/executable correspondence after all concurrent replays finish.
 for name in VARIANTS:
  assert sha(O/name/'navigation_replay')==build[name]['binary_sha256']
  patch=''
  for rel,candidate in [('ros2_ws/src/tram_odometry/src/navigation.cpp',O/name/'navigation.cpp'),('ros2_ws/src/tram_odometry/include/tram_odometry/navigation.hpp',O/name/'include/tram_odometry/navigation.hpp')]:
   patch+=''.join(difflib.unified_diff((S/rel).read_text().splitlines(True),candidate.read_text().splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
  p=D/('position_'+name+'.patch');p.write_text(patch);build[name]['full_patch_sha256']=sha(p);build[name]['minimum_separated_windows_for_drift']=2;build[name]['separate_window_gap_s']=10
 # Independent long-gap covariance check records an implementation limitation.
 p00,p01,p11=4.,0.,4e-6;cov={}
 for distance in range(1,20001):
  p00=min(400.,p00+2*p01+p11+.0025);p01+=p11;p11=min(1e-4,p11+1e-8)
  det=p00*p11-p01*p01
  if det<0:cov={'first_negative_determinant_distance_m':distance,'determinant':det,'P00':p00,'P01':p01,'P11':p11,'disposition':'Numerical cap is not PSD-preserving. Reject these prototype binaries; no production integration. A future version must use Joseph update and PSD-preserving bounds and repeat selection.'};break
 tails={}
 for name in VARIANTS:
  values=[]
  for bag in train['bags']:
   for mode,r in bag['modes'].items():
    if r['baseline']['xy']['n']:values.append({'bag':bag['bag'],'session':bag['session'],'mode':mode,'baseline_xy_rmse_m':r['baseline']['xy']['rmse_m'],'candidate_xy_rmse_m':r[name]['xy']['rmse_m'],'delta_xy_rmse_m':r[name]['xy']['rmse_m']-r['baseline']['xy']['rmse_m'],'baseline_max_xy_m':r['baseline']['xy']['max_m'],'candidate_max_xy_m':r[name]['xy']['max_m']})
  tails[name]={'worst':sorted(values,key=lambda r:r['delta_xy_rmse_m'],reverse=True)[:10],'best':sorted(values,key=lambda r:r['delta_xy_rmse_m'])[:10]}
 decision={'decision':'REJECT all three position-only prototypes; retain baseline fixed-gain guarded GNSS correction.','rationale':'No robust cross-mode train improvement; constant-scale synthetic benefit trades against bad-GNSS extrapolation and per-bag tails. Distance-domain propagation correctly stops during blackout at standstill. Covariance cap also needs numerical redesign. These findings reject the implemented bounded variants, not Kalman filtering in general.','public_validation_holdout_read':False,'frozen_baseline_commit':'5a82712','frozen_source_root':str(S),'build':build,'train_report_sha256':sha(O/'train_report.json'),'synthetic_report_sha256':sha(O/'synthetic_report.json'),'covariance_check':cov,'tails':tails,'scripts_sha256':{p.name:sha(p) for p in (ROOT/'evaluation').glob('round4_position*.py')}}
 for a,b in [('train_report.json','position_train.json'),('synthetic_report.json','position_synthetic.json')]:shutil.copyfile(O/a,D/b)
 (D/'position_decision.json').write_text(json.dumps(decision,indent=2)+'\n')
 lines=['# Position offset/scale uncertainty experiment — rejected','','Three predefined scratch candidates were generated from immutable `5a82712` sources. Startup and the physical estimator were unchanged. All existing GNSS age, payload-freeze, pair-baseline, route/branch and three-fix-median gates remain. Only the Navigation along-track position anchor is updated.','',
 'The model is `a(d+Δd)=a(d)+b·Δd`, with anchor offset `a` in metres and scale-like drift `b` in m/m. Published wheel distance and velocity are never changed. Measurement is the gated median of `match.s − wheel_distance`. The scalar variant fixes b=0; drift variants bound |b| to0.005 or0.015. Initial covariance is Paa=4m², Pbb=4e-6; process variance grows0.0025m²/m and drift variance1e-8 per metre. R is0.25m² for status2 and4m² otherwise. Updates are at least0.5s apart across both antennas; drift gain remains zero until two GNSS windows separated by>10s. These are engineering assumptions, not measured GNSS covariance. Overlapping medians and common GNSS biases remain correlated.','',
 '## Synthetic behavior','','Known straight track, 10m/s cruise,1% wheel scale (healthy control uses correct scale),GNSS first30s then2s windows every120s, stop after270s. XY RMS after30s:','','| Case | Baseline | Scalar | Drift0.5% | Drift1.5% |','| --- | ---: | ---: | ---: | ---: |']
 for case,rs in syn['cases'].items():lines.append('| '+case+' | '+' | '.join(f"{rs[n]['moving_and_stop_after30s']['rmse']:.4f}" for n in ['baseline',*VARIANTS])+' |')
 lines+=['','All variants have exactly zero position range during the stopped GNSS blackout280–350s. Stopped position can subsequently change when a legitimate new fix arrives; that is a correction, not time-based drift. A common20m badGNSS window passes inherited single-antenna gates and is amplified by drift propagation. Frozen and delayed cases do not improve.','',
 '## Train-only results','','33 unique30618 train bags × four input modes × baseline plus three candidates =528 Navigation replays. The initial30s GNSS period permits ongoing corrections; startup selection itself is unchanged. No validation, holdout or public fused-reference file was read. Proxy is the round3 cleaned paired-GNSS+physical-TF estimate sampled one in ten timestamps, nearest output within50ms. Absolute-valid masks, output stamps, velocity and wheel distance match exactly; no registration or time fitting. Proxy errors are diagnostics, not official scores.','',
 '| Mode | Baseline pooledXY RMS | Scalar | Drift0.5% | Drift1.5% | Samples |','| --- | ---: | ---: | ---: | ---: | ---: |']
 for mode,rs in train['summary']['all'].items():lines.append('| '+mode+' | '+' | '.join(f"{rs[n]['pooled_xy_rmse_m']:.4f}" for n in ['baseline',*VARIANTS])+f" | {rs['n']} |")
 lines+=['','The JSON retains each bag/session/mode, XY and XYZ RMS, p95, maximum and final error, coverage, hashes, and best/worst deltas. Do not choose solely by pooled RMS: bad GNSS and startup tails determine rejection.','', '## Numerical limitation and decision','','The prototype covariance independently caps its diagonals while allowing the cross term to grow. In a standalone no-measurement propagation check its determinant becomes negative after5050m. This is not a production-safe covariance bound. No prototype is integrated; any future attempt requires PSD-preserving bounds/Joseph updates and a new train selection. Current rejection is specific to these implementations.','', '**Retain the baseline correction.** Constant wheel-scale errors are a favorable case for distance-domain drift; neither that synthetic gain nor occasional bag improvements outweigh the measured regressions and covariance limitation.','', '## Reproduce','','```sh','python3 evaluation/round4_position_build.py','python3 evaluation/round4_position_synthetic.py','python3 evaluation/round4_position_train.py','python3 evaluation/round4_position_report.py','```','','Frozen source and binaries are in `/private/tmp/odometry-round4-source` and `/private/tmp/odometry-round4-baseline`. Candidate source/executables and raw outputs are in `/private/tmp/round4-position`. Cached inputs are restricted by the train manifest before opening; the cache also contains validation files that this script does not use. Full candidate patches and executable/generator hashes are retained beside this note. Production sources and Git were not modified.']
 details=['## Session and failure-tail check','','For the drift1.5% prototype, the pooled frozen-mode gain is dominated by one September run: `30618_defd0170` improves58.8407→29.4414m RMS. This does not establish broad robustness: its sparse mode worsens12.0985→14.2904m, while July frozen data worsen in aggregate.','','| Session / mode | BaselineXY RMS | Drift1.5% |','| --- | ---: | ---: |']
 for session,summary in train['summary'].items():
  if session=='all':continue
  for mode,row in summary.items():details.append(f"| {session} / {mode} | {row['baseline']['pooled_xy_rmse_m']:.4f} | {row['drift_015']['pooled_xy_rmse_m']:.4f} |")
 details+=['','Drift1.5% improves>0.1m on3 sparse bags but worsens8 (worst+2.1919m). For frozen GNSS,7 improve and11 worsen (worst+1.5503m). The latter includes `30618_68d1748a`, whose RMS grows1.4611→3.0114m. Scalar correction has a much larger frozen failure,18.4711→41.9858m on `30618_0686195f`. These tails motivate rejection despite favorable pooled values.','']
 at=lines.index('## Numerical limitation and decision');lines[at:at]=details
 (D/'position_decision.md').write_text('\n'.join(lines)+'\n');print(json.dumps(train['summary']['all'],indent=2))
if __name__=='__main__':main()
