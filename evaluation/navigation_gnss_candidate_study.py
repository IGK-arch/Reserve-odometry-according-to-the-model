#!/usr/bin/env python3
"""Compare a GNSS candidate to cached navigation_gnss_stress outputs.

Run navigation_gnss_stress.py with the frozen executable first. This study reuses
those exact permitted-input CSVs, reconstructs the independent paired-GNSS proxy,
and requires equal publication stamps and absolute-position masks before scoring.
Use --baseline-exe when cached output predates frame diagnostics; every original
output field must equal the frozen CSV before appended metadata is used.
It never supplies the proxy to either executable. Candidate output CSVs are
scratch files; their hashes and all per-trip metrics are retained in report.json.

Example:
  python3 evaluation/navigation_gnss_candidate_study.py \
    --baseline-dir /private/tmp/gnss-round2-baseline \
    --exe build/navigation_replay --outdir /private/tmp/gnss-round2-current \
    --split train validation
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np

# Also permits running a saved scratch copy with --source-root.
if '--source-root' in sys.argv:
    source_root=Path(sys.argv[sys.argv.index('--source-root')+1]).resolve()
else:
    source_root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(source_root/'evaluation'))
import navigation_gnss_stress as stress


def aggregate(rows):
    result={}
    for session in sorted({row['session'] for row in rows}):
        result[session]={}
        session_rows=[row for row in rows if row['session']==session]
        for mode in stress.MODES:
            eligible=[row['modes'][mode] for row in session_rows if row['modes'][mode]['baseline']['n']]
            matched=sum(row['modes'][mode]['baseline']['proxy_matched_count'] for row in session_rows)
            summary={'attempted_bags':len(session_rows), 'scored_bags':len(eligible),
                     'n':sum(r['baseline']['n'] for r in eligible), 'proxy_matched_count':matched,
                     'no_absolute_position_bags':[row['bag'] for row in session_rows if not row['modes'][mode]['baseline']['n']]}
            summary['absolute_position_coverage']=summary['n']/matched if matched else None
            for version in ('baseline','candidate'):
                errors=[r[version]['rmse_3d_m'] for r in eligible]
                summary[version+'_bag_median_rmse_m']=float(np.median(errors)) if errors else None
                summary[version+'_pooled_rmse_m']=(sum(r[version]['n']*r[version]['rmse_3d_m']**2 for r in eligible)/summary['n'])**.5 if summary['n'] else None
            summary['improved_bags']=sum(r['candidate']['rmse_3d_m']<r['baseline']['rmse_3d_m']-1e-9 for r in eligible)
            summary['worse_bags']=sum(r['candidate']['rmse_3d_m']>r['baseline']['rmse_3d_m']+1e-9 for r in eligible)
            result[session][mode]=summary
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,default=source_root)
    parser.add_argument('--dataset-root',type=Path,default=source_root/'dataset/data')
    parser.add_argument('--baseline-dir',type=Path,required=True)
    parser.add_argument('--exe',type=Path,required=True)
    parser.add_argument('--baseline-exe',type=Path,help='Frozen replay rebuilt only to append frame diagnostics; original output fields must remain exactly equal')
    parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--split',nargs='+',choices=('train','validation'),default=['train'])
    args=parser.parse_args()
    args.exe=args.exe.resolve(); args.outdir.mkdir(parents=True,exist_ok=True)
    baseline=json.loads((args.baseline_dir/'report.json').read_text())
    manifest=json.loads((source_root/'tools/split_manifest.json').read_text())
    groups={bag:group for group in manifest['groups'] for bag in group['bags']}
    projection=stress.build_projection(args.outdir,'c++')
    report={'note':'Paired-GNSS proxy only; fixed inputs and common valid masks, no parameter fitting.',
            'baseline_report_sha256':stress.sha256(args.baseline_dir/'report.json'),
            'candidate_executable_sha256':stress.sha256(args.exe),
            'baseline_diagnostic_executable_sha256':stress.sha256(args.baseline_exe) if args.baseline_exe else None,
            'command':sys.argv,'bags':[],'skipped':[]}
    for original in baseline['bags']:
        bag=original['bag']
        if original['split'] not in args.split or not bag.startswith('30618_'): continue
        if 'modes' not in original:
            report['skipped'].append(original); continue
        inputs=stress.read_permitted(args.dataset_root/bag)
        reference_t,reference_p=stress.scoring_proxy(inputs,projection)
        row={'bag':bag,'split':original['split'],'session':groups[bag]['date'],'modes':{}}
        for mode in stress.MODES:
            input_path=args.baseline_dir/f'{bag}_{mode}_input.csv'
            with (args.baseline_dir/f'{bag}_{mode}_output.csv').open() as stream:
                old=list(csv.DictReader(stream))
            if args.baseline_exe:
                diagnostic=stress.replay(args.baseline_exe,input_path,args.outdir/'baseline_scratch_output.csv',30618)
                if len(old)!=len(diagnostic) or any(any(row[key]!=updated[key] for key in row) for row,updated in zip(old,diagnostic)):
                    raise ValueError(f'{bag} {mode}: baseline diagnostic rebuild changed original output fields')
                old=diagnostic
            output_path=args.outdir/'scratch_output.csv'
            new=stress.replay(args.exe,input_path,output_path,30618)
            old_result,old_t,old_p,old_state=stress.score(old,reference_t,reference_p)
            new_result,new_t,new_p,new_state=stress.score(new,reference_t,reference_p)
            if not np.array_equal(old_t,new_t): raise ValueError(f'{bag} {mode}: stamps changed')
            old_valid=np.array([r['position_valid']=='1' and r['frame_id']=='mgrs_37UCB' for r in old])&np.isfinite(old_p).all(axis=1)
            new_valid=np.array([r['position_valid']=='1' and r['frame_id']=='mgrs_37UCB' for r in new])&np.isfinite(new_p).all(axis=1)
            if not np.array_equal(old_valid,new_valid): raise ValueError(f'{bag} {mode}: valid masks changed')
            row['modes'][mode]={'baseline':old_result,'candidate':new_result,
                'identical_velocity_and_wheel_distance':bool(np.array_equal(old_state,new_state)),
                'identical_publication_stamps_and_valid_mask':True,
                'baseline_diagnostic_preserves_frozen_output_fields':True if args.baseline_exe else None,
                'max_position_change_m':float(np.linalg.norm(new_p-old_p,axis=1).max()),
                'input_sha256':stress.sha256(input_path),'output_sha256':stress.sha256(output_path)}
        report['bags'].append(row)
        print(bag,json.dumps({mode:{v:r[v]['rmse_3d_m'] for v in ('baseline','candidate')} for mode,r in row['modes'].items()}),flush=True)
    report['sessions']=aggregate(report['bags'])
    (args.outdir/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.outdir/'scratch_output.csv').unlink(missing_ok=True)
    (args.outdir/'baseline_scratch_output.csv').unlink(missing_ok=True)
    print(json.dumps(report['sessions'],indent=2))
    return 0

if __name__=='__main__': raise SystemExit(main())
