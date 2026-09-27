#!/usr/bin/env python3
"""Frozen-executable GNSS comparison with explicit startup coverage changes.

Uses cached permitted inputs, rebuilds the paired-GNSS proxy from original bags,
reports each version's coverage AND errors on their common absolute mask.
This is diagnostic GNSS truth, not the official fused reference. No fitting.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import navigation_gnss_stress as stress


def compare(old, new, reference_t, reference_p):
    a, at, ap, av = stress.score(old, reference_t, reference_p)
    b, bt, bp, bv = stress.score(new, reference_t, reference_p)
    if not np.array_equal(at, bt):
        raise ValueError('Publication stamps differ; explicit outer matching is required')
    index, age = stress.nearest(reference_t, at)
    def mask(rows, points):
        return (np.array([r['position_valid']=='1' and r['frame_id']=='mgrs_37UCB'
                          for r in rows]) & np.isfinite(points).all(axis=1)
                & (age <= 50_000_000))
    am, bm = mask(old, ap), mask(new, bp)
    common = am & bm
    def rmse(points):
        return float(np.sqrt(np.mean(np.sum((points[common]-reference_p[index[common]])**2,
                                            axis=1)))) if common.any() else None
    return {'baseline': a, 'candidate': b,
            'common': {'n': int(common.sum()), 'baseline_rmse_m': rmse(ap),
                       'candidate_rmse_m': rmse(bp),
                       'mask_sha256': hashlib.sha256(at[common].tobytes()).hexdigest()},
            'lost_absolute_matches': int((am & ~bm).sum()),
            'gained_absolute_matches': int((bm & ~am).sum()),
            'identical_velocity_and_distance': bool(np.array_equal(av, bv))}


def aggregate(rows):
    summary = {}
    for session in sorted({r['session'] for r in rows}):
        summary[session] = {}
        for mode in stress.MODES:
            samples = [r['modes'][mode] for r in rows if r['session']==session]
            n = sum(r['common']['n'] for r in samples)
            out = {'bags': len(samples), 'common_n': n,
                   'lost_absolute_matches': sum(r['lost_absolute_matches'] for r in samples),
                   'gained_absolute_matches': sum(r['gained_absolute_matches'] for r in samples)}
            for version in ('baseline', 'candidate'):
                out[version+'_n'] = sum(r[version]['n'] for r in samples)
                out[version+'_common_rmse_m'] = (sum(r['common']['n'] * r['common'][version+'_rmse_m']**2
                    for r in samples if r['common']['n']) / n)**.5 if n else None
            summary[session][mode] = out
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline-exe', type=Path, required=True)
    p.add_argument('--candidate-exe', type=Path, required=True)
    p.add_argument('--inputs-dir', type=Path, required=True)
    p.add_argument('--outdir', type=Path, required=True)
    args = p.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    cached = json.loads((args.inputs_dir/'report.json').read_text())
    manifest = json.loads((root/'tools/split_manifest.json').read_text())
    groups = {b:g for g in manifest['groups'] for b in g['bags']}
    projection = stress.build_projection(args.outdir, 'c++')
    report = {'protocol': __doc__, 'command': sys.argv,
              'executables_sha256': {v:stress.sha256(exe) for v,exe in
                   [('baseline',args.baseline_exe),('candidate',args.candidate_exe)]},
              'script_sha256': stress.sha256(Path(__file__)), 'bags': [], 'skipped': []}
    for original in cached['bags']:
        bag = original['bag']
        if 'modes' not in original:
            report['skipped'].append(original)
            continue
        inputs = stress.read_permitted(root/'dataset/data'/bag)
        rt, rp = stress.scoring_proxy(inputs, projection)
        row = {'bag': bag, 'session': groups[bag]['date'], 'split': original['split'], 'modes': {},
               'proxy_t_sha256': hashlib.sha256(rt.tobytes()).hexdigest(),
               'proxy_xyz_sha256': hashlib.sha256(rp.tobytes()).hexdigest()}
        for mode in stress.MODES:
            path = args.inputs_dir/f'{bag}_{mode}_input.csv'
            old = stress.replay(args.baseline_exe.resolve(), path, args.outdir/'baseline.csv', int(bag[:5]))
            new = stress.replay(args.candidate_exe.resolve(), path, args.outdir/'candidate.csv', int(bag[:5]))
            result = compare(old, new, rt, rp)
            result['input_sha256'] = stress.sha256(path)
            result['output_sha256'] = {v:stress.sha256(args.outdir/f'{v}.csv') for v in ('baseline','candidate')}
            row['modes'][mode] = result
        report['bags'].append(row)
        print(bag, flush=True)
    report['sessions'] = aggregate(report['bags'])
    report['provenance'] = stress.provenance(root/'dataset/data', args.candidate_exe.resolve(),
        [(r['bag'], r['split']) for r in report['bags']])
    report['provenance']['cached_report_sha256'] = stress.sha256(args.inputs_dir/'report.json')
    report['provenance']['manifest_sha256'] = stress.sha256(root/'tools/split_manifest.json')
    (args.outdir/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
