#!/usr/bin/env python3
"""Reproduce fixed navigation ablations; reference is never passed to C++."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

from reference_benchmark import ROOT, read_bag, read_candidate, evaluate, provenance
from causal_baseline import causal_wheel_baseline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--exe', type=Path, default=ROOT / 'build/navigation_replay')
    parser.add_argument('--outdir', type=Path, default=ROOT / 'evaluation/runs/reference_ablation')
    args = parser.parse_args()
    exe = args.exe.resolve()
    if not exe.is_file():
        parser.error('Build navigation_replay with the root CMake project first.')
    if args.outdir.exists() and any(args.outdir.iterdir()):
        parser.error('--outdir must be new or empty; previous evidence is preserved')
    args.outdir.mkdir(parents=True, exist_ok=True)
    inputs_path = args.outdir / 'permitted_inputs.csv'
    events, reference, inputs = read_bag(args.bag)
    if not reference:
        parser.error('The requested bag has no localization/kinematic_state reference.')
    with inputs_path.open('w', newline='') as stream:
        csv.writer(stream).writerows(inputs)
    assets = ROOT / 'ros2_ws/src/tram_odometry/assets'
    common = [str(exe), '--input', str(inputs_path.resolve()), '--map', str(assets / 'route_map.csv'),
              '--drive-table', str(assets / 'drive_accel_table.csv')]
    profiles = {
        'startup': ['--gnss-mode', 'startup'],
        'corrections': ['--gnss-mode', 'corrections'],
        'branch': ['--gnss-mode', 'corrections', '--alternate-map', str(assets / 'route_map_branch_a.csv')],
        'full': ['--gnss-mode', 'corrections', '--alternate-map', str(assets / 'route_map_branch_a.csv'),
                 '--elevation', str(assets / 'official_elevation.csv')],
    }
    baseline = causal_wheel_baseline(events)
    report = {'protocol': 'All profiles use the same current speed estimator; only navigation sources change.',
              'matching': 'Offline nearest reference <=50ms, not official ROS ATS.', 'profiles': {}}
    artifacts = [exe, Path(__file__), ROOT / 'evaluation/navigation_replay.cpp',
                 ROOT / 'evaluation/reference_benchmark.py', ROOT / 'analysis/viewer/rosbag_cdr.py']
    artifacts += sorted((ROOT / 'ros2_ws/src/tram_odometry/src').glob('*.cpp'))
    artifacts += sorted((ROOT / 'ros2_ws/src/tram_odometry/include/tram_odometry').glob('*.hpp'))
    artifacts += [assets / name for name in ('route_map.csv', 'route_map_branch_a.csv',
                                           'drive_accel_table.csv', 'official_elevation.csv')]
    for name, options in profiles.items():
        output = args.outdir / f'{name}.csv'
        command = [*common, '--output', str(output.resolve()), *options]
        subprocess.run(command, cwd=ROOT, check=True)
        metrics = evaluate(read_candidate(output), reference, baseline)
        # Cumulative counters create thousands of unhelpful histogram keys.
        metrics['candidate'].pop('flag_counts', None)
        report['profiles'][name] = {'command': command, 'metrics': metrics}
        print(name, metrics['candidate']['position']['rmse_3d_m'], flush=True)
    report['provenance'] = provenance(args.bag, artifacts)
    report['provenance']['command'] = sys.argv
    (args.outdir / 'ablation.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
