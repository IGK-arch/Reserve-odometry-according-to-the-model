#!/usr/bin/env python3
"""Compare two output CSVs against reference Odometry on identical sample masks.

Velocity and position are independent channels. For each channel/header stamp,
keep the publication with greatest receive_ns; equal receive_ns uses the later
CSV row. Thus separate ROS velocity/position rows do not overwrite each other.
Both candidates must have finite values at the same exact header stamp before
matching a reference within inclusive 50 ms. This is not ROS ATS scoring.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from reference_benchmark import (
    ROOT, ReferenceIndex, error_metrics, finite_position, gaps, provenance,
    read_bag, read_candidate,
)


def channel_rows(samples, channel):
    rows = [sample for sample in samples if sample.velocity_present] if channel == 'velocity' else [
        sample for sample in samples if sample.position is not None
    ]
    selected = {}
    for sample in rows:
        previous = selected.get(sample.stamp_ns)
        if previous is None or sample.receive_ns >= previous.receive_ns:
            selected[sample.stamp_ns] = sample
    finite = lambda sample: math.isfinite(sample.velocity_mps) if channel == 'velocity' else finite_position(sample.position)
    counts = {
        'published_rows': len(rows), 'unique_stamps': len(selected),
        'duplicate_rows': len(rows) - len(selected),
        'valid_unique_stamps': sum(finite(sample) for sample in selected.values()),
        'invalid_unique_stamps': sum(not finite(sample) for sample in selected.values()),
        'gaps': gaps(sorted(selected.values(), key=lambda sample: sample.receive_ns)),
    }
    return selected, counts


def position_metrics(pairs):
    errors = [tuple(actual - expected for actual, expected in zip(sample.position, reference.position))
              for sample, reference in pairs]
    norms = [math.sqrt(sum(component * component for component in error)) for error in errors]
    return {
        'n': len(pairs),
        **{axis: error_metrics([error[i] for error in errors], 'm') for i, axis in enumerate('xyz')},
        'rmse_3d_m': error_metrics(norms, 'm')['rmse_m'],
        'max_3d_m': max(norms) if norms else None,
        'end_error_3d_m': norms[-1] if norms else None,
        'end_error_xyz_m': errors[-1] if errors else None,
        'end_stamp_ns': pairs[-1][0].stamp_ns if pairs else None,
    }


def compare(before, after, reference, tolerance_ns=50_000_000):
    if tolerance_ns < 0:
        raise ValueError('tolerance_ns must be nonnegative')
    index = ReferenceIndex(reference)
    report = {
        'matching': {'method': 'exact candidate header stamp intersection, nearest reference header stamp',
                     'tolerance_ns': tolerance_ns, 'boundary': 'inclusive', 'ties': 'earlier reference stamp',
                     'reference_reuse': True, 'equivalent_to_ros_ats': False,
                     'reference_velocity_field': 'twist.twist.linear.x'},
        'duplicate_policy': 'independent per channel and header stamp: greatest receive_ns; ties use later CSV row; retain nonfinite latest publications before masking',
        'coverage_denominator': 'unique published channel stamps, including nonfinite values, before intersection',
        'before_counts': {'rows': len(before)}, 'after_counts': {'rows': len(after)},
        'reference_counts': {'rows': len(reference), 'unique_stamps': len(set(index.stamps))},
    }
    for channel in ('velocity', 'position'):
        before_rows, report['before_counts'][channel] = channel_rows(before, channel)
        after_rows, report['after_counts'][channel] = channel_rows(after, channel)
        finite = lambda sample: math.isfinite(sample.velocity_mps) if channel == 'velocity' else finite_position(sample.position)
        shared = sorted(before_rows.keys() & after_rows.keys())
        shared_finite = [stamp for stamp in shared if finite(before_rows[stamp]) and finite(after_rows[stamp])]
        matched, outside, invalid_reference = [], 0, 0
        for stamp in shared_finite:
            ref = index.nearest(stamp, tolerance_ns)
            if ref is None:
                outside += 1
            elif not finite(ref):
                invalid_reference += 1
            else:
                matched.append((before_rows[stamp], after_rows[stamp], ref))
        stamps = [before_sample.stamp_ns for before_sample, _, _ in matched]
        before_pairs = [(sample, ref) for sample, _, ref in matched]
        after_pairs = [(sample, ref) for _, sample, ref in matched]
        offsets = [abs(sample.stamp_ns - ref.stamp_ns) for sample, ref in before_pairs]
        mask = {
            'shared_stamps': len(shared), 'shared_finite_stamps': len(shared_finite),
            'excluded_nonfinite_candidate': len(shared) - len(shared_finite),
            'outside_tolerance': outside, 'invalid_reference': invalid_reference,
            'matched': len(matched),
            'before_only_stamps': len(before_rows.keys() - after_rows.keys()),
            'after_only_stamps': len(after_rows.keys() - before_rows.keys()),
            'before_coverage': len(matched) / len(before_rows) if before_rows else 0.,
            'after_coverage': len(matched) / len(after_rows) if after_rows else 0.,
            'intersection_coverage': len(matched) / len(shared) if shared else 0.,
            'first_stamp_ns': stamps[0] if stamps else None,
            'last_stamp_ns': stamps[-1] if stamps else None,
            'stamp_mask_sha256': hashlib.sha256(''.join(f'{stamp}\n' for stamp in stamps).encode()).hexdigest(),
            'unique_reference_stamps': len({ref.stamp_ns for _, _, ref in matched}),
            'max_abs_reference_offset_ms': max(offsets) / 1e6 if offsets else None,
            'gaps': gaps([sample for sample, _ in before_pairs]),
        }
        metrics = (lambda pairs: error_metrics([sample.velocity_mps - ref.velocity_mps for sample, ref in pairs], 'mps')) if channel == 'velocity' else position_metrics
        report[channel] = {'mask': mask, 'before': metrics(before_pairs), 'after': metrics(after_pairs)}
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--before', required=True, type=Path)
    parser.add_argument('--after', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--tolerance-ms', type=float, default=50.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.tolerance_ms) or args.tolerance_ms < 0:
        parser.error('--tolerance-ms must be finite and nonnegative')
    _, reference, _ = read_bag(args.bag)
    before, after = read_candidate(args.before), read_candidate(args.after)
    report = compare(before, after, reference, round(args.tolerance_ms * 1e6))
    report['provenance'] = provenance(args.bag, [args.before, args.after, Path(__file__),
                                               ROOT / 'evaluation/reference_benchmark.py',
                                               ROOT / 'analysis/viewer/rosbag_cdr.py'])
    report['provenance']['before_csv'] = str(args.before.resolve())
    report['provenance']['after_csv'] = str(args.after.resolve())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if any(report[channel]['mask']['matched'] for channel in ('velocity', 'position')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
