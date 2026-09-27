#!/usr/bin/env python3
"""Independent offline comparison with /localization/kinematic_state.

Each result uses its nearest reference header stamp within an inclusive 50 ms
window (ties choose the earlier stamp). References may be reused. This is NOT
the official checker's arrival-order, finite-queue, one-to-one ROS approximate
time synchronizer. The official checker remains the authority for ROS scores.
Reference velocity is signed twist.twist.linear.x, never the vector norm.
Only C/F/R and permitted GNSS are exported to an estimator; reference Odometry
is read solely by this evaluator. All timestamp arithmetic uses integer ns.
"""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
import csv
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'analysis' / 'viewer'))
sys.path.insert(0, str(ROOT / 'evaluation'))
from rosbag_cdr import messages  # noqa: E402
from causal_baseline import CMD, FRONT, REAR, causal_wheel_baseline  # noqa: E402

REFERENCE = '/localization/kinematic_state'
INPUT_CODES = {
    CMD: 'C', FRONT: 'F', REAR: 'R',
    '/sensing/gnss/master/fix': 'MF', '/sensing/gnss/rover/fix': 'RF',
    '/sensing/gnss/master/vel': 'MV', '/sensing/gnss/rover/vel': 'RV',
}


@dataclass
class Sample:
    stamp_ns: int
    receive_ns: int
    velocity_mps: float = math.nan
    position: tuple[float, float, float] | None = None
    distance_m: float = math.nan
    flags: dict = field(default_factory=dict)
    velocity_present: bool = True


class ReferenceIndex:
    def __init__(self, samples):
        self.samples = sorted(samples, key=lambda sample: sample.stamp_ns)
        self.stamps = [sample.stamp_ns for sample in self.samples]

    def nearest(self, stamp_ns, tolerance_ns=50_000_000):
        if tolerance_ns < 0:
            raise ValueError('tolerance_ns must be nonnegative')
        if not self.samples:
            return None
        i = bisect.bisect_left(self.stamps, stamp_ns)
        candidates = range(max(0, i - 1), min(i + 1, len(self.samples)))
        nearest = min(candidates, key=lambda j: (abs(self.stamps[j] - stamp_ns), self.stamps[j]))
        return self.samples[nearest] if abs(self.stamps[nearest] - stamp_ns) <= tolerance_ns else None


def finite_position(position):
    return position is not None and len(position) == 3 and all(math.isfinite(x) for x in position)


def read_candidate(path):
    samples = []
    with Path(path).open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if not {'stamp_ns', 'velocity_mps'} <= set(reader.fieldnames or []):
            raise ValueError('candidate CSV requires stamp_ns and velocity_mps columns')
        for line, row in enumerate(reader, start=2):
            try:
                stamp = int(row['stamp_ns'])
                receive = int(row.get('receive_ns') or stamp)
                position = None
                valid = (row.get('position_valid') or '0').lower()
                if valid not in ('0', '1', 'false', 'true'):
                    raise ValueError('position_valid must be 0/1 or false/true')
                if valid in ('1', 'true'):
                    position = tuple(float(row[key]) for key in ('x', 'y', 'z'))
                flags = {key: value for key, value in row.items() if key not in {
                    'stamp_ns', 'receive_ns', 'velocity_mps', 'distance_m', 'position_valid', 'velocity_present', 'x', 'y', 'z'
                }}
                samples.append(Sample(stamp, receive, float(row['velocity_mps']), position,
                                      float(row.get('distance_m') or 'nan'), flags,
                                      (row.get('velocity_present') or '1').lower() in ('1', 'true')))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'{path}:{line}: {error}') from error
    return samples


def read_bag(bag):
    """Return wheel baseline events, scoring truth, and isolated replay input."""
    events, reference, inputs = [], [], []
    for topic, receive, msg in messages(Path(bag), set(INPUT_CODES) | {REFERENCE}):
        stamp = msg['stamp_ns']
        if topic == REFERENCE:
            reference.append(Sample(stamp, receive, msg['linear'][0], msg['position']))
            continue
        if topic in (FRONT, REAR, CMD):
            value = msg['position'] if topic == CMD else msg['velocity_raw']
            events.append((receive, stamp, topic, value))
            values = (value,)
        elif topic.endswith('/fix'):
            values = (msg['latitude'], msg['longitude'], msg['altitude'], msg['status'])
        else:
            values = msg['linear']
        inputs.append((receive, stamp, INPUT_CODES[topic], *values))
    return events, reference, inputs


def read_recorded_outputs(bag):
    """Keep ROS position and velocity publications as independent sample rows."""
    samples = []
    for topic, receive, msg in messages(Path(bag), {'/result/velocity', '/result/position'}):
        if topic == '/result/velocity':
            samples.append(Sample(msg['stamp_ns'], receive, msg['velocity_raw']))
        else:
            samples.append(Sample(msg['stamp_ns'], receive, position=msg['position'], velocity_present=False))
    return samples


def write_candidate(path, samples):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('stamp_ns', 'receive_ns', 'velocity_mps', 'distance_m', 'position_valid', 'x', 'y', 'z', 'velocity_present'))
        for sample in samples:
            writer.writerow((sample.stamp_ns, sample.receive_ns, sample.velocity_mps, sample.distance_m,
                             int(sample.position is not None), *(sample.position or (math.nan,) * 3), int(sample.velocity_present)))


def error_metrics(errors, unit):
    n = len(errors)
    return {
        'n': n,
        f'rmse_{unit}': math.sqrt(sum(error * error for error in errors) / n) if n else None,
        f'mae_{unit}': sum(abs(error) for error in errors) / n if n else None,
        f'bias_{unit}': sum(errors) / n if n else None,
        f'max_abs_{unit}': max(map(abs, errors)) if n else None,
    }


def gaps(samples, gap_threshold_ns=100_000_000):
    stamps = [sample.stamp_ns for sample in samples]
    ordered = sorted(set(stamps))
    differences = [b - a for a, b in zip(ordered, ordered[1:])]
    return {
        'first_stamp_ns': min(stamps) if stamps else None,
        'last_stamp_ns': max(stamps) if stamps else None,
        'span_s': (ordered[-1] - ordered[0]) / 1e9 if ordered else 0.,
        'max_gap_s': max(differences, default=0) / 1e9,
        'gap_threshold_s': gap_threshold_ns / 1e9,
        'gaps_over_threshold': sum(value > gap_threshold_ns for value in differences),
        'duplicate_stamps': len(stamps) - len(ordered),
        'backward_stamps_in_receive_order': sum(b < a for a, b in zip(stamps, stamps[1:])),
    }


def match_report(samples, pairs, reference_count):
    offsets = [abs(sample.stamp_ns - reference.stamp_ns) for sample, reference in pairs]
    unique = len({reference.stamp_ns for _, reference in pairs})
    return {
        'outputs': len(samples), 'matched': len(pairs), 'unmatched': len(samples) - len(pairs),
        'coverage': len(pairs) / len(samples) if samples else 0.,
        'unique_reference_matches': unique,
        'reference_coverage': unique / reference_count if reference_count else 0.,
        'mean_abs_match_offset_ms': sum(offsets) / len(offsets) / 1e6 if offsets else None,
        'max_abs_match_offset_ms': max(offsets) / 1e6 if offsets else None,
        'gaps': gaps(samples), 'matched_gaps': gaps([sample for sample, _ in pairs]),
    }


def evaluate(candidate, reference, baseline, tolerance_ns=50_000_000):
    if tolerance_ns < 0:
        raise ValueError('tolerance_ns must be nonnegative')
    index = ReferenceIndex(reference)
    velocity = [sample for sample in candidate if sample.velocity_present]
    position = [sample for sample in candidate if sample.position is not None]
    velocity_pairs, position_pairs = [], []
    for samples, pairs, valid in (
        (velocity, velocity_pairs, lambda sample: math.isfinite(sample.velocity_mps)),
        (position, position_pairs, lambda sample: finite_position(sample.position)),
    ):
        for sample in samples:
            ref = index.nearest(sample.stamp_ns, tolerance_ns)
            if valid(sample) and ref is not None and valid(ref):
                pairs.append((sample, ref))
    reference_count = len(set(index.stamps))
    velocity_errors = [sample.velocity_mps - ref.velocity_mps for sample, ref in velocity_pairs]
    velocity_report = {**match_report(velocity, velocity_pairs, reference_count),
                       'invalid': sum(not math.isfinite(sample.velocity_mps) for sample in velocity),
                       **error_metrics(velocity_errors, 'mps')}
    position_errors = [tuple(a - b for a, b in zip(sample.position, ref.position)) for sample, ref in position_pairs]
    norms = [math.sqrt(sum(error * error for error in errors)) for errors in position_errors]
    last_pair = max(range(len(position_pairs)), key=lambda i: (position_pairs[i][0].stamp_ns, position_pairs[i][0].receive_ns)) if position_pairs else None
    position_report = {
        **match_report(position, position_pairs, reference_count),
        'invalid': sum(not finite_position(sample.position) for sample in position),
        'rmse_3d_m': error_metrics(norms, 'm')['rmse_m'],
        'max_3d_m': max(norms) if norms else None,
        'end_error_3d_m': norms[last_pair] if last_pair is not None else None,
        'end_stamp_ns': position_pairs[last_pair][0].stamp_ns if last_pair is not None else None,
        **{axis: error_metrics([error[i] for error in position_errors], 'm') for i, axis in enumerate('xyz')},
    }
    baseline_by_stamp = {sample.stamp_ns: sample for sample in baseline if math.isfinite(sample.speed_mps)}
    common = [(sample, ref, baseline_by_stamp[sample.stamp_ns]) for sample, ref in velocity_pairs if sample.stamp_ns in baseline_by_stamp]
    flags = Counter(f'{key}={value}' for sample in candidate for key, value in sample.flags.items())
    return {
        'matching': {'method': 'nearest_reference_header_stamp', 'tolerance_ns': tolerance_ns,
                     'boundary': 'inclusive', 'ties': 'earlier_reference_stamp', 'reference_reuse': True,
                     'equivalent_to_ros_approximate_time_synchronizer': False,
                     'reference_velocity_field': 'twist.twist.linear.x'},
        'reference': {'outputs': len(reference), 'unique_stamps': reference_count, 'gaps': gaps(reference)},
        'candidate': {'rows': len(candidate), 'velocity': velocity_report, 'position': position_report, 'flag_counts': dict(flags)},
        'common_velocity': {
            'mask': 'finite candidate and reference velocity, exact baseline output stamp, nearest reference within tolerance',
            'baseline_outputs': len(baseline), 'matched': len(common),
            'candidate': error_metrics([sample.velocity_mps - ref.velocity_mps for sample, ref, _ in common], 'mps'),
            'baseline': error_metrics([base.speed_mps - ref.velocity_mps for _, ref, base in common], 'mps'),
        },
    }


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def provenance(bag, files=(), config=None):
    paths = sorted(set(Path(path).resolve() for path in files))
    bag_files = sorted(Path(bag).glob('*.db3'))
    if (Path(bag) / 'metadata.yaml').is_file():
        bag_files.append(Path(bag) / 'metadata.yaml')
    def git(*args):
        result = subprocess.run(['git', *args], cwd=ROOT, text=True, capture_output=True)
        return result.stdout.strip() if result.returncode == 0 else None
    return {
        'git_commit': git('rev-parse', 'HEAD'),
        'git_dirty': bool(git('status', '--porcelain', '--untracked-files=normal')),
        'bag': str(Path(bag).resolve()),
        'bag_files_sha256': {path.name: sha256(path) for path in bag_files},
        'artifacts_sha256': {str(path): sha256(path) for path in paths},
        'config_path': str(Path(config).resolve()) if config else None,
        'config_text': Path(config).read_text(encoding='utf-8') if config else None,
        'command': sys.argv,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path, help='Bag directory or ID under --dataset-root')
    parser.add_argument('--dataset-root', type=Path, default=ROOT / 'dataset' / 'data')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--candidate', type=Path)
    group.add_argument('--recorded-bag', type=Path, help='ROS recording containing /result/velocity and /result/position')
    parser.add_argument('--candidate-out', type=Path, help='Write normalized CSV, particularly useful with --recorded-bag')
    parser.add_argument('--out', type=Path, help='JSON metrics report')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--artifact', action='append', type=Path, default=[], help='Map, table, executable or other provenance artifact; repeatable')
    parser.add_argument('--export-input', type=Path, help='Write permitted C/F/R/MF/RF/MV/RV replay rows without a header')
    parser.add_argument('--tolerance-ms', type=float, default=50.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.tolerance_ms) or args.tolerance_ms < 0:
        parser.error('--tolerance-ms must be finite and nonnegative')
    bag = args.bag if args.bag.is_dir() else args.dataset_root / args.bag
    events, reference, inputs = read_bag(bag)
    if args.export_input:
        args.export_input.parent.mkdir(parents=True, exist_ok=True)
        with args.export_input.open('w', newline='', encoding='utf-8') as stream:
            csv.writer(stream).writerows(inputs)
    if not args.candidate and not args.recorded_bag:
        if not args.export_input:
            parser.error('provide --candidate, --recorded-bag or --export-input')
        return 0
    candidate = read_candidate(args.candidate) if args.candidate else read_recorded_outputs(args.recorded_bag)
    if args.candidate_out:
        write_candidate(args.candidate_out, candidate)
    report = evaluate(candidate, reference, causal_wheel_baseline(events), round(args.tolerance_ms * 1e6))
    artifacts = list(args.artifact) + [Path(__file__), ROOT / 'analysis/viewer/rosbag_cdr.py', ROOT / 'evaluation/causal_baseline.py']
    if args.config:
        artifacts.append(args.config)
    if args.candidate:
        artifacts.append(args.candidate)
    if args.recorded_bag:
        artifacts.extend(args.recorded_bag.glob('*.db3'))
    report['provenance'] = provenance(bag, artifacts, args.config)
    output = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(output, encoding='utf-8')
    print(output, end='')
    return 0 if reference and candidate else 1


if __name__ == '__main__':
    raise SystemExit(main())
