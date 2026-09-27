#!/usr/bin/env python3
"""Known-truth causal wheel fault experiments; these are synthetic diagnostics.

Trajectories are prescribed independently of Estimator, never fitted to a bag.
The complete input schedule is identical for before/after. Sensor observations
are 10 Hz, controller is 20 Hz, wheel receipt is delayed 40 ms. The seed only
changes measurement noise and receipt jitter. No overall contest score is inferred.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import random
import subprocess

EPOCH_NS = 1_700_000_000_000_000_000
DURATION_S = 90
FAULTS = {
    'clean': (30., 40.),
    'pair_jump_up': (30., 40.),
    'pair_jump_down': (30., 40.),
    'front_jump': (30., 40.),
    'rear_jump': (30., 40.),
    'pair_dropout': (30., 35.),
    'front_dropout': (30., 40.),
    'pair_freeze': (15., 23.),
    'front_freeze': (15., 23.),
    'pair_slow_bias': (27., 43.),
    'front_delay': (30., 40.),
    'pair_jump_long': (25., 45.),
}
EXTENDED_FREEZE_FAULTS = {
    'pair_freeze_brake': (50., 58.),
    'pair_freeze_transition': (42., 50.),
}
ALL_FAULTS = {**FAULTS, **EXTENDED_FREEZE_FAULTS}


def nanoseconds(t):
    return EPOCH_NS + round(t * 1e9)


def seconds(stamp):
    return (stamp - EPOCH_NS) / 1e9


def truth(t, profile):
    """Analytic (velocity, distance, driver notch); no estimator calls."""
    if profile == 'steady':
        return 5., 5. * t, 0
    if profile != 'trip':
        raise ValueError(f'unknown profile: {profile}')
    if t < 5:
        return 0., 0., 0
    if t < 25:
        u = t - 5
        return .5 * u, .25 * u * u, 4
    if t < 45:
        return 10., 100. + 10. * (t - 25), 0
    if t < 65:
        u = t - 45
        return 10. - .5 * u, 300. + 10. * u - .25 * u * u, -5
    return 0., 400., 0


def generate_events(profile, fault, seed, vehicle):
    if fault not in ALL_FAULTS:
        raise ValueError(f'unknown fault: {fault}')
    if vehicle not in (30618, 30639):
        raise ValueError('expected vehicle 30618 or 30639')
    scales = (1.000295, 1.000195) if vehicle == 30618 else (1.003512, 1.003187)
    rng = random.Random(seed)
    start, end = ALL_FAULTS[fault]
    events = []
    frozen = {}
    for i in range(DURATION_S * 20 + 1):
        t = i / 20
        stamp = nanoseconds(t)
        events.append((stamp + 2_000_000, stamp, 'C', float(truth(t, profile)[2])))
    for i in range(DURATION_S * 10 + 1):
        t = i / 10
        stamp = nanoseconds(t)
        for channel, scale in zip(('F', 'R'), scales):
            v = max(0., truth(t, profile)[0] + rng.gauss(0., .008))
            delay_ns = 40_000_000 + rng.randrange(-1_000_000, 1_000_001)
            active = start <= t < end
            if active:
                if fault == 'pair_dropout' or (fault == 'front_dropout' and channel == 'F'):
                    continue
                if fault in ('pair_jump_up', 'pair_jump_long') or (fault == 'front_jump' and channel == 'F') or (fault == 'rear_jump' and channel == 'R'):
                    v += 4.
                elif fault == 'pair_jump_down':
                    v = max(0., v - 4.)
                elif fault in ('pair_freeze', 'pair_freeze_brake',
                               'pair_freeze_transition') or (fault == 'front_freeze' and channel == 'F'):
                    v = frozen.setdefault(channel, v)
                elif fault == 'pair_slow_bias':
                    v += min(2., .25 * (t - start))
                elif fault == 'front_delay' and channel == 'F':
                    delay_ns += 500_000_000
            events.append((stamp + delay_ns, stamp, channel, v * 3.6 / scale))
    return sorted(events, key=lambda r: r[0])


def serialize_events(events):
    return ''.join(f'{recv},{stamp},{code},{value:.17g}\n' for recv, stamp, code, value in events)


def replay(exe, payload, vehicle, table):
    command = [str(exe), str(vehicle)]
    if table is not None:
        command.append(str(table))
    result = subprocess.run(command, input=payload, text=True, capture_output=True,
                            check=True, timeout=120)
    outputs = {}
    for row in csv.DictReader(io.StringIO(result.stdout)):
        stamp = int(row['stamp_ns'])
        if stamp in outputs:
            raise ValueError(f'duplicate output timestamp: {stamp}')
        outputs[stamp] = {'v': float(row['velocity_mps']), 's': float(row['distance_m']),
                          'model_only': bool(int(row['model_only'])),
                          'slip': bool(int(row['front_slip'])) or bool(int(row['rear_slip']))}
    return outputs


def error_metrics(values):
    return {'n': len(values),
            'rmse': math.sqrt(sum(x*x for x in values)/len(values)) if values else None,
            'max_abs': max(map(abs, values)) if values else None}


def score_outputs(rows, expected, profile, fault):
    valid = {t: r for t, r in rows.items() if math.isfinite(r['v']) and math.isfinite(r['s'])}
    stamps = [t for t in expected if t in valid]
    errors = [valid[t]['v']-truth(seconds(t), profile)[0] for t in stamps]
    start, end = ALL_FAULTS[fault]
    during = [e for t, e in zip(stamps, errors) if start <= seconds(t) < end]
    recovery = [e for t, e in zip(stamps, errors) if end <= seconds(t) < end + 5]
    relative_errors = []
    if stamps:
        first = stamps[0]
        s0 = valid[first]['s']
        true0 = truth(seconds(first), profile)[1]
        relative_errors = [valid[t]['s']-s0-(truth(seconds(t), profile)[1]-true0) for t in stamps]
    regained = None
    run = []
    for t, err in zip(stamps, errors):
        if seconds(t) < end:
            continue
        if abs(err) <= .1 and (not run or t-run[-1] <= 60_000_000):
            run.append(t)
        else:
            run = [t] if abs(err) <= .1 else []
        if run and run[-1]-run[0] >= 1_000_000_000:
            regained = seconds(run[0])-end
            break
    return {'expected_outputs': len(expected), 'matched_outputs': len(stamps),
            'missing_outputs': sum(t not in rows for t in expected),
            'invalid_outputs': sum(t in rows and t not in valid for t in expected),
            'coverage': len(stamps)/len(expected) if expected else 0.,
            'speed_rmse_mps': error_metrics(errors)['rmse'],
            'speed_max_abs_mps': error_metrics(errors)['max_abs'],
            'fault_speed': error_metrics(during), 'first_5s_recovery_speed': error_metrics(recovery),
            'end_distance_error_m': relative_errors[-1] if relative_errors else None,
            'distance_rmse_m': error_metrics(relative_errors)['rmse'],
            'recovery_to_0p1_mps_for_1s_s': regained,
            'model_only_fraction': sum(valid[t]['model_only'] for t in stamps)/len(stamps) if stamps else None,
            'slip_fraction': sum(valid[t]['slip'] for t in stamps)/len(stamps) if stamps else None}


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', required=True, type=Path)
    parser.add_argument('--after', required=True, type=Path)
    parser.add_argument('--table', type=Path)
    parser.add_argument('--vehicle', type=int, choices=(30618,30639), default=30618)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7,137,911])
    parser.add_argument('--extended-freeze', action='store_true',
                        help='also score paired freezes during braking and command transitions')
    parser.add_argument('--out', required=True, type=Path)
    args=parser.parse_args()
    if args.out.exists():parser.error('--out must be a new file')
    paths={'before':args.before.resolve(), 'after':args.after.resolve()}
    table=args.table.resolve() if args.table else None
    expected=[nanoseconds(i/20) for i in range(100,DURATION_S*20+1)]
    reports=[]
    faults = list(FAULTS) + (list(EXTENDED_FREEZE_FAULTS) if args.extended_freeze else [])
    for profile in ('steady','trip'):
        for fault in faults:
            for seed in args.seeds:
                payload=serialize_events(generate_events(profile,fault,seed,args.vehicle))
                runs={name:replay(exe,payload,args.vehicle,table) for name,exe in paths.items()}
                common=[t for t in expected if all(t in rows and math.isfinite(rows[t]['v']) and math.isfinite(rows[t]['s']) for rows in runs.values())]
                item={'profile':profile,'fault':fault,'fault_window_s':ALL_FAULTS[fault], 'seed':seed,
                      'input_sha256':hashlib.sha256(payload.encode()).hexdigest(), 'common_outputs':len(common)}
                for name,rows in runs.items():
                    item[name]=score_outputs(rows,expected,profile,fault)
                    item[name]['common_mask_metrics']=score_outputs(rows,common,profile,fault)
                reports.append(item)
            print(profile,fault,'done',flush=True)
    report={'protocol':'Independent analytic ground truth; fixed faults; 20 Hz commands, 10 Hz wheels delayed 40 ms with deterministic noise/jitter; 5 s warmup. Fault timestamps are measurement time. Same input bytes and exact common output times for both versions.',
            'limitations':['Synthetic faults do not measure prevalence or contest robustness score.',
                           'Smooth common bias and frozen wheels at constant actual speed may be unobservable.',
                           'Reported distance is relative to the first scored output at 5 seconds.',
                           'First-5s recovery errors may include model errors beyond a fault boundary.'],
            'vehicle':args.vehicle,'seeds':args.seeds,'extended_freeze':args.extended_freeze,
            'table':str(table) if table else None,
            'provenance':{'script_sha256':file_hash(Path(__file__)), 'executables':{name:{'path':str(path),'sha256':file_hash(path)} for name,path in paths.items()},
                          'table_sha256':file_hash(table) if table else None},'scenarios':reports}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('saved',args.out)


if __name__=='__main__':main()
