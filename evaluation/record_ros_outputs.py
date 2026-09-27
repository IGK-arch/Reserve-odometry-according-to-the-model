#!/usr/bin/env python3
"""Run the real ROS node, installed official checker and result recorder.

The shell entry point is tools/ros_reference_check.sh. Outputs include raw bags,
checker/node/player/recorder logs, effective parameters, normalized candidate
CSV, independent offline metrics, and run_summary.json. No reference relay is
started. The checker package must already be installed in the ROS environment.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import sys
import time

from reference_benchmark import INPUT_CODES, REFERENCE, ROOT, main as evaluate_main, provenance
from ros_runtime_metrics import RuntimeMonitor


def parse_checker_log(log):
    """Extract the last official log report, retaining absent/NaN metrics."""
    result = {'velocity': None, 'position': {}}
    pattern = re.compile(r'(velocity|x|y|z|distance): RMSE=([\deE+.\-naif]+), max=([\deE+.\-naif]+), n=(\d+)')
    for line in log.splitlines():
        if 'Velocity metrics [m/s]:' not in line and 'Position metrics [m]:' not in line:
            continue
        for name, rmse, maximum, count in pattern.findall(line):
            metric = {'rmse': float(rmse), 'max': float(maximum), 'n': int(count)}
            metric = {key: (value if not isinstance(value, float) or math.isfinite(value) else None) for key, value in metric.items()}
            if name == 'velocity':
                result['velocity'] = metric
            else:
                result['position'][name] = metric
    return result


def stop_process(process, grace_s=10.):
    """Signal a process group created with start_new_session=True, then reap it."""
    sent = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        sent.append(sig.name)
        try:
            process.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            continue
    return {'returncode': process.wait(), 'signals': sent}


def clean_shutdown(result):
    if any(name in result['signals'] for name in ('SIGTERM', 'SIGKILL')):
        return False
    return result['returncode'] == 0 or (
        'SIGINT' in result['signals'] and result['returncode'] in (-signal.SIGINT, 128 + signal.SIGINT)
    )


def recorded_counts(bag):
    counts = {}
    for path in Path(bag).glob('*.db3'):
        connection = sqlite3.connect(f'{path.resolve().as_uri()}?mode=ro', uri=True)
        try:
            for name, count in connection.execute('SELECT topics.name, COUNT(*) FROM messages JOIN topics ON topics.id=messages.topic_id GROUP BY topics.name'):
                counts[name] = counts.get(name, 0) + count
        finally:
            connection.close()
    return counts


def safe_recorded_counts(bag):
    try:
        return {'counts': recorded_counts(bag), 'error': None}
    except (OSError, sqlite3.Error) as error:
        return {'counts': {}, 'error': f'{type(error).__name__}: {error}'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--outdir', required=True, type=Path, help='New or empty directory; existing results are never overwritten')
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--vehicle-id', type=int, default=30618)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--timeout', type=float, default=1800., help='Maximum wall seconds after playback resumes; timeout is a failed partial run')
    parser.add_argument('--ready-timeout', type=float, default=60.)
    parser.add_argument('--drain-seconds', type=float, default=2.)
    parser.add_argument('--artifact', type=Path, action='append', default=[])
    args = parser.parse_args(argv)
    for name in ('rate', 'timeout', 'ready_timeout', 'drain_seconds'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    bag, outdir, config = args.bag.resolve(), args.outdir.resolve(), args.config.resolve()
    if not (bag / 'metadata.yaml').is_file() or not config.is_file():
        parser.error('bag metadata.yaml and config must exist')
    if outdir.exists() and any(outdir.iterdir()):
        parser.error('--outdir must be new or empty')
    # Import only after argument validation so --help and unit tests need no ROS.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rosbag2_interfaces.srv import Resume
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    checker_spec = importlib.util.find_spec('hackathon_solution_checker.metrics')
    if checker_spec is None:
        parser.error('the official hackathon_solution_checker package is not installed')
    package = Path(get_package_share_directory('tram_odometry'))
    executable = Path(get_package_prefix('tram_odometry')) / 'lib/tram_odometry/odometry_node'
    outdir.mkdir(parents=True, exist_ok=True)
    summary = {'status': 'starting', 'playback_completed': False, 'commands': {}, 'processes': {},
               'domain_id': os.environ.get('ROS_DOMAIN_ID'), 'rate': args.rate, 'vehicle_id': args.vehicle_id,
               'offline_matching_is_official_ats': False}
    processes, logs = {}, {}
    interrupted = []
    def on_signal(signum, frame):
        interrupted.append(signal.Signals(signum).name)
    previous = {sig: signal.signal(sig, on_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    observer = rclpy.create_node(f'reference_run_supervisor_{os.getpid()}')
    started = time.monotonic()
    runtime = None
    def tick():
        rclpy.spin_once(observer, timeout_sec=.1)
        if runtime is not None:
            runtime.sample()
    def launch(name, command):
        summary['commands'][name] = command
        log = (outdir / f'{name}.log').open('w', encoding='utf-8')
        logs[name] = log
        processes[name] = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def spin():
        if interrupted:
            raise InterruptedError(interrupted[-1])
        for name, process in processes.items():
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited before run completion (exit {process.returncode}); see {name}.log')
        tick()
    try:
        artifacts = [config, executable, Path(checker_spec.origin), Path(__file__), ROOT / 'evaluation/ros_runtime_metrics.py', ROOT / 'tools/ros_reference_check.sh']
        artifacts.extend(package.glob('assets/*'))
        artifacts.extend(path.resolve() for path in args.artifact)
        summary['provenance'] = provenance(bag, [path for path in artifacts if path.is_file()], config)
        runtime = RuntimeMonitor(observer, outdir, executable)
        # Refuse a shared result stream; never stop unrelated processes.
        discovery_deadline = time.monotonic() + 1.
        while time.monotonic() < discovery_deadline:
            tick()
        if any(observer.count_publishers(topic) for topic in ('/result/velocity', '/result/position')):
            raise RuntimeError('result publishers already exist in this ROS domain; use a separate ROS_DOMAIN_ID')
        launch('node', ['ros2', 'launch', 'tram_odometry', 'tram_odometry.launch.py', f'config:={config}', f'vehicle_id:={args.vehicle_id}', 'use_sim_time:=true'])
        runtime.launch_pid = processes['node'].pid
        # Direct Python process receives SIGINT and runs MetricsNode.report() in finally.
        launch('checker', [sys.executable, '-m', 'hackathon_solution_checker.metrics', '--ros-args', '-p', 'report_period_sec:=5.0'])
        recorded_bag = outdir / 'result_bag'
        launch('recorder', ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(recorded_bag), '/result/velocity', '/result/position', '/result/diagnostics', REFERENCE, '/clock'])
        launch('player', ['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(args.rate), '--start-paused', '--disable-keyboard-controls', '--topics', *INPUT_CODES, REFERENCE])
        resume = observer.create_client(Resume, '/rosbag2_player/resume')
        deadline = time.monotonic() + args.ready_timeout
        minimum_subscriptions = {'/result/velocity': 3, '/result/position': 3, '/result/diagnostics': 1,
                                 REFERENCE: 3, **{topic: 2 for topic in INPUT_CODES if topic.startswith('/vehicle/')}}
        while time.monotonic() < deadline:
            spin()
            names = set(observer.get_node_names())
            counts = {topic: observer.count_subscribers(topic) for topic in minimum_subscriptions}
            if {'tram_odometry', 'hackathon_solution_checker', 'rosbag2_recorder', 'rosbag2_player'} <= names and all(counts[topic] >= count for topic, count in minimum_subscriptions.items()) and resume.service_is_ready():
                break
        else:
            raise RuntimeError(f'ROS subscriber readiness timed out: {counts}')
        estimator_subs = dict(observer.get_subscriber_names_and_types_by_node('tram_odometry', '/'))
        if REFERENCE in estimator_subs:
            raise RuntimeError('estimator subscribes to scoring reference; refusing to evaluate')
        summary['readiness'] = {'elapsed_s': time.monotonic() - started, 'subscribers': counts,
                                'estimator_subscriptions': estimator_subs}
        parameters = subprocess.run(['ros2', 'param', 'dump', '/tram_odometry'], capture_output=True, text=True, timeout=20, check=True)
        effective_config = outdir / 'effective_parameters.yaml'
        effective_config.write_text(parameters.stdout, encoding='utf-8')
        # Retain hashes of external maps/tables referenced by resolved runtime parameters.
        import yaml
        effective = yaml.safe_load(parameters.stdout) or {}
        external_paths = []
        for node_values in effective.values():
            for name, value in node_values.get('ros__parameters', {}).items():
                if isinstance(value, str) and ('file' in name or 'path' in name) and Path(value).is_file():
                    external_paths.append(Path(value))
        summary['provenance']['effective_parameters'] = provenance(bag, [effective_config, *external_paths])['artifacts_sha256']
        future = resume.call_async(Resume.Request())
        deadline = time.monotonic() + args.ready_timeout
        while not future.done() and time.monotonic() < deadline:
            spin()
        if not future.done() or future.exception() is not None:
            raise RuntimeError('could not resume paused rosbag player')
        play_start = time.monotonic()
        while processes['player'].poll() is None:
            if time.monotonic() - play_start > args.timeout:
                raise TimeoutError(f'playback exceeded {args.timeout} wall seconds; result is partial')
            if interrupted:
                raise InterruptedError(interrupted[-1])
            for name in ('node', 'checker', 'recorder'):
                if processes[name].poll() is not None:
                    raise RuntimeError(f'{name} exited during playback; see {name}.log')
            tick()
        if processes['player'].returncode != 0:
            raise RuntimeError(f'rosbag playback failed: exit {processes["player"].returncode}')
        summary['playback_completed'] = True
        summary['playback_wall_s'] = time.monotonic() - play_start
        # Give DDS and the checker time to consume the final messages.
        deadline = time.monotonic() + args.drain_seconds
        while time.monotonic() < deadline:
            if interrupted:
                raise InterruptedError(interrupted[-1])
            tick()
        summary['status'] = 'completed'
    except Exception as error:
        summary['status'] = 'failed'
        summary['error'] = f'{type(error).__name__}: {error}'
        print(summary['error'], file=sys.stderr)
    finally:
        if runtime is not None:
            runtime.sample(force=True)
            summary['runtime'] = runtime.close()
        # Stop publishers first; checker gets a direct SIGINT for its final report.
        for name in ('player', 'node', 'recorder', 'checker'):
            if name in processes:
                summary['processes'][name] = stop_process(processes[name])
        for log in logs.values():
            log.close()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        summary['interrupted_by'] = interrupted
        summary['wall_s'] = time.monotonic() - started
        log_path = outdir / 'checker.log'
        summary['official_checker'] = parse_checker_log(log_path.read_text(encoding='utf-8', errors='replace') if log_path.exists() else '')
        recorded_bag = outdir / 'result_bag'
        recording = safe_recorded_counts(recorded_bag)
        counts = recording['counts']
        if recording['error']:
            summary['recording_error'] = recording['error']
            summary['status'] = 'failed'
        summary['recorded_message_counts'] = counts
        velocity = summary['official_checker']['velocity']
        position = summary['official_checker']['position'].get('distance')
        summary['official_coverage'] = {
            'velocity_matched_per_recorded_result': velocity['n'] / counts['/result/velocity'] if velocity and counts.get('/result/velocity') else None,
            'position_matched_per_recorded_result': position['n'] / counts['/result/position'] if position and counts.get('/result/position') else None,
            'denominator_note': 'Recorder and checker have independent DDS subscriptions; losses may differ.',
        }
        if not velocity or not velocity['n'] or not position or not position['n']:
            summary['status'] = 'failed'
            summary.setdefault('error', 'official checker has no matched velocity or position samples')
        if any(not clean_shutdown(value) for value in summary['processes'].values()):
            summary['status'] = 'failed'
            summary.setdefault('error', 'a process crashed or required forced shutdown; final reports may be incomplete')
        if (recorded_bag / 'metadata.yaml').is_file():
            try:
                eval_args = ['--bag', str(bag), '--recorded-bag', str(recorded_bag), '--candidate-out', str(outdir / 'candidate.csv'), '--out', str(outdir / 'offline_metrics.json'), '--config', str(config)]
                for artifact in args.artifact:
                    eval_args.extend(('--artifact', str(artifact)))
                if evaluate_main(eval_args):
                    raise RuntimeError('offline evaluation has no reference or result samples')
            except Exception as error:
                summary['offline_error'] = str(error)
                summary['status'] = 'failed'
        else:
            summary['status'] = 'failed'
            summary.setdefault('error', 'recorder did not finalize metadata.yaml')
        (outdir / 'run_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2, allow_nan=False))
    return 0 if summary['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
