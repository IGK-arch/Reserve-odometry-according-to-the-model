#!/usr/bin/env python3
"""Bounded real ROS 2 Humble rosbag smoke for the installed odometry build.

Only startup GNSS fixes from the source bag are retained. No GNSS velocity or
reference topic is played. Scratch copies and reports stay outside the source
repository and do not alter the original bag.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import statistics
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import Resume
from tram_vehicle_msgs.msg import VelocitySensor
import yaml


INPUTS = [
    '/vehicle/front_bogie_velocity',
    '/vehicle/rear_bogie_velocity',
    '/vehicle/driver_position_cmd',
    '/sensing/gnss/master/fix',
    '/sensing/gnss/rover/fix',
]


def quantile(data, p):
    if not data:
        return None
    a = sorted(data)
    i = (len(a) - 1) * p
    lo = math.floor(i)
    hi = math.ceil(i)
    return a[lo] * (hi - i) + a[hi] * (i - lo)


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def make_startup_only_bag(source, scratch):
    scratch.mkdir(parents=True, exist_ok=True)
    source_db = next(source.glob('*.db3'))
    target_db = scratch / source_db.name
    shutil.copy2(source_db, target_db)
    with sqlite3.connect(target_db) as db:
        topics = dict(db.execute('SELECT name, id FROM topics'))
        first = db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
        cutoff = first + 1_500_000_000
        gnss_ids = [topics[x] for x in ('/sensing/gnss/master/fix', '/sensing/gnss/rover/fix')]
        vel_ids = [topics[x] for x in ('/sensing/gnss/master/vel', '/sensing/gnss/rover/vel')]
        params = (*gnss_ids, cutoff, *vel_ids)
        db.execute('DELETE FROM messages WHERE ((topic_id IN (?, ?) AND timestamp > ?) OR topic_id IN (?, ?))', params)
        db.commit()
        counts = dict(db.execute('SELECT topics.name, COUNT(*) FROM messages JOIN topics ON topics.id=messages.topic_id GROUP BY topics.name'))
        first_remaining, last_remaining = db.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
    with (source / 'metadata.yaml').open(encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    info = meta['rosbag2_bagfile_information']
    info['message_count'] = sum(counts.values())
    info['files'][0]['message_count'] = info['message_count']
    info['starting_time']['nanoseconds_since_epoch'] = first_remaining
    info['duration']['nanoseconds'] = last_remaining - first_remaining
    info['files'][0]['starting_time']['nanoseconds_since_epoch'] = first_remaining
    info['files'][0]['duration']['nanoseconds'] = last_remaining - first_remaining
    for topic in info['topics_with_message_count']:
        topic['message_count'] = counts.get(topic['topic_metadata']['name'], 0)
    with (scratch / 'metadata.yaml').open('w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return counts, cutoff


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=8)
    return process.returncode


def run(args):
    out = args.out.resolve()
    if out.exists() and (out / 'summary.json').exists():
        raise RuntimeError('Refusing to overwrite completed smoke run')
    out.mkdir(parents=True, exist_ok=True)
    bag = out / 'startup_only_bag'
    counts, cutoff = make_startup_only_bag(args.bag.resolve(), bag)
    summary = {
        'source_bag': str(args.bag.resolve()),
        'startup_only_bag': str(bag),
        'retained_input_counts': {k: counts.get(k, 0) for k in INPUTS},
        'gnss_cutoff_ns': cutoff,
        'rate': args.rate,
        'wall_limit_s': args.wall_limit,
        'playback_completed': False,
        'status': 'starting',
    }
    rclpy.init()
    observer = rclpy.create_node('final_ros_smoke_observer')
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    velocities, positions, diagnostics = [], [], []

    def on_velocity(msg):
        velocities.append((stamp(msg.header), msg.velocity, time.monotonic()))

    def on_position(msg):
        p = msg.pose.pose.position
        positions.append((stamp(msg.header), p.x, p.y, p.z,
                          msg.twist.twist.linear.x, msg.header.frame_id,
                          msg.child_frame_id, time.monotonic()))

    def on_diagnostic(msg):
        vals = {}
        for status in msg.status:
            vals.update({item.key: item.value for item in status.values})
        diagnostics.append((stamp(msg.header), vals, time.monotonic()))

    observer.create_subscription(VelocitySensor, '/result/velocity', on_velocity, qos)
    observer.create_subscription(Odometry, '/result/position', on_position, qos)
    observer.create_subscription(DiagnosticArray, '/result/diagnostics', on_diagnostic, qos)
    processes = {}
    logs = {}
    resource_samples = []

    def node_resources():
        try:
            log_text = (out / 'node.log').read_text(encoding='utf-8', errors='replace')
            match = re.search(r'process started with pid \[(\d+)\]', log_text)
            if not match:
                return
            pid = int(match.group(1))
            status = Path(f'/proc/{pid}/status').read_text()
            rss = re.search(r'^VmRSS:\s+(\d+) kB', status, re.MULTILINE)
            if not rss:
                return
            stat = Path(f'/proc/{pid}/stat').read_text().split()
            cpu_s = (int(stat[13]) + int(stat[14])) / os.sysconf('SC_CLK_TCK')
            resource_samples.append((time.monotonic(), int(rss.group(1)) / 1024, cpu_s))
        except (OSError, ValueError):
            return
    def start(name, command):
        log = (out / (name + '.log')).open('w', encoding='utf-8')
        logs[name] = log
        processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           stdin=subprocess.DEVNULL, start_new_session=True)
        summary[name + '_command'] = command

    def spin(sec=.05):
        rclpy.spin_once(observer, timeout_sec=sec)

    try:
        start('node', ['ros2', 'launch', 'tram_odometry', 'tram_odometry.launch.py',
                       'vehicle_id:=30618', 'use_sim_time:=true'])
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            spin()
            if processes['node'].poll() is not None:
                raise RuntimeError('odometry launch exited early')
            if all(observer.count_publishers(x) > 0 for x in
                   ('/result/velocity', '/result/position', '/result/diagnostics')):
                break
        else:
            raise TimeoutError('odometry publishers not ready')
        summary['estimator_subscriptions'] = dict(
            observer.get_subscriber_names_and_types_by_node('tram_odometry', '/'))
        start('player', ['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(args.rate),
                         '--start-paused', '--disable-keyboard-controls', '--topics', *INPUTS])
        client = observer.create_client(Resume, '/rosbag2_player/resume')
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            spin()
            if processes['player'].poll() is not None:
                raise RuntimeError('rosbag player exited early')
            if client.service_is_ready() and all(observer.count_subscribers(x) > 0 for x in INPUTS):
                break
        else:
            raise TimeoutError('rosbag subscribers or resume service not ready')
        future = client.call_async(Resume.Request())
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not future.done():
            spin()
        if not future.done() or future.exception():
            raise RuntimeError('could not resume rosbag')
        wall_start = time.monotonic()
        next_resource_sample = wall_start
        while time.monotonic() - wall_start < args.wall_limit:
            spin()
            if time.monotonic() >= next_resource_sample:
                node_resources()
                next_resource_sample += 1
            if processes['node'].poll() is not None:
                raise RuntimeError('odometry node exited during replay')
            if processes['player'].poll() is not None:
                if processes['player'].returncode:
                    raise RuntimeError('rosbag player failed')
                summary['playback_completed'] = True
                break
        summary['replay_wall_s'] = time.monotonic() - wall_start
        for _ in range(20):
            spin()
        summary['status'] = 'completed'
    except Exception as exc:
        summary['status'] = 'failed'
        summary['error'] = repr(exc)
    finally:
        for name in ('player', 'node'):
            if name in processes:
                summary[name + '_returncode'] = stop_process(processes[name])
        for log in logs.values():
            log.close()
        observer.destroy_node()
        rclpy.shutdown()

    latency = []
    reported_rate = []
    branches = {}
    for _, vals, _ in diagnostics:
        try:
            latency.append(float(vals['callback_to_position_publish_ms']))
        except (KeyError, ValueError):
            pass
        try:
            reported_rate.append(float(vals['output_rate_hz_1s']))
        except (KeyError, ValueError):
            pass
        branch = vals.get('route_branch', vals.get('branch', ''))
        if branch:
            branches[branch] = branches.get(branch, 0) + 1
    summary.update({
        'result_counts': {'velocity': len(velocities), 'position': len(positions),
                          'diagnostics': len(diagnostics)},
        'velocity_finite': all(math.isfinite(x[1]) for x in velocities),
        'position_finite': all(all(math.isfinite(v) for v in x[1:5]) for x in positions),
        'velocity_stamp_span_s': velocities[-1][0] - velocities[0][0] if len(velocities) > 1 else 0,
        'position_stamp_span_s': positions[-1][0] - positions[0][0] if len(positions) > 1 else 0,
        'velocity_sim_rate_hz': (len(velocities) - 1) / (velocities[-1][0] - velocities[0][0]) if len(velocities) > 1 and velocities[-1][0] > velocities[0][0] else None,
        'position_sim_rate_hz': (len(positions) - 1) / (positions[-1][0] - positions[0][0]) if len(positions) > 1 and positions[-1][0] > positions[0][0] else None,
        'position_frames': sorted(set(x[5] for x in positions)),
        'child_frames': sorted(set(x[6] for x in positions)),
        'callback_to_position_publish_ms': {'n': len(latency), 'p50': quantile(latency, .5),
                                             'p95': quantile(latency, .95), 'max': max(latency) if latency else None},
        'reported_output_rate_hz_1s': {'n': len(reported_rate), 'p50': quantile(reported_rate, .5),
                                        'p05': quantile(reported_rate, .05)},
        'branches': branches,
        'node_resource_samples': len(resource_samples),
        'node_peak_rss_mib': max((x[1] for x in resource_samples), default=None),
        'node_cpu_average_cores': ((resource_samples[-1][2] - resource_samples[0][2]) /
                                   (resource_samples[-1][0] - resource_samples[0][0]))
                                  if len(resource_samples) > 1 else None,
    })
    with (out / 'observations.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['type', 'stamp_s', 'a', 'b', 'c', 'd', 'frame_id', 'child_frame_id', 'received_mono_s'])
        for x in velocities:
            w.writerow(['velocity', x[0], x[1], '', '', '', '', '', x[2]])
        for x in positions:
            w.writerow(['position', *x])
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary['status'] == 'completed' and velocities and positions and diagnostics else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--rate', type=float, default=1)
    parser.add_argument('--wall-limit', type=float, default=90)
    raise SystemExit(run(parser.parse_args()))
