"""Supervisor-only ROS latency and Linux odometry-process resource measurements."""
from __future__ import annotations

from collections import Counter
import csv
import math
import os
from pathlib import Path
import time


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * probability
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


class LatencyTracker:
    """Exact header stamps, latest prior supervisor input callback per stamp.

    This measures observer receive-to-receive latency including DDS and observer
    scheduling. Separate subscribers can reorder callbacks; those are counted,
    never reported as negative latency. It is not hardware timestamp latency.
    """
    def __init__(self):
        self.inputs = {}
        self.pending = {'velocity': {}, 'position': {}}
        self.values = {'velocity': [], 'position': []}
        self.counts = {'velocity': Counter(), 'position': Counter()}

    def input(self, stamp_ns, wall_ns, code):
        self.inputs[stamp_ns] = (wall_ns, code)
        for channel in self.pending:
            pending = self.pending[channel].pop(stamp_ns, [])
            self.counts[channel]['reordered'] += sum(value < wall_ns for value in pending)

    def output(self, channel, stamp_ns, wall_ns):
        self.counts[channel]['outputs'] += 1
        origin = self.inputs.get(stamp_ns)
        if origin is None:
            self.pending[channel].setdefault(stamp_ns, []).append(wall_ns)
        elif wall_ns < origin[0]:
            self.counts[channel]['reordered'] += 1
        else:
            self.values[channel].append((wall_ns - origin[0]) / 1e6)

    def summary(self):
        result = {'method': 'supervisor monotonic receive(C/F/R exact header stamp) to receive(result)',
                  'duplicate_input_stamps': 'latest prior supervisor receive callback',
                  'limitations': 'Independent DDS subscriptions and supervisor scheduling affect this observation; reordered callbacks are excluded.'}
        for channel, values in self.values.items():
            counts = self.counts[channel]
            result[channel] = {'outputs': counts['outputs'], 'matched': len(values),
                               'unmatched': counts['outputs'] - len(values) - counts['reordered'],
                               'reordered': counts['reordered'],
                               'coverage': len(values) / counts['outputs'] if counts['outputs'] else 0.,
                               'p50_ms': percentile(values, .5), 'p95_ms': percentile(values, .95),
                               'max_ms': max(values) if values else None}
        return result


def parse_proc(stat_text, status_text, clock_ticks):
    # comm may contain spaces/parentheses; numeric fields start after its last ).
    fields = stat_text.rsplit(') ', 1)[1].split()
    status = {}
    for line in status_text.splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            status[key] = value.strip().split()
    return {'cpu_seconds': (int(fields[11]) + int(fields[12])) / clock_ticks,
            'start_ticks': int(fields[19]),
            'rss_kb': int(status.get('VmRSS', ['0'])[0]),
            'peak_rss_kb': int(status.get('VmHWM', ['0'])[0])}


class ProcessMetrics:
    def __init__(self):
        self.samples = []
        self.last = None
        self.cpu_seconds = 0.
        self.wall_seconds = 0.

    def add(self, pid, wall_s, values):
        sample = {'wall_monotonic_s': wall_s, 'pid': pid, **values, 'cpu_percent_one_core': None}
        if self.last and (self.last['pid'], self.last['start_ticks']) == (pid, values['start_ticks']):
            elapsed = wall_s - self.last['wall_monotonic_s']
            cpu = values['cpu_seconds'] - self.last['cpu_seconds']
            if elapsed > 0 and cpu >= 0:
                sample['cpu_percent_one_core'] = cpu / elapsed * 100.
                self.cpu_seconds += cpu
                self.wall_seconds += elapsed
        self.samples.append(sample)
        self.last = sample
        return sample

    def summary(self):
        cpu = [row['cpu_percent_one_core'] for row in self.samples if row['cpu_percent_one_core'] is not None]
        return {'samples': len(self.samples), 'pids': sorted({row['pid'] for row in self.samples}),
                'memoryPeakKB': max((row['peak_rss_kb'] for row in self.samples), default=None),
                'rssSamplePeakKB': max((row['rss_kb'] for row in self.samples), default=None),
                'cpuPercentOneCoreMean': self.cpu_seconds / self.wall_seconds * 100 if self.wall_seconds else None,
                'cpuPercentOneCoreP95': percentile(cpu, .95), 'cpuPercentOneCoreMax': max(cpu) if cpu else None,
                'cpuMeasuredSeconds': self.cpu_seconds, 'sampledWallSeconds': self.wall_seconds,
                'scope': 'odometry_node process only; 100% CPU equals one logical core; memory uses /proc VmHWM KiB'}


def find_executable_descendant(parent_pid, executable, proc_root=Path('/proc')):
    """Walk only this launch process's descendant tree, matching executable path."""
    pending, seen = [parent_pid], set()
    expected = str(Path(executable).resolve())
    while pending:
        pid = pending.pop()
        if pid in seen:
            continue
        seen.add(pid)
        base = proc_root / str(pid)
        try:
            cmdline = (base / 'cmdline').read_bytes().split(b'\0')
            if cmdline and str(Path(os.fsdecode(cmdline[0])).resolve()) == expected:
                return pid
            pending.extend(int(value) for value in (base / 'task' / str(pid) / 'children').read_text().split())
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return None


class RuntimeMonitor:
    def __init__(self, observer, outdir, executable, interval_s=.5):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor
        self.latency = LatencyTracker()
        self.process = ProcessMetrics()
        self.executable = executable
        self.launch_pid = None
        self.target_pid = None
        self.interval_s = interval_s
        self.next_sample = 0.
        self.clock_ticks = os.sysconf('SC_CLK_TCK')
        self.errors = Counter()
        self.stream = (Path(outdir) / 'runtime_samples.csv').open('w', newline='', encoding='utf-8')
        self.writer = None
        def stamp(message):
            return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.subscriptions = []
        for topic, kind, code in (
            ('/vehicle/driver_position_cmd', DriverControllerCommand, 'C'),
            ('/vehicle/front_bogie_velocity', VelocitySensor, 'F'),
            ('/vehicle/rear_bogie_velocity', VelocitySensor, 'R'),
        ):
            self.subscriptions.append(observer.create_subscription(kind, topic, lambda msg, code=code: self.latency.input(stamp(msg), time.monotonic_ns(), code), qos_profile_sensor_data))
        for topic, kind, channel in (('/result/velocity', VelocitySensor, 'velocity'), ('/result/position', Odometry, 'position')):
            self.subscriptions.append(observer.create_subscription(kind, topic, lambda msg, channel=channel: self.latency.output(channel, stamp(msg), time.monotonic_ns()), qos_profile_sensor_data))

    def sample(self, force=False):
        now = time.monotonic()
        if not self.launch_pid or (not force and now < self.next_sample):
            return
        self.next_sample = now + self.interval_s
        try:
            if self.target_pid is None:
                self.target_pid = find_executable_descendant(self.launch_pid, self.executable)
            if self.target_pid is None:
                return
            base = Path('/proc') / str(self.target_pid)
            values = parse_proc((base / 'stat').read_text(), (base / 'status').read_text(), self.clock_ticks)
            row = self.process.add(self.target_pid, now, values)
            if self.writer is None:
                self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
                self.writer.writeheader()
            self.writer.writerow(row)
            self.stream.flush()
        except (OSError, ValueError, IndexError) as error:
            self.errors[type(error).__name__] += 1
            self.target_pid = None

    def close(self):
        self.stream.close()
        return {'resources': self.process.summary(), 'latency': self.latency.summary(),
                'sampling_interval_s': self.interval_s, 'sampling_errors': dict(self.errors)}
