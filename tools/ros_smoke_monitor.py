#!/usr/bin/env python3
"""Check a short, live ROS replay against the hackathon output contract."""

import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import signal
import statistics
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor


EXPECTED_TYPES = {
    "/result/velocity": "tram_vehicle_msgs/msg/VelocitySensor",
    "/result/position": "nav_msgs/msg/Odometry",
    "/result/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
}


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def rate_hz(samples, index):
    if len(samples) < 2:
        return 0.0
    elapsed = (samples[-1][index] - samples[0][index]) / 1e9
    return (len(samples) - 1) / elapsed if elapsed > 0 else 0.0


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


class Monitor:
    def __init__(self, node, node_executable, record_samples=False):
        self.node = node
        self.node_executable = os.path.realpath(node_executable)
        self.resource_pids = set()
        self.rss_mb = []
        self.cpu_one_core_pct = []
        self.last_cpu_ticks = {}
        self.clock_ticks_per_s = os.sysconf("SC_CLK_TCK")
        self.input_stamps = set()
        self.velocity = []
        self.position = []
        self.xyz = []
        self.samples = [] if record_samples else None
        self.invalid_stamps = Counter()
        self.invalid_numeric = Counter()
        self.velocity_frames = Counter()
        self.position_frames = Counter()
        self.anchor_sources = Counter()
        self.position_modes = Counter()
        self.callback_ms = []
        self.output_rate_diagnostics = []
        node.create_subscription(
            VelocitySensor, "/vehicle/front_bogie_velocity",
            self.on_input, qos_profile_sensor_data)
        node.create_subscription(
            VelocitySensor, "/vehicle/rear_bogie_velocity",
            self.on_input, qos_profile_sensor_data)
        node.create_subscription(
            DriverControllerCommand, "/vehicle/driver_position_cmd",
            self.on_input, qos_profile_sensor_data)
        node.create_subscription(
            VelocitySensor, "/result/velocity",
            self.on_velocity, qos_profile_sensor_data)
        node.create_subscription(
            Odometry, "/result/position",
            self.on_position, qos_profile_sensor_data)
        node.create_subscription(
            DiagnosticArray, "/result/diagnostics",
            self.on_diagnostics, qos_profile_sensor_data)

    def sample_resources(self):
        """Read the node process counters from procfs without external packages."""
        now = time.monotonic()
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                if os.path.realpath(proc / "exe") != self.node_executable:
                    continue
                status = (proc / "status").read_text(encoding="ascii")
                stat = (proc / "stat").read_text(encoding="ascii")
                values = stat.split(") ", 1)[1].split()
                ticks = int(values[11]) + int(values[12])
                fields = {}
                for line in status.splitlines():
                    if line.startswith(("VmRSS:", "VmHWM:")):
                        key, value = line.split(":", 1)
                        fields[key] = float(value.split()[0]) / 1024.0
            except (OSError, IndexError, ValueError):
                continue
            pid = int(proc.name)
            self.resource_pids.add(pid)
            rss = fields.get("VmHWM", fields.get("VmRSS"))
            if rss is not None:
                self.rss_mb.append(rss)
            last = self.last_cpu_ticks.get(pid)
            if last is not None and now > last[0]:
                cpu_pct = 100.0 * (ticks - last[1]) / (
                    self.clock_ticks_per_s * (now - last[0]))
                if cpu_pct >= 0.0 and math.isfinite(cpu_pct):
                    self.cpu_one_core_pct.append(cpu_pct)
            self.last_cpu_ticks[pid] = (now, ticks)

    def record(self, topic, message, target):
        stamp = stamp_ns(message.header)
        if stamp <= 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000:
            self.invalid_stamps[topic] += 1
        target.append((stamp, time.monotonic_ns()))

    def on_input(self, message):
        self.input_stamps.add(stamp_ns(message.header))

    def on_velocity(self, message):
        self.record("velocity", message, self.velocity)
        if self.samples is not None:
            self.samples.append((len(self.samples), "velocity", stamp_ns(message.header),
                                 time.monotonic_ns(), message.velocity, "", "", ""))
        self.velocity_frames[message.header.frame_id] += 1
        if not math.isfinite(message.velocity) or not 0.0 <= message.velocity <= 30.0:
            self.invalid_numeric["velocity"] += 1

    def on_position(self, message):
        self.record("position", message, self.position)
        self.position_frames[message.header.frame_id] += 1
        p = message.pose.pose.position
        if self.samples is not None:
            self.samples.append((len(self.samples), "position", stamp_ns(message.header),
                                 time.monotonic_ns(), message.twist.twist.linear.x,
                                 p.x, p.y, p.z))
        q = message.pose.pose.orientation
        self.xyz.append((p.x, p.y, p.z))
        numbers = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                   message.twist.twist.linear.x)
        if not all(math.isfinite(number) for number in numbers):
            self.invalid_numeric["position"] += 1

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != "tram_odometry":
                continue
            values = {item.key: item.value for item in status.values}
            if "anchor_source" in values:
                self.anchor_sources[values["anchor_source"]] += 1
            if "position_mode" in values:
                self.position_modes[values["position_mode"]] += 1
            for key, target in (("callback_to_position_publish_ms", self.callback_ms),
                                ("output_rate_hz_1s", self.output_rate_diagnostics)):
                try:
                    value = float(values[key])
                except (KeyError, ValueError):
                    continue
                if math.isfinite(value):
                    target.append(value)

    def report(self, mode, vehicle_id):
        discovered = dict(self.node.get_topic_names_and_types())
        types = {topic: discovered.get(topic, []) for topic in EXPECTED_TYPES}
        velocity_stamps = {record[0] for record in self.velocity}
        position_stamps = {record[0] for record in self.position}
        results = {
            "mode": mode,
            "vehicle_id": vehicle_id,
            "topic_types": types,
            "input_unique_stamps": len(self.input_stamps),
            "velocity_count": len(self.velocity),
            "position_count": len(self.position),
            "velocity_rate_wall_hz": rate_hz(self.velocity, 1),
            "position_rate_wall_hz": rate_hz(self.position, 1),
            "velocity_rate_header_hz": rate_hz(self.velocity, 0),
            "position_rate_header_hz": rate_hz(self.position, 0),
            "velocity_input_stamp_matches": len(velocity_stamps & self.input_stamps),
            "position_velocity_stamp_matches": len(position_stamps & velocity_stamps),
            "velocity_frames": dict(self.velocity_frames),
            "position_frames": dict(self.position_frames),
            "first_position_stamp_ns": self.position[0][0] if self.position else None,
            "first_position_xyz": self.xyz[0] if self.xyz else None,
            "position_xyz_min": [min(point[i] for point in self.xyz) for i in range(3)]
                if self.xyz else None,
            "position_xyz_max": [max(point[i] for point in self.xyz) for i in range(3)]
                if self.xyz else None,
            "anchor_sources": dict(self.anchor_sources),
            "position_modes": dict(self.position_modes),
            "invalid_stamps": dict(self.invalid_stamps),
            "invalid_numeric": dict(self.invalid_numeric),
            "callback_to_publish_ms_p50": statistics.median(self.callback_ms)
                if self.callback_ms else None,
            "callback_to_publish_ms_p95": percentile(self.callback_ms, 0.95),
            "callback_to_publish_ms_p99": percentile(self.callback_ms, 0.99),
            "callback_to_publish_ms_max": max(self.callback_ms)
                if self.callback_ms else None,
            "reported_rate_hz_p50": statistics.median(self.output_rate_diagnostics)
                if self.output_rate_diagnostics else None,
            "node_resource_pids": sorted(self.resource_pids),
            "node_resource_samples": len(self.rss_mb),
            "node_peak_rss_mb": max(self.rss_mb) if self.rss_mb else None,
            "node_cpu_one_core_mean_pct": statistics.mean(self.cpu_one_core_pct)
                if self.cpu_one_core_pct else None,
            "node_cpu_one_core_p95_pct": percentile(self.cpu_one_core_pct, 0.95),
        }
        failures = []
        for topic, expected in EXPECTED_TYPES.items():
            if expected not in types[topic]:
                failures.append(f"{topic} must have type {expected}; saw {types[topic]}")
        if len(self.velocity) < 100 or len(self.position) < 100:
            failures.append("fewer than 100 messages on a required output topic")
        if self.invalid_stamps or self.invalid_numeric:
            failures.append("invalid output timestamp or numeric value")
        if any(results[key] < 10.0 for key in (
            "velocity_rate_wall_hz", "position_rate_wall_hz",
            "velocity_rate_header_hz", "position_rate_header_hz")):
            failures.append("required output rate below 10 Hz")
        if len(velocity_stamps & self.input_stamps) < 20:
            failures.append("too few result velocity stamps exactly match input stamps")
        if len(position_stamps & velocity_stamps) < 20:
            failures.append("too few position stamps exactly match velocity stamps")
        if self.velocity_frames.get("base_link", 0) != len(self.velocity):
            failures.append("/result/velocity header.frame_id is not always base_link")
        expected_frame = "mgrs_37UCB" if mode == "anchored" else "odom"
        if self.position_frames.get(expected_frame, 0) != len(self.position):
            failures.append(f"/result/position frame is not always {expected_frame}")
        if mode == "anchored":
            if not (self.anchor_sources.get("master", 0) or
                    self.anchor_sources.get("rover_fallback", 0)):
                failures.append("no startup GNSS anchor reported")
            if not self.position_modes.get("route_map", 0):
                failures.append("route map mode absent")
        else:
            if self.anchor_sources.get("master", 0) or self.anchor_sources.get("rover_fallback", 0):
                failures.append("GNSS anchor reported despite filtered GNSS")
            if not self.position_modes.get("relative_straight", 0):
                failures.append("relative position mode absent")
        if not self.callback_ms:
            failures.append("callback-to-publish diagnostics absent")
        elif results["callback_to_publish_ms_p95"] > 100.0:
            failures.append("p95 callback-to-position-publication exceeded 100 ms")
        results["failures"] = failures
        results["passed"] = not failures
        return results

    def write_samples(self, path):
        if self.samples is None:
            raise RuntimeError("Sample recording was not enabled")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("sequence", "topic", "stamp_ns", "receive_monotonic_ns",
                             "velocity_mps", "x_m", "y_m", "z_m"))
            writer.writerows(self.samples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("anchored", "no-gnss"), required=True)
    parser.add_argument("--vehicle-id", type=int, required=True)
    parser.add_argument("--node-executable", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--samples", type=Path,
                        help="Optional CSV of every observed output for numeric replay audit")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("tram_odometry_smoke_monitor")
    monitor = Monitor(node, args.node_executable, args.samples is not None)
    stopped = False

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        next_resource_sample = 0.0
        while not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_resource_sample:
                monitor.sample_resources()
                next_resource_sample = now + 0.25
    finally:
        result = monitor.report(args.mode, args.vehicle_id)
        if args.samples is not None:
            monitor.write_samples(args.samples)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
