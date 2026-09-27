import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'evaluation'))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('ros_runtime_metrics'), 'runtime metrics module must exist')
        import ros_runtime_metrics
        self.metrics = ros_runtime_metrics

    def test_latency_uses_exact_integer_stamp_and_wall_receive_clock(self):
        tracker = self.metrics.LatencyTracker()
        stamp = 1_788_955_800_123_456_789
        tracker.input(stamp, 1_000_000_000, 'C')
        tracker.output('velocity', stamp, 1_003_000_000)
        tracker.output('position', stamp, 1_007_000_000)
        tracker.output('velocity', stamp + 1, 1_009_000_000)
        result = tracker.summary()
        self.assertEqual(result['velocity']['matched'], 1)
        self.assertEqual(result['velocity']['unmatched'], 1)
        self.assertEqual(result['velocity']['p50_ms'], 3)
        self.assertEqual(result['position']['max_ms'], 7)

    def test_reordered_callbacks_do_not_create_negative_latency(self):
        tracker = self.metrics.LatencyTracker()
        tracker.output('velocity', 100, 1_000)
        tracker.input(100, 2_000, 'F')
        tracker.input(200, 10_000, 'R')
        tracker.output('velocity', 200, 15_000)
        result = tracker.summary()['velocity']
        self.assertEqual(result['matched'], 1)
        self.assertEqual(result['reordered'], 1)
        self.assertEqual(result['p50_ms'], .005)

    def test_duplicate_input_stamps_use_latest_prior_callback(self):
        tracker = self.metrics.LatencyTracker()
        tracker.input(100, 1_000, 'F')
        tracker.input(100, 2_000, 'C')
        tracker.output('velocity', 100, 4_000)
        self.assertEqual(tracker.summary()['velocity']['p95_ms'], .002)

    def test_proc_parser_handles_spaces_in_comm_and_one_core_cpu(self):
        # /proc/stat tail starts at field 3; utime/stime/starttime fields 14/15/22.
        fields = ['S'] + ['0'] * 21
        fields[1], fields[11], fields[12], fields[19] = '77', '200', '50', '900'
        stat = '123 (odometry node) ' + ' '.join(fields)
        parsed = self.metrics.parse_proc(stat, 'VmRSS:\t1000 kB\nVmHWM:\t1500 kB\n', 100)
        self.assertEqual(parsed['cpu_seconds'], 2.5)
        self.assertEqual(parsed['rss_kb'], 1000)
        monitor = self.metrics.ProcessMetrics()
        monitor.add(123, 10., parsed)
        parsed = dict(parsed, cpu_seconds=3.0, rss_kb=1200, peak_rss_kb=1600)
        monitor.add(123, 11., parsed)
        report = monitor.summary()
        self.assertEqual(report['memoryPeakKB'], 1600)
        self.assertEqual(report['cpuPercentOneCoreMean'], 50)
        self.assertEqual(report['cpuPercentOneCoreMax'], 50)

    def test_descendant_lookup_matches_symlinked_executable_and_ignores_unrelated_process(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            executable = root / 'build' / 'odometry_node'
            executable.parent.mkdir()
            executable.touch()
            link = root / 'installed_node'
            link.symlink_to(executable)
            for pid, children, command in ((10, '20', '/usr/bin/launch'), (20, '', str(link)), (30, '', str(executable))):
                base = root / str(pid)
                (base / 'task' / str(pid)).mkdir(parents=True)
                (base / 'cmdline').write_bytes(command.encode() + b'\0')
                (base / 'task' / str(pid) / 'children').write_text(children)
            self.assertEqual(self.metrics.find_executable_descendant(10, executable, root), 20)
            self.assertIsNone(self.metrics.find_executable_descendant(40, executable, root))

    def test_process_restart_does_not_create_negative_cpu(self):
        monitor = self.metrics.ProcessMetrics()
        monitor.add(123, 10., {'cpu_seconds': 5., 'rss_kb': 100, 'peak_rss_kb': 200, 'start_ticks': 1})
        monitor.add(124, 11., {'cpu_seconds': 1., 'rss_kb': 200, 'peak_rss_kb': 300, 'start_ticks': 2})
        self.assertIsNone(monitor.summary()['cpuPercentOneCoreMean'])
        self.assertEqual(monitor.summary()['memoryPeakKB'], 300)


if __name__ == '__main__':
    unittest.main()
