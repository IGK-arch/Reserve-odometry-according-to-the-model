import importlib.util
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'evaluation'))


class RosRunnerTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('record_ros_outputs'), 'ROS reference runner must exist')
        import record_ros_outputs
        self.runner = record_ros_outputs

    def test_official_log_parser_uses_last_reports_and_keeps_missing_as_null(self):
        report = self.runner.parse_checker_log('''
[INFO] Velocity metrics [m/s]: velocity: RMSE=0.300000, max=0.900000, n=3
[INFO] Position metrics [m]: x: RMSE=1.000000, max=2.000000, n=3, y: RMSE=3.000000, max=4.000000, n=3, z: RMSE=nan, max=0.000000, n=0, distance: RMSE=4.000000, max=5.000000, n=3
[INFO] Velocity metrics [m/s]: velocity: RMSE=0.200000, max=0.900000, n=4
''')
        self.assertEqual(report['velocity']['n'], 4)
        self.assertEqual(report['velocity']['rmse'], .2)
        self.assertEqual(report['position']['distance']['max'], 5)
        self.assertIsNone(report['position']['z']['rmse'])
        self.assertEqual(self.runner.parse_checker_log(''), {'velocity': None, 'position': {}})

    def test_incomplete_recording_returns_error_without_losing_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / 'unfinished.db3').touch()
            result = self.runner.safe_recorded_counts(Path(directory))
            self.assertEqual(result['counts'], {})
            self.assertIn('no such table', result['error'])

    def test_shutdown_summary_rejects_crashes_even_when_logs_have_metrics(self):
        self.assertFalse(self.runner.clean_shutdown({'returncode': 1, 'signals': []}))
        self.assertFalse(self.runner.clean_shutdown({'returncode': 0, 'signals': ['SIGINT', 'SIGTERM']}))
        self.assertTrue(self.runner.clean_shutdown({'returncode': 0, 'signals': ['SIGINT']}))
        self.assertTrue(self.runner.clean_shutdown({'returncode': -signal.SIGINT, 'signals': ['SIGINT']}))
        self.assertFalse(self.runner.clean_shutdown({'returncode': -signal.SIGINT, 'signals': []}))

    def test_sigint_reaches_owned_process_for_final_report(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / 'ready'
            final = Path(directory) / 'final'
            program = '''import pathlib,signal,sys,time
ready,final = map(pathlib.Path,sys.argv[1:])
def stop(signum, frame):
    final.write_text('final report')
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
ready.touch()
while True: time.sleep(.01)
'''
            process = subprocess.Popen([sys.executable, '-c', program, str(ready), str(final)], start_new_session=True)
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(ready.exists())
                result = self.runner.stop_process(process, grace_s=2)
                self.assertEqual(result['returncode'], 0)
                self.assertEqual(result['signals'], ['SIGINT'])
                self.assertEqual(final.read_text(), 'final report')
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    def test_shutdown_escalates_only_after_grace_timeout(self):
        process = subprocess.Popen([sys.executable, '-c', 'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); print("ready", flush=True); time.sleep(30)'], stdout=subprocess.PIPE, text=True, start_new_session=True)
        try:
            self.assertEqual(process.stdout.readline().strip(), 'ready')
            result = self.runner.stop_process(process, grace_s=.05)
            self.assertEqual(result['signals'], ['SIGINT', 'SIGTERM', 'SIGKILL'])
            self.assertEqual(result['returncode'], -signal.SIGKILL)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            process.stdout.close()


if __name__ == '__main__':
    unittest.main()
