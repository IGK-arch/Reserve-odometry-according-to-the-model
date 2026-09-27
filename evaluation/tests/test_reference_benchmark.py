import csv
import importlib.util
import math
from pathlib import Path
import sqlite3
import struct
import sys
import subprocess
import json
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'analysis' / 'viewer'))
sys.path.insert(0, str(ROOT / 'evaluation'))
from rosbag_cdr import decode, messages
from causal_baseline import Estimate


class CdrWriter:
    def __init__(self):
        self.data = bytearray(b'\x00\x01\x00\x00')

    def put(self, fmt, value, alignment):
        self.data.extend(b'\0' * (-(len(self.data) - 4) % alignment))
        self.data.extend(struct.pack('<' + fmt, value))

    def string(self, value):
        encoded = value.encode() + b'\0'
        self.put('I', len(encoded), 4)
        self.data.extend(encoded)

    def header(self, stamp=1_600_000_000_123_456_789):
        self.put('i', stamp // 1_000_000_000, 4)
        self.put('I', stamp % 1_000_000_000, 4)
        self.string('map')


def odometry(stamp=1_600_000_000_123_456_789):
    writer = CdrWriter()
    writer.header(stamp)
    writer.string('base_link')
    for value in (1., 2., 3., 0., 0., .6, .8, *range(36), 4., 5., 6., .1, .2, .3, *range(36)):
        writer.put('d', value, 8)
    return bytes(writer.data)


def write_bag(path, rows):
    conn = sqlite3.connect(path)
    conn.executescript('CREATE TABLE topics(id INTEGER, name TEXT, type TEXT); CREATE TABLE messages(id INTEGER, topic_id INTEGER, timestamp INTEGER, data BLOB);')
    conn.execute('INSERT INTO topics VALUES(1,?,?)', ('/localization/kinematic_state', 'nav_msgs/msg/Odometry'))
    conn.executemany('INSERT INTO messages VALUES(?,1,?,?)', [(i, recv, odometry(stamp)) for i, (recv, stamp) in enumerate(rows)])
    conn.commit()
    conn.close()


class CdrTests(unittest.TestCase):
    def test_decodes_reference_odometry_with_integer_nanoseconds(self):
        msg = decode('/localization/kinematic_state', odometry())
        self.assertEqual(msg['stamp_ns'], 1_600_000_000_123_456_789)
        self.assertIsInstance(msg['stamp_ns'], int)
        self.assertEqual(msg['child_frame_id'], 'base_link')
        self.assertEqual(msg['position'], (1., 2., 3.))
        self.assertEqual(msg['orientation'], (0., 0., .6, .8))
        self.assertEqual(msg['linear'], (4., 5., 6.))
        self.assertEqual(msg['angular'], (.1, .2, .3))
        self.assertEqual(len(msg['pose_covariance']), 36)
        self.assertEqual(len(msg['twist_covariance']), 36)

    def test_messages_merge_split_storage_in_global_receive_order(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory)
            write_bag(bag / 'a.db3', [(300, 30), (500, 50)])
            write_bag(bag / 'b.db3', [(100, 10), (400, 40)])
            self.assertEqual([row[1] for row in messages(bag)], [100, 300, 400, 500])


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('reference_benchmark'), 'reference evaluator must exist')
        import reference_benchmark
        self.benchmark = reference_benchmark

    def sample(self, stamp, velocity=0., position=None, receive=None):
        return self.benchmark.Sample(stamp, stamp if receive is None else receive, velocity, position)

    def test_nearest_is_inclusive_at_50_ms_and_chooses_earlier_tie(self):
        samples = [self.sample(100_000_000, 1), self.sample(200_000_000, 2)]
        index = self.benchmark.ReferenceIndex(samples)
        self.assertEqual(index.nearest(150_000_000).velocity_mps, 1)
        self.assertIsNone(index.nearest(250_000_001))

    def test_metrics_use_signed_x_velocity_and_full_xyz_position(self):
        reference = [self.sample(100, -2, (1., 2., 3.)), self.sample(200, 4, (5., 6., 7.))]
        candidate = [self.sample(100, 0, (4., 6., 3.)), self.sample(200, 3, (5., 6., 9.))]
        baseline = [Estimate(100, 100, 1, True, True), Estimate(200, 200, 2, True, True)]
        report = self.benchmark.evaluate(candidate, reference, baseline)
        self.assertAlmostEqual(report['candidate']['velocity']['rmse_mps'], math.sqrt(2.5))
        self.assertEqual(report['candidate']['velocity']['bias_mps'], .5)
        self.assertEqual(report['candidate']['velocity']['max_abs_mps'], 2)
        position = report['candidate']['position']
        self.assertAlmostEqual(position['rmse_3d_m'], math.sqrt(14.5))
        self.assertEqual(position['max_3d_m'], 5)
        self.assertEqual(position['end_error_3d_m'], 2)
        self.assertAlmostEqual(position['x']['rmse_m'], math.sqrt(4.5))
        self.assertEqual(report['common_velocity']['matched'], 2)
        self.assertAlmostEqual(report['common_velocity']['baseline']['rmse_mps'], math.sqrt(6.5))

    def test_common_comparison_masks_missing_baseline_and_nonfinite_values(self):
        reference = [self.sample(100, 1), self.sample(200, 2), self.sample(300, 3)]
        candidate = [self.sample(100, 2), self.sample(200, math.nan), self.sample(300, 4)]
        baseline = [Estimate(100, 100, 0, True, True), Estimate(200, 200, 0, True, True)]
        report = self.benchmark.evaluate(candidate, reference, baseline)
        self.assertEqual(report['candidate']['velocity']['matched'], 2)
        self.assertEqual(report['candidate']['velocity']['outputs'], 3)
        self.assertEqual(report['candidate']['velocity']['invalid'], 1)
        self.assertAlmostEqual(report['candidate']['velocity']['coverage'], 2 / 3)
        self.assertEqual(report['common_velocity']['matched'], 1)
        self.assertEqual(report['common_velocity']['candidate']['rmse_mps'], 1)
        self.assertEqual(report['common_velocity']['baseline']['rmse_mps'], 1)
        self.assertEqual(report['candidate']['position']['matched'], 0)
        self.assertIsNone(report['candidate']['position']['rmse_3d_m'])

    def test_gap_report_includes_missing_matches_and_duplicate_stamps(self):
        rows = [self.sample(100), self.sample(100), self.sample(2_000_000_100)]
        report = self.benchmark.evaluate(rows, [self.sample(100)], [])
        velocity = report['candidate']['velocity']
        self.assertEqual(velocity['matched'], 2)
        self.assertEqual(velocity['unmatched'], 1)
        self.assertEqual(velocity['unique_reference_matches'], 1)
        self.assertEqual(velocity['gaps']['duplicate_stamps'], 1)
        self.assertEqual(velocity['gaps']['max_gap_s'], 2)

    def test_csv_preserves_large_integer_stamp_and_optional_position(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.csv'
            path.write_text('stamp_ns,receive_ns,velocity_mps,position_valid,x,y,z,model_only\n1600000000123456789,1600000000223456789,3,0,nan,nan,nan,1\n1600000000123456790,1600000000223456790,4,1,1,2,3,0\n')
            rows = self.benchmark.read_candidate(path)
            self.assertEqual(rows[0].stamp_ns, 1_600_000_000_123_456_789)
            self.assertIsNone(rows[0].position)
            self.assertEqual(rows[1].position, (1., 2., 3.))
            self.assertEqual(rows[0].flags['model_only'], '1')

    def test_recorded_result_streams_remain_independent(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory)
            write_bag(bag / 'data.db3', [])
            connection = sqlite3.connect(bag / 'data.db3')
            connection.execute('INSERT INTO topics VALUES(2,?,?)', ('/result/velocity', 'tram_vehicle_msgs/msg/VelocitySensor'))
            connection.execute('INSERT INTO topics VALUES(3,?,?)', ('/result/position', 'nav_msgs/msg/Odometry'))
            writer = CdrWriter()
            writer.header(123)
            writer.put('d', 9.5, 8)
            connection.execute('INSERT INTO messages VALUES(1,2,200,?)', (bytes(writer.data),))
            connection.execute('INSERT INTO messages VALUES(2,3,300,?)', (odometry(123),))
            connection.commit()
            connection.close()
            rows = self.benchmark.read_recorded_outputs(bag)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0].velocity_mps, 9.5)
            self.assertIsNone(rows[0].position)
            self.assertTrue(math.isnan(rows[1].velocity_mps))
            self.assertEqual(rows[1].position, (1., 2., 3.))
            report = self.benchmark.evaluate(rows, [self.sample(123, 9.5, (1., 2., 3.))], [])
            self.assertEqual(report['candidate']['velocity']['outputs'], 1)
            self.assertEqual(report['candidate']['velocity']['invalid'], 0)
            path = bag / 'candidate.csv'
            self.benchmark.write_candidate(path, rows)
            reloaded = self.benchmark.read_candidate(path)
            self.assertEqual(reloaded[0].receive_ns, 200)
            self.assertEqual(reloaded[1].receive_ns, 300)

    def test_cli_writes_strict_json_with_provenance_and_config_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory)
            write_bag(bag / 'data.db3', [(100, 100)])
            config = bag / 'config.yaml'
            config.write_text('test_config: 1\n')
            candidate = bag / 'candidate.csv'
            candidate.write_text('stamp_ns,velocity_mps\n100,4\n')
            output = bag / 'report.json'
            process = subprocess.run([sys.executable, str(ROOT / 'evaluation/reference_benchmark.py'), '--bag', str(bag), '--candidate', str(candidate), '--config', str(config), '--out', str(output)], text=True, capture_output=True)
            self.assertEqual(process.returncode, 0, process.stderr)
            report = json.loads(output.read_text(), parse_constant=lambda value: self.fail(f'nonstandard JSON value {value}'))
            self.assertEqual(report['candidate']['velocity']['rmse_mps'], 0.)
            self.assertEqual(report['provenance']['config_text'], 'test_config: 1\n')
            self.assertEqual(len(report['provenance']['bag_files_sha256']['data.db3']), 64)
            self.assertIn(str(config.resolve()), report['provenance']['artifacts_sha256'])
            self.assertIn('git_commit', report['provenance'])

    def test_csv_rejects_lossy_floating_nanosecond_stamps(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.csv'
            path.write_text('stamp_ns,velocity_mps\n1.600000000123456789e18,3\n')
            with self.assertRaisesRegex(ValueError, ':2:'):
                self.benchmark.read_candidate(path)

    def test_exported_estimator_input_excludes_reference_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory)
            write_bag(bag / 'data.db3', [(100, 100)])
            events, reference, inputs = self.benchmark.read_bag(bag)
            self.assertEqual(events, [])
            self.assertEqual(inputs, [])
            self.assertEqual(reference[0].velocity_mps, 4)


if __name__ == '__main__':
    unittest.main()
