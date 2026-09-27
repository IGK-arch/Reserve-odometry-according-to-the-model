import importlib.util
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'evaluation'))
from reference_benchmark import Sample


class CompareRunsTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('compare_reference_runs'), 'paired run comparator must exist')
        import compare_reference_runs
        self.compare = compare_reference_runs.compare

    def test_velocity_and_position_use_separate_identical_masks(self):
        truth = [Sample(stamp, stamp, 2., (0., 0., 0.)) for stamp in (100, 200, 300)]
        before = [Sample(100, 100, 3., (3., 0., 0.)), Sample(200, 200, 8., (0., 4., 0.))]
        after = [Sample(100, 100, 2.), Sample(200, 200, math.nan, (0., 2., 0.)), Sample(300, 300, 99., (99., 0., 0.))]
        report = self.compare(before, after, truth)
        self.assertEqual(report['velocity']['mask']['matched'], 1)
        self.assertEqual(report['velocity']['mask']['first_stamp_ns'], 100)
        self.assertEqual(report['velocity']['before']['rmse_mps'], 1.)
        self.assertEqual(report['velocity']['after']['rmse_mps'], 0.)
        self.assertEqual(report['position']['mask']['matched'], 1)
        self.assertEqual(report['position']['mask']['first_stamp_ns'], 200)
        self.assertEqual(report['position']['before']['rmse_3d_m'], 4.)
        self.assertEqual(report['position']['after']['rmse_3d_m'], 2.)
        self.assertEqual(report['position']['after']['end_error_xyz_m'], (0., 2., 0.))

    def test_separate_ros_rows_merge_per_channel_and_latest_receive_wins(self):
        before = [Sample(100, 20, 9.), Sample(100, 30, position=(3., 0., 0.), velocity_present=False),
                  Sample(100, 10, 999.), Sample(100, 40, 2.),
                  Sample(100, 35, position=(1., 0., 0.), velocity_present=False)]
        after = [Sample(100, 50, 1.), Sample(100, 60, position=(0., 0., 0.), velocity_present=False)]
        report = self.compare(before, after, [Sample(100, 1, 1., (0., 0., 0.))])
        self.assertEqual(report['velocity']['before']['rmse_mps'], 1.)
        self.assertEqual(report['position']['before']['rmse_3d_m'], 1.)
        self.assertEqual(report['before_counts']['velocity']['published_rows'], 3)
        self.assertEqual(report['before_counts']['velocity']['duplicate_rows'], 2)
        self.assertEqual(report['before_counts']['position']['duplicate_rows'], 1)
        self.assertEqual(report['velocity']['mask']['before_coverage'], 1.)
        self.assertEqual(report['position']['mask']['after_coverage'], 1.)

    def test_nonfinite_latest_publication_and_outside_tolerance_are_excluded_symmetrically(self):
        before = [Sample(100, 10, 2., (0., 0., 0.)), Sample(100, 20, math.inf, (0., 0., math.nan)),
                  Sample(200_000_000, 30, 2., (0., 0., 0.))]
        after = [Sample(100, 40, 1., (0., 0., 0.)), Sample(200_000_000, 50, 1., (0., 0., 0.))]
        report = self.compare(before, after, [Sample(100, 1, 1., (0., 0., 0.))])
        for channel in ('velocity', 'position'):
            self.assertEqual(report[channel]['mask']['shared_stamps'], 2)
            self.assertEqual(report[channel]['mask']['shared_finite_stamps'], 1)
            self.assertEqual(report[channel]['mask']['matched'], 0)
            self.assertEqual(report[channel]['mask']['outside_tolerance'], 1)
            self.assertEqual(report['before_counts'][channel]['invalid_unique_stamps'], 1)
            self.assertEqual(report[channel]['mask']['before_coverage'], 0.)
        self.assertIsNone(report['velocity']['before']['rmse_mps'])
        self.assertIsNone(report['position']['after']['rmse_3d_m'])


if __name__ == '__main__':
    unittest.main()
