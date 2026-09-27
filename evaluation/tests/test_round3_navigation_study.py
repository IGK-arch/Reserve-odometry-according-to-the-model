import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from round3_navigation_study import compare
import test_navigation_gnss_stress as fixtures


class ComparisonTests(unittest.TestCase):
    def test_delayed_anchor_reports_lost_coverage_and_common_error_separately(self):
        row = fixtures.StressReplayTests.output_row
        old = [row(1_000_000_000, 110), row(2_000_000_000, 102)]
        new = [row(1_000_000_000, 0, frame='odom'), row(2_000_000_000, 101)]
        result = compare(old, new, np.array([1_000_000_000, 2_000_000_000]),
                         np.array([[100., 0, 0], [100., 0, 0]]))
        assert result['baseline']['n'] == 2
        assert result['candidate']['n'] == 1
        assert result['lost_absolute_matches'] == 1
        assert result['gained_absolute_matches'] == 0
        assert result['common']['n'] == 1
        assert result['common']['baseline_rmse_m'] == 2
        assert result['common']['candidate_rmse_m'] == 1


    def test_no_common_absolute_outputs_does_not_produce_zero_error(self):
        row = fixtures.StressReplayTests.output_row
        result = compare([row(1, 0)], [row(1, 0, frame='odom')],
                         np.array([1, 2]), np.zeros((2, 3)))
        assert result['common']['n'] == 0
        assert result['common']['candidate_rmse_m'] is None
