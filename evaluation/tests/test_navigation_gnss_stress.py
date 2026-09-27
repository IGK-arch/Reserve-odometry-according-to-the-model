import tempfile
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'evaluation'))
import navigation_gnss_stress as stress


class StressReplayTests(unittest.TestCase):
    def replay_command(self, vehicle):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'output.csv'
            output.write_text('stamp_ns\n1\n', encoding='utf-8')
            with patch.object(stress.subprocess, 'run') as run:
                stress.replay(Path('/dummy/navigation_replay'), Path('/dummy/input.csv'),
                              output, vehicle=vehicle)
            return run.call_args.args[0]

    def test_30639_command_selects_its_vehicle_configuration(self):
        command = self.replay_command(30639)
        self.assertIn('--vehicle', command)
        self.assertEqual(command[command.index('--vehicle') + 1], '30639')
        self.assertEqual(command[command.index('--gnss-mode') + 1], 'corrections')

    def test_30618_command_keeps_existing_mode_and_assets(self):
        command = self.replay_command(30618)
        self.assertEqual(command[command.index('--vehicle') + 1], '30618')
        self.assertEqual(command[command.index('--gnss-mode') + 1], 'corrections')
        self.assertTrue(command[command.index('--map') + 1].endswith('/assets/route_map.csv'))
        self.assertTrue(command[command.index('--drive-table') + 1].endswith('/assets/drive_accel_table.csv'))

    @staticmethod
    def output_row(stamp, x, frame='mgrs_37UCB', valid='1'):
        return {'stamp_ns': str(stamp), 'x': str(x), 'y': '0', 'z': '0',
                'velocity_mps': '0', 'distance_m': '0', 'position_valid': valid,
                'frame_id': frame, 'mapped': '0' if frame == 'odom' else '1',
                'anchored': '0' if frame == 'odom' else '1', 'clamped': '0',
                'gnss_corrections': '0', 'gnss_rejected': '0', 'branch_switches': '0',
                'branch': 'primary'}

    def test_relative_odometry_is_not_an_absolute_position_measurement(self):
        rows = [self.output_row(1_000_000_000, 0, frame='odom')]
        reference_t = np.array([1_000_000_000, 2_000_000_000], dtype=np.int64)
        reference_p = np.array([[100., 0, 0], [100., 0, 0]])
        result, *_ = stress.score(rows, reference_t, reference_p)
        self.assertEqual(result['n'], 0)
        self.assertIsNone(result['rmse_3d_m'])
        self.assertEqual(result['absolute_position_coverage'], 0)
        self.assertEqual(result['matched_relative_frame_count'], 1)
        self.assertEqual(result['scoring_status'], 'no_absolute_position_matches')

    def test_missing_absolute_outputs_reduce_coverage_instead_of_improving_error(self):
        rows = [self.output_row(1_000_000_000, 105),
                self.output_row(2_000_000_000, 100, frame='odom'),
                self.output_row(3_000_000_000, 100, valid='0')]
        reference_t = np.array([1_000_000_000, 2_000_000_000, 3_000_000_000], dtype=np.int64)
        reference_p = np.array([[100., 0, 0]] * 3)
        result, *_ = stress.score(rows, reference_t, reference_p)
        self.assertEqual(result['n'], 1)
        self.assertEqual(result['rmse_3d_m'], 5.)
        self.assertEqual(result['absolute_position_coverage'], 1/3)
        self.assertEqual(result['proxy_matched_count'], 3)
        self.assertEqual(result['unscored_proxy_matched_count'], 2)

    def test_old_replay_without_frame_metadata_is_rejected(self):
        row = self.output_row(1_000_000_000, 100)
        del row['frame_id']
        with self.assertRaisesRegex(ValueError, 'frame_id'):
            stress.score([row], np.array([1_000_000_000, 2_000_000_000]),
                         np.array([[100., 0, 0]] * 2))

    def test_duplicate_only_timestamps_rejected_before_projection(self):
        inputs = [(1_000_000_001, 1_000_000_000, 'MF', 55., 37., 168., 2)] * 2
        with patch.object(stress, 'project', side_effect=lambda points, *args: np.zeros((len(points), 3))) as project:
            with self.assertRaisesRegex(ValueError, 'distinct MF'):
                stress.scoring_proxy(inputs, Path('/dummy/projection_cli'))
            project.assert_not_called()


if __name__ == '__main__':
    unittest.main()
