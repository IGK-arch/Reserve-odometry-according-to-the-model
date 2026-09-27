import importlib.util
from pathlib import Path
import sys
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'wheel_fault_benchmark.py'
spec = importlib.util.spec_from_file_location('wheel_fault_benchmark', MODULE)
b = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = b
spec.loader.exec_module(b)


class WheelFaultBenchmarkTest(unittest.TestCase):
    def test_trajectory_has_exact_integral(self):
        self.assertEqual(b.truth(0, 'trip'), (0.0, 0.0, 0))
        self.assertAlmostEqual(b.truth(25, 'trip')[0], 10)
        self.assertAlmostEqual(b.truth(25, 'trip')[1], 100)
        self.assertAlmostEqual(b.truth(45, 'trip')[1], 300)
        self.assertAlmostEqual(b.truth(65, 'trip')[1], 400)
        self.assertAlmostEqual(b.truth(80, 'trip')[1], 400)

    def test_fault_only_changes_target_channel_during_window(self):
        clean=b.generate_events('trip', 'clean', 7, 30618)
        fault=b.generate_events('trip', 'front_jump', 7, 30618)
        self.assertEqual(len(clean),len(fault))
        changed=0
        for a,c in zip(clean,fault):
            self.assertEqual(a[:3],c[:3])
            t=b.seconds(a[1])
            if a[2]=='F' and 30 <= t < 40:
                changed+=1
                self.assertAlmostEqual(c[3]-a[3],4*3.6/1.000295)
            else:self.assertEqual(a,c)
        self.assertEqual(changed,100)

    def test_drop_removes_only_wheels_and_is_deterministic(self):
        clean=b.generate_events('trip','clean',7,30618)
        fault=b.generate_events('trip','pair_dropout',7,30618)
        expected=[r for r in clean if not(r[2] in ('F','R') and 30 <= b.seconds(r[1]) < 35)]
        self.assertEqual(fault,expected)
        self.assertEqual(fault,b.generate_events('trip','pair_dropout',7,30618))
        self.assertEqual(fault,sorted(fault,key=lambda r:r[0]))

    def test_freeze_preserves_payload_but_advances_header(self):
        rows=b.generate_events('trip','pair_freeze',7,30618)
        for channel in ('F','R'):
            bad=[r for r in rows if r[2]==channel and 15 <= b.seconds(r[1]) < 23]
            self.assertEqual(len({r[3] for r in bad}),1)
            self.assertEqual(len({r[1] for r in bad}),80)

    def test_missing_and_nonfinite_outputs_cannot_improve_coverage(self):
        expected=[b.nanoseconds(5+i*.05) for i in range(41)]
        result=b.score_outputs({},expected,'steady','clean')
        self.assertEqual(result['coverage'],0)
        self.assertIsNone(result['speed_rmse_mps'])
        rows={t:{'v':5.,'s':5*b.seconds(t),'model_only':False,'slip':False} for t in expected}
        rows[expected[0]]['v']=float('nan')
        result=b.score_outputs(rows,expected,'steady','clean')
        self.assertEqual(result['invalid_outputs'],1)
        self.assertLess(result['coverage'],1)

    def test_relative_distance_removes_startup_offset(self):
        expected=[b.nanoseconds(5+i*.05) for i in range(41)]
        rows={t:{'v':5.,'s':5*b.seconds(t)+100,'model_only':False,'slip':False} for t in expected}
        result=b.score_outputs(rows,expected,'steady','clean')
        self.assertAlmostEqual(result['speed_rmse_mps'],0)
        self.assertAlmostEqual(result['end_distance_error_m'],0)
        self.assertEqual(result['coverage'],1)


if __name__ == '__main__':unittest.main()
