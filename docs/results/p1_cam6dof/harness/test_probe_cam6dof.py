"""Negative controls for the session pre-execution guard; no ROS or hardware."""
from types import SimpleNamespace
import unittest
from probe_cam6dof import install_margin_guard


class MarginGuardTest(unittest.TestCase):
    def run_guard(self, raw_j2=5.0, verdict='PLANNED', invalid=None):
        trajectory = object()
        peak = {f't2_a1_joint_{j}': 0.0 for j in range(1, 7)}
        peak['t2_a1_joint_2'] = raw_j2
        if invalid == 'missing':
            del peak['t2_a1_joint_6']
        elif invalid == 'nan':
            peak['t2_a1_joint_6'] = float('nan')
        elif invalid == 'negative':
            peak['t2_a1_joint_6'] = -0.1
        calls = []
        def predict(*args):
            calls.append(args)
            return peak
        probe = SimpleNamespace(
            _plan_and_screen=lambda *a, **k: (verdict, trajectory if verdict == 'PLANNED' else None),
            predict_peak_torque=predict,
            JOINT_PREFIX={'arm_3': 't2_a1_'},
            JOINT_TORQUE_OFFSET_NM={1: 6.6, 2: 7.7, 3: 6.6, 4: 1.98, 5: 1.98, 6: 1.98},
            JOINT_EFFORT_LIMIT={1: 10., 2: 14., 3: 10., 4: 7., 5: 7., 6: 7.})
        logger = SimpleNamespace(info=lambda *a: None, error=lambda *a: None)
        node = SimpleNamespace(get_logger=lambda: logger)
        install_margin_guard(probe)
        result = probe._plan_and_screen('arm_3', None, node)
        return result, trajectory, calls

    def test_clear_plan_preserves_exact_trajectory(self):
        result, trajectory, calls = self.run_guard()
        self.assertEqual(result, ('PLANNED', trajectory))
        self.assertEqual(len(calls), 1)

    def test_rounded_97_percent_still_refuses_above_exact_limit(self):
        result, _, _ = self.run_guard(raw_j2=5.9)
        self.assertEqual(result, ('TORQUE-UNSAFE', None))

    def test_just_below_limit_is_allowed(self):
        result, trajectory, _ = self.run_guard(raw_j2=5.8799999)
        self.assertEqual(result, ('PLANNED', trajectory))

    def test_existing_collision_refusal_is_preserved(self):
        result, _, calls = self.run_guard(verdict='ENV-COLLIDE')
        self.assertEqual(result, ('ENV-COLLIDE', None))
        self.assertEqual(calls, [])

    def test_invalid_prediction_refuses(self):
        for invalid in ('missing', 'nan', 'negative'):
            with self.subTest(invalid=invalid):
                result, _, _ = self.run_guard(invalid=invalid)
                self.assertEqual(result, ('TORQUE-UNSCREENED', None))


if __name__ == '__main__':
    unittest.main()
