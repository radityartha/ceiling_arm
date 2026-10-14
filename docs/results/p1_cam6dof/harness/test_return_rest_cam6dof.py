"""Negative controls for return guards; no ROS node or hardware action."""
import math
from types import SimpleNamespace
import unittest
import return_rest_cam6dof as guarded


class Future:
    def __init__(self, done=True, value=None):
        self.ready, self.value = done, value

    def done(self):
        return self.ready

    def result(self):
        return self.value


class Goal:
    def __init__(self, done=False, error=0):
        self.result_future = Future(done, SimpleNamespace(status=4, result=SimpleNamespace(error_code=error)))
        self.cancels = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return Future()


class SafetyControls(unittest.TestCase):
    def test_fraction_rounded_to_97_still_exceeds_limit(self):
        peak = {f't2_a1_joint_{j}': 0.0 for j in range(1, 7)}
        peak['t2_a1_joint_2'] = .9714 * guarded.probe.JOINT_EFFORT_LIMIT[2] - guarded.probe.JOINT_TORQUE_OFFSET_NM[2]
        worst = max(guarded.margin(peak, 't2_a1_').values())
        self.assertEqual(round(worst * 100), 97)
        self.assertGreater(worst, .97)

    def test_missing_nan_and_negative_model_are_refused(self):
        valid = {f't2_a1_joint_{j}': 0.0 for j in range(1, 7)}
        cases = [None, {}, {**valid, 't2_a1_joint_2': math.nan},
                 {**valid, 't2_a1_joint_2': -1.0}]
        for peak in cases:
            with self.subTest(peak=peak), self.assertRaises(ValueError):
                guarded.margin(peak, 't2_a1_')

    def test_torque_over_14_cancels_even_when_result_already_successful(self):
        goal = Goal(done=True)
        node = SimpleNamespace(tau_peak=14.001, invalid_effort=False)
        verdict, _ = guarded.wait_motion(node, goal, 30, spin=lambda: None, clock=lambda: 0)
        self.assertEqual(verdict, 'TORQUE-ABORT')
        self.assertEqual(goal.cancels, 1)

    def test_exact_14_is_allowed_until_success(self):
        goal = Goal()
        node = SimpleNamespace(tau_peak=14.0, invalid_effort=False)
        verdict, _ = guarded.wait_motion(node, goal, 30, spin=lambda: setattr(goal.result_future, 'ready', True), clock=lambda: 0)
        self.assertEqual(verdict, 'MOVED')
        self.assertEqual(goal.cancels, 0)

    def test_controller_error_is_not_moved(self):
        goal = Goal(done=True, error=1)
        verdict, _ = guarded.wait_motion(SimpleNamespace(tau_peak=1.0, invalid_effort=False), goal, 30, clock=lambda: 0)
        self.assertEqual(verdict, 'EXEC-FAIL')


if __name__ == '__main__':
    unittest.main()
