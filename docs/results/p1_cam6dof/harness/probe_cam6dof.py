#!/usr/bin/env python3
"""CAM-6DOF: reuse every probe screen; refuse >97% before execution.

Same CLI as scripts/reach_dwell_probe.py. This session wrapper does not
change the shared probe, torque model, offsets, or measured abort limit.
--move still requires one explicit operator approval per invocation.
"""
import importlib.util
import math
from pathlib import Path
import sys


def install_margin_guard(probe):
    original = probe._plan_and_screen

    def screen(arm, p_cmd, node, *args, **kwargs):
        verdict, traj = original(arm, p_cmd, node, *args, **kwargs)
        if verdict != 'PLANNED':
            return verdict, traj
        peak = probe.predict_peak_torque(traj, arm, node)
        expected = {f'{probe.JOINT_PREFIX[arm]}joint_{j}' for j in range(1, 7)}
        if (peak is None or set(peak) != expected
                or any(not math.isfinite(v) or v < 0 for v in peak.values())):
            node.get_logger().error('CAM-6DOF: torsi tidak lengkap/sah; TOLAK')
            return 'TORQUE-UNSCREENED', None
        fractions = {}
        for name, value in peak.items():
            joint = int(name.rsplit('joint_', 1)[1])
            fractions[name] = (value + probe.JOINT_TORQUE_OFFSET_NM[joint]) / probe.JOINT_EFFORT_LIMIT[joint]
        worst = max(fractions, key=fractions.get)
        ratio = fractions[worst]
        node.get_logger().info(f'CAM-6DOF margin: {worst} {ratio * 100:.6f}% (batas 97%)')
        if ratio > 0.97:
            node.get_logger().error('CAM-6DOF MARGIN=TOLAK; sebelum eksekusi')
            return 'TORQUE-UNSAFE', None
        return verdict, traj

    probe._plan_and_screen = screen


def main():
    root = Path(__file__).resolve().parents[4]
    sys.path.insert(0, str(root / 'scripts'))
    spec = importlib.util.spec_from_file_location('cam6dof_shared_probe', root / 'scripts/reach_dwell_probe.py')
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    install_margin_guard(probe)
    return probe.main()


if __name__ == '__main__':
    sys.exit(main())
