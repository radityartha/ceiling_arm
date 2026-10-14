#!/usr/bin/env python3
"""G63 session wrapper around scripts/reach_dwell_probe.py (same CLI + the --g63-* options below).

Adds, without editing any shared file:
  * the CAM-6DOF margin guard: a PLANNED trajectory whose (RNEA peak + offset) / limit > 97 % is REFUSED;
  * the payload URDF (boards with mass + collision box) for torque, GOAL-MISMATCH FK, same-gantry, cross-gantry and
    env screens (g63_screens.install + verify -> fail loud if a board is missing from any screen);
  * --g63-tool-quat x,y,z,w: the target orientation (tilted board poses). The reach_dwell monitor and GOAL-MISMATCH
    score the approach axis = +Z of THIS quaternion. Requires --approach 0 (the target IS the tool pose);
  * a hard check, before the first screen, that the boards are attached in MoveIt's planning scene (g63_attach.py);
    missing -> the screen returns None -> the probe REFUSES, exactly like an unavailable screen.
--move still needs one explicit operator approval per invocation (handoff §2).

    probe_g63.py --g63-payload-urdf P.urdf [--g63-boards A] [--g63-tool-quat x,y,z,w] <reach_dwell_probe args>
"""
import importlib.util
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))


def _pop(argv, flag, nargs=1):
    if flag not in argv:
        return None
    i = argv.index(flag)
    vals = argv[i + 1:i + 1 + nargs]
    if len(vals) != nargs or any(v.startswith('--') for v in vals):
        raise SystemExit(f'{flag} butuh {nargs} nilai')
    del argv[i:i + 1 + nargs]
    return vals if nargs > 1 else vals[0]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def install_quat(probe, quat):
    orig = probe.Probe.command_pose

    def command_pose(self, p):
        out = orig(self, p)
        if self.approach != 0.0:
            raise SystemExit('--g63-tool-quat butuh --approach 0 (target = pose tool)')
        o = out.pose.orientation
        o.x, o.y, o.z, o.w = quat
        return out
    probe.Probe.command_pose = command_pose


def install_attach_check(probe, boards):
    import g63_attach
    orig = probe.screen_env
    state = {}

    def screen_env(traj, node, *a, **k):
        if 'ok' not in state:
            try:
                ids = g63_attach.attached_ids(node)
            except Exception as e:                          # noqa: BLE001
                node.get_logger().error(f'G63: cek papan di planning scene gagal: {e} -- TOLAK')
                return None
            want = {g63_attach.object_id(b) for b in boards}
            state['ok'] = want <= set(ids)
            if not state['ok']:
                node.get_logger().error(f'G63: papan {sorted(want - set(ids))} TIDAK terpasang di planning scene '
                                        f'({ids}) -- MoveIt merencana buta papan. Jalankan g63_attach.py. TOLAK')
            else:
                node.get_logger().info(f'G63: papan terpasang di scene {ids}')
        if not state['ok']:
            return None
        return orig(traj, node, *a, **k)
    probe.screen_env = screen_env


def main():
    argv = sys.argv[1:]
    payload = _pop(argv, '--g63-payload-urdf')
    if not payload:
        raise SystemExit('--g63-payload-urdf wajib (URDF dengan papan; g63_urdf.py)')
    boards = ['A', 'B']
    if '--g63-boards' in argv:
        i = argv.index('--g63-boards')
        j = i + 1
        while j < len(argv) and not argv[j].startswith('--'):
            j += 1
        boards = argv[i + 1:j]
        del argv[i:j]
    quat = _pop(argv, '--g63-tool-quat')
    sys.argv = [sys.argv[0]] + argv

    sys.path.insert(0, str(ROOT / 'scripts'))
    probe = _load('g63_shared_probe', ROOT / 'scripts/reach_dwell_probe.py')
    guard = _load('g63_cam6dof_guard', ROOT / 'docs/results/p1_cam6dof/harness/probe_cam6dof.py')
    import g63_screens
    g63_screens.install(payload, probe, boards)
    rep = g63_screens.verify(boards)
    print(f'G63: penyaring memakai {os.path.abspath(payload)}; papan di penyaring {rep}', flush=True)
    guard.install_margin_guard(probe)
    install_attach_check(probe, boards)
    if quat:
        q = [float(v) for v in quat.split(',')]
        if len(q) != 4 or abs(sum(v * v for v in q) - 1) > 1e-3:
            raise SystemExit('--g63-tool-quat x,y,z,w harus quaternion satuan')
        install_quat(probe, q)
    return probe.main()


if __name__ == '__main__':
    sys.exit(main())
