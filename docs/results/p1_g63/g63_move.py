#!/usr/bin/env python3
"""G63 HW: ONE leg = probe_g63.py with exactly the arguments g63_plancheck.py planned it with (pose file goal joints,
tool quat, '=' target, payload URDF, S0c extra env, --tau-max 14, --plan-attempts 8), but from the MEASURED state and
with --move unless --dry. One operator approval per invocation (handoff §2).

    g63_move.py --session DIR --poses poses.json --payload P.urdf --extra-env S.npz --arm arm_2 --pose A-R0 [--dry]
    (--pose REST = the REST leg, tool pose from the payload URDF as in g63_plancheck)
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from g63_common import ARM_PREFIX  # noqa: E402
from g63_plancheck import rest_goal  # noqa: E402
from g63_poses import REST  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    for k in ('--session', '--poses', '--payload', '--extra-env', '--pose'):
        ap.add_argument(k, required=True)
    ap.add_argument('--arm', required=True, choices=['arm_2', 'arm_3'])
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()
    P = json.load(open(a.poses))
    if a.pose == 'REST':
        import numpy as np
        import pinocchio as pin
        from scipy.spatial.transform import Rotation
        m = pin.buildModelFromUrdf(a.payload)
        d = m.createData()
        q = pin.neutral(m)
        pre = ARM_PREFIX[a.arm]
        q[m.joints[m.getJointId(f'{pre[:2]}_linear_joint')].idx_q] = P['rail1'] if pre.startswith('t1') else P['rail2']
        for k, v in enumerate(REST):
            q[m.joints[m.getJointId(f'{pre}joint_{k + 1}')].idx_q] = v
        pin.framesForwardKinematics(m, d, q)
        M = d.oMf[m.getFrameId(pre + 'tool_frame')]
        p = rest_goal(a.arm, (np.array(M.translation).tolist(), Rotation.from_matrix(M.rotation).as_quat().tolist()))
    else:
        p = next(x for x in P['arms'][a.arm]['poses'] if x['id'] == a.pose)
    n = len([f for f in os.listdir(a.session) if f.startswith('move_')]) if os.path.isdir(a.session) else 0
    tag = f'move_{n:02d}_{a.arm}_{p["id"]}' + ('_dry' if a.dry else '')
    cmd = [sys.executable, '-u', os.path.join(HERE, 'probe_g63.py'), '--g63-payload-urdf', a.payload,
           '--g63-extra-env', a.extra_env,
           '--g63-tool-quat', ','.join(str(v) for v in p['tool_quat_xyzw']), '--arms', a.arm,
           '--target=' + ','.join(str(v) for v in p['tool_xyz']),
           '--goal-joints', json.dumps(p['goal_joints']),
           '--approach', '0', '--tau-max', '14', '--plan-attempts', '8', '--trials', '1',
           '--out-json', os.path.join(a.session, tag + '.json')]
    cmd += ['--plan-check'] if a.dry else ['--move']
    print(' '.join(cmd), flush=True)
    with open(os.path.join(a.session, tag + '.log'), 'w') as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, text=True)
    log = open(os.path.join(a.session, tag + '.log')).read().splitlines()
    keep = ('rencana:', 'margin', 'minimum', 'ujung rencana', 'torsi', 'MOVED', 'SUCCESS', 'TOLAK', 'DITOLAK', 'REJECT',
            'GOAL-MISMATCH', 'dwell', 'ABORT', 'abort', 'fault', 'FAIL', 'G63:')
    for ln in log:
        if any(k in ln for k in keep) and 'UserWarning' not in ln:
            print(ln[:240])
    print(f'{tag}: rc {r.returncode}')
    return r.returncode


if __name__ == '__main__':
    sys.exit(main())
