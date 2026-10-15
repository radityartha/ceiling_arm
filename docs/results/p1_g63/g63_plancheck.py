#!/usr/bin/env python3
"""G63 T6: chained PLAN-ONLY check of the locked pose order (mock domain 77 or HW plan-only). NEVER moves.

Execution sequence per arm (§A2/§A3): REST -> R0 -> P0 poses -> R0 -> T1 poses -> R0 -> T2 -> R0 -> T3 -> R0 -> REST.
Each leg is planned by probe_g63.py --plan-check from the PREVIOUS leg's goal (--start-joints), so every leg that
will really be executed is screened (torque 97 %, same/cross-gantry, env, GOAL-MISMATCH, boards attached).
The first leg starts from the PLACED REST state (rails from the pose file), not the measured one.

    g63_plancheck.py --poses poses.json --payload P.urdf --arm arm_2 --out pc_arm_2.json [--stages P0 T1] [--extra-env S.npz]
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from g63_common import ARM_PREFIX  # noqa: E402
from g63_poses import REST  # noqa: E402

FILTER = ('UserWarning', 'buildGeom', 'self.geom')


def sequence(poses, stages):
    r0 = next(p for p in poses if p['role'] == 'R0')
    seq = [r0]
    for st in stages:
        seq += [p for p in poses if p['stage'] == st] + [r0]
    return seq


def rest_goal(arm, cell_tool):
    pre = ARM_PREFIX[arm]
    return dict(id='REST', role='REST', stage='-', goal_joints={f'{pre}joint_{k + 1}': v for k, v in enumerate(REST)},
                tool_xyz=cell_tool[0], tool_quat_xyzw=cell_tool[1])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--poses', required=True)
    ap.add_argument('--payload', required=True)
    ap.add_argument('--arm', required=True, choices=['arm_2', 'arm_3'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--stages', nargs='+', default=['P0', 'T1', 'T2', 'T3'])
    ap.add_argument('--attempts', type=int, default=8)
    ap.add_argument('--extra-env', default=None, help='probe_g63 --g63-extra-env (S0c voxels, A8.15)')
    a = ap.parse_args()
    P = json.load(open(a.poses))
    poses = P['arms'][a.arm]['poses']
    # REST tool pose from the payload URDF (tool frame unaffected by the board)
    import pinocchio as pin
    from scipy.spatial.transform import Rotation
    m = pin.buildModelFromUrdf(a.payload)
    d = m.createData()
    q = pin.neutral(m)
    pre = ARM_PREFIX[a.arm]
    g = pre[:2]
    q[m.joints[m.getJointId(f'{g}_linear_joint')].idx_q] = P['rail1'] if g == 't1' else P['rail2']
    for k, v in enumerate(REST):
        q[m.joints[m.getJointId(f'{pre}joint_{k + 1}')].idx_q] = v
    pin.framesForwardKinematics(m, d, q)
    M = d.oMf[m.getFrameId(pre + 'tool_frame')]
    rest = rest_goal(a.arm, (np.array(M.translation).tolist(), Rotation.from_matrix(M.rotation).as_quat().tolist()))
    seq = sequence(poses, a.stages) + [rest]
    logdir = os.path.splitext(a.out)[0] + '_logs'
    os.makedirs(logdir, exist_ok=True)
    # first leg ALSO from a placed state (REST, rails from the pose file): independent of where the mock's rail is
    results, start = [], dict(rest['goal_joints'])
    for k, p in enumerate(seq):
        cmd = [sys.executable, '-u', os.path.join(HERE, 'probe_g63.py'), '--g63-payload-urdf', a.payload,
               '--g63-tool-quat', ','.join(str(v) for v in p['tool_quat_xyzw']), '--arms', a.arm,
               '--target=' + ','.join(str(v) for v in p['tool_xyz']),   # '=' form: x may be negative
               '--goal-joints', json.dumps(p['goal_joints']),   # HW 10-15: was swallowed by the comment (eec3a4e)
               '--approach', '0', '--plan-check', '--tau-max', '14', '--plan-attempts', str(a.attempts),
               '--trials', '1', '--out-json', os.path.join(logdir, f'{k:02d}_{p["id"]}.json')]
        if a.extra_env:
            cmd += ['--g63-extra-env', a.extra_env]
        if start is not None:   # FULL placed state: rails, rotations, every arm (others REST) + this arm's goal
            full = {'t1_linear_joint': P['rail1'], 't2_linear_joint': P['rail2'], 't1_rotation_joint': 0.0,
                    't2_rotation_joint': 0.0}
            for ap_, pp in ARM_PREFIX.items():
                full.update({f'{pp}joint_{i + 1}': v for i, v in enumerate(REST)})
            full.update(start)
            cmd += ['--start-joints', json.dumps(full)]
        t0 = time.time()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        log = '\n'.join(ln for ln in (r.stdout + r.stderr).splitlines() if not any(f in ln for f in FILTER))
        open(os.path.join(logdir, f'{k:02d}_{p["id"]}.log'), 'w').write(log)
        verdict = 'PLANNED' if 'rencana: PLANNED' in log else next(
            (ln.split('rencana:')[-1].strip() for ln in log.splitlines() if 'rencana:' in ln), f'rc {r.returncode}')
        # LAST margin line = the plan that was accepted (earlier attempts may have been refused by the 97 % guard)
        margin = next((ln.split('margin:')[-1].strip() for ln in reversed(log.splitlines()) if 'margin:' in ln), '')
        n_guard = sum('MARGIN=TOLAK' in ln for ln in log.splitlines())
        env = next((ln.split('minimum')[-1].strip() for ln in log.splitlines() if 'peta lingkungan minimum' in ln), '')
        res = dict(k=k, id=p['id'], role=p['role'], stage=p['stage'], verdict=verdict, margin=margin, env=env,
                   guard_refusals=n_guard,
                   rc=r.returncode, s=round(time.time() - t0, 1))
        results.append(res)
        print(json.dumps(res), flush=True)
        if verdict == 'PLANNED':
            start = p['goal_joints']             # next leg starts from this goal (planned, not measured)
        # a refused pose is skipped on HW too: the next leg starts from the last PLANNED goal
    ok = sum(r['verdict'] == 'PLANNED' for r in results)
    refused = sorted({r['id'] for r in results if r['verdict'] != 'PLANNED'})
    out = dict(arm=a.arm, poses=a.poses, payload=a.payload, stages=a.stages, n=len(results), planned=ok,
               refused_ids=refused, results=results)
    json.dump(out, open(a.out, 'w'), indent=1)
    print(f'{a.arm}: PLANNED {ok}/{len(results)} -> {a.out}')
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
