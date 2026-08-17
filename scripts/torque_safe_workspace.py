#!/usr/bin/env python3
"""How much of the REACHABLE workspace is also TORQUE-SAFE? Offline, no hardware.

docs/p1_g16_hw.md B5 measured the thing this answers: three of step 2's eight
successes needed more than `joint_2`'s 14 N.m rating to arrive. So reachability
and safety are not the same set, and only the first one had ever been mapped.

Method, and its limits:

  * reachable set comes from the capability map (L1, 5 cm) at a MEASURED rail
    position -- the same source reachable_targets.py uses, so the two agree by
    construction rather than by coincidence.
  * for each node, solve IK, then take the GRAVITY torque at that configuration
    via pinocchio. Static only: a full RNEA needs a trajectory, and there is no
    trajectory until MoveIt plans one.
  * that static number is corrected by the SAME +6.6 N.m offset calibrated in
    B5.4, then compared against each joint's own rating.

  The static-only step is the honest weakness. G16 measured static 9.597 vs
  predicted 9.253 (3.6 % low), so statics are trustworthy; but the peaks that
  actually tripped the guard were transients during motion. This map therefore
  reports a set that is NECESSARY but not sufficient for safety -- a pose
  failing here cannot be safe, while a pose passing here may still spike in
  transit. Reported that way rather than as a safety certificate.

    python3 scripts/torque_safe_workspace.py --arm arm_1 --lin 0.550
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'ros2_ws' / 'src' / 'reachability_gng'))

from reachability_gng.capability import (           # noqa: E402
    CapabilityMap, GANTRY_ARM, PARTNER)

ARM_MAP = {'arm_1': (1, GANTRY_ARM[1]), 'arm_2': (1, PARTNER[1]),
           'arm_3': (2, GANTRY_ARM[2]), 'arm_4': (2, PARTNER[2])}
JOINT_PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_',
                'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
JOINT_EFFORT_LIMIT = {1: 10.0, 2: 14.0, 3: 10.0, 4: 7.0, 5: 7.0, 6: 7.0}
TORQUE_MODEL_OFFSET_NM = 6.6


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', default='arm_1', choices=sorted(ARM_MAP))
    ap.add_argument('--lin', type=float, required=True,
                    help='rail position in METRES, as MEASURED')
    ap.add_argument('--rot', type=float, default=0.0)
    ap.add_argument('--urdf', default='/tmp/reach_dwell_live.urdf',
                    help='URDF captured from the running system')
    ap.add_argument('--offset', type=float, default=TORQUE_MODEL_OFFSET_NM)
    a = ap.parse_args()

    import pinocchio as pin

    gantry, arm_key = ARM_MAP[a.arm]
    cm = CapabilityMap.load(f'/tmp/cap_g{gantry}_rail160.npz')
    li = int(np.argmin(abs(cm.lin - a.lin)))
    ri = int(np.argmin(abs(cm.rot - np.radians(a.rot))))

    ok = np.array([xyz for xyz in cm.nodes
                   if cm.reach(arm_key, xyz, tol_i=0)[li, ri]])
    print(f'{a.arm}: {len(ok)} of {len(cm.nodes)} nodes REACHABLE '
          f'at lin={cm.lin[li]:.3f} rot={np.degrees(cm.rot[ri]):+.0f}')

    model = pin.buildModelFromUrdf(a.urdf)
    data = model.createData()
    pre = JOINT_PREFIX[a.arm]
    names = [f'{pre}joint_{i}' for i in range(1, 7)]
    idx = {n: (model.joints[model.getJointId(n)].idx_q,
               model.joints[model.getJointId(n)].idx_v) for n in names}
    tool = model.getFrameId(f'{pre}tool_frame')

    # IK per node, then gravity torque there. Damped least squares on the
    # position residual only: the capability map itself is position-based, so
    # constraining orientation here would reject poses the map calls reachable
    # and make the two disagree.
    def ik(target, q0, iters=200):
        q = q0.copy()
        for _ in range(iters):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            err = target - data.oMf[tool].translation
            if np.linalg.norm(err) < 2e-3:
                return q, True
            J = pin.computeFrameJacobian(model, data, q, tool,
                                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
            Jr = np.zeros((3, 6))
            for c, n in enumerate(names):
                Jr[:, c] = J[:, idx[n][1]]
            dq = Jr.T @ np.linalg.solve(Jr @ Jr.T + 1e-6 * np.eye(3), err)
            for c, n in enumerate(names):
                q[idx[n][0]] += 0.5 * dq[c]
        return q, False

    q0 = pin.neutral(model)
    for n, v in zip(names, [-0.46352, 0.10710, 0.12916,
                            -1.38653, -0.17648, 1.73885]):
        q0[idx[n][0]] = v
    q0[model.joints[model.getJointId(f't{gantry}_linear_joint')].idx_q] = a.lin

    safe, unsafe, nosol = [], [], 0
    worst_counts = {}
    for xyz in ok:
        q, done = ik(np.asarray(xyz, float), q0)
        if not done:
            nosol += 1
            continue
        g = pin.computeGeneralizedGravity(model, data, q)
        frac, worst = 0.0, None
        for n in names:
            j = int(n.split('joint_')[-1])
            f = (abs(g[idx[n][1]]) + a.offset) / JOINT_EFFORT_LIMIT[j]
            if f > frac:
                frac, worst = f, j
        (safe if frac <= 1.0 else unsafe).append(xyz)
        if frac > 1.0:
            worst_counts[worst] = worst_counts.get(worst, 0) + 1

    solved = len(safe) + len(unsafe)
    print(f'IK tidak konvergen pada {nosol} node (dilaporkan, tidak dibuang)')
    if not solved:
        return 1
    print(f'\n  AMAN-TORSI   : {len(safe):5d} / {solved}  '
          f'({100.0 * len(safe) / solved:.1f} %)')
    print(f'  TIDAK AMAN   : {len(unsafe):5d} / {solved}  '
          f'({100.0 * len(unsafe) / solved:.1f} %)')
    print(f'  sendi pengikat: ' + ', '.join(
        f'joint_{k} {v}x' for k, v in sorted(worst_counts.items())))
    print(f'\n  ambang: gravitasi + {a.offset} N.m vs rating per-sendi '
          f'{JOINT_EFFORT_LIMIT}')
    print('  CATATAN: statis saja -> syarat PERLU, bukan CUKUP. Pose yang lolos '
          'di sini masih bisa melonjak saat transit (B5.4).')
    if len(safe):
        s = np.array(safe)
        print(f'\n  kotak aman: x {s[:,0].min():.2f}..{s[:,0].max():.2f}  '
              f'y {s[:,1].min():.2f}..{s[:,1].max():.2f}  '
              f'z {s[:,2].min():.2f}..{s[:,2].max():.2f}')
        np.save('/tmp/torque_safe_nodes.npy', s)
        print('  -> /tmp/torque_safe_nodes.npy')
    return 0


if __name__ == '__main__':
    sys.exit(main())
