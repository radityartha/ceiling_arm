#!/usr/bin/env python3
"""G63b §A6.1 (operator 2026-10-15): j4 boleh berburu bila gerak papan akibat j4 <= 1.5 mm peak-to-peak.
Batas sudut = 1.5 mm / tuas MAKS (jarak sumbu j4 -> sudut muka yang terlihat) atas SEMUA pose terkunci kedua lengan
(konservatif: satu angka untuk semua pose). Ditulis ke j4_limit.json, dibaca g63b_capture.py.

    python3 g63b_j4_limit.py --poses poses_g63b_v4.json --mount mount_g63b.json
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from g63_common import ARM_PREFIX, T_from_xyzrpy, board_points  # noqa: E402

BOARD_MOTION_MAX_M = 0.0015
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]

ap = argparse.ArgumentParser()
ap.add_argument('--poses', required=True)
ap.add_argument('--mount', required=True)
ap.add_argument('--urdf', default=os.path.join(os.path.dirname(HERE), 'live_urdf_20261011.urdf'))
a = ap.parse_args()
import pinocchio as pin  # noqa: E402
D = json.load(open(a.poses))
mt = json.load(open(a.mount))['A']
m = pin.buildModelFromUrdf(a.urdf)
d = m.createData()
TA = T_from_xyzrpy(*mt['xyz'], *mt['rpy'])
T12 = np.array(D['g63b']['T_12'])
P = np.array([board_points('A', mt['square_m'])[i] for i in range(12)])
lev = []
for arm, A in D['arms'].items():
    pre = ARM_PREFIX[arm]
    for p in A['poses']:
        q = pin.neutral(m)
        base = {'t1_linear_joint': D['rail1'], 't2_linear_joint': D['rail2']}
        for pp in ARM_PREFIX.values():
            base.update({f'{pp}joint_{k + 1}': v for k, v in enumerate(REST)})
        base.update(p['goal_joints'])
        for n, v in base.items():
            q[m.joints[m.getJointId(n)].idx_q] = v
        pin.framesForwardKinematics(m, d, q)
        L = d.oMf[m.getFrameId(pre + mt['link'])]
        J = d.oMi[m.getJointId(pre + 'joint_4')]
        ax = J.rotation[:, 2]
        for c, v in p['cams'].items():
            T = TA if int(v['face']) == 1 else TA @ T12
            W = (L.rotation @ (T[:3, :3] @ P.T + T[:3, 3:4])).T + L.translation
            r = W - J.translation
            lev.append((float(np.linalg.norm(r - np.outer(r @ ax, ax), axis=1).max()), p['id'], c))
lev.sort(reverse=True)
lim = BOARD_MOTION_MAX_M / lev[0][0]
out = dict(board_motion_max_m=BOARD_MOTION_MAX_M, lever_max_m=lev[0][0], lever_max_at=lev[0][1:], j4_pp_max_rad=lim,
           note='operator 2026-10-15 "setuju rekomendasimu" (§A6.1 ii); P0 A-02 G63 j4 berburu pp 3.2e-3 rad')
json.dump(out, open(os.path.join(HERE, 'j4_limit.json'), 'w'), indent=1)
print(out)
