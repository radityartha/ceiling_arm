#!/usr/bin/env python3
"""TAS-RAK luring: posisi basis arm_3/arm_4 vs (g2 rel, rot); jarak xy ke tas + jarak basis lengan pasangan ke basis arm_1/arm_2
(g1 1.40). Hanya geometri basis (URDF live2), BUKAN penyaring -- kandidat untuk mock pose_to_g/probe."""
import itertools
import json
import sys

import numpy as np
import pinocchio as pin

U = 'docs/results/p1_g63/g63b/hw_20261016/live2.urdf'
BAG = np.array([0.98, -0.83])
m = pin.buildModelFromUrdf(U); d = m.createData()
iq = lambda j: m.idx_qs[m.getJointId(j)]


def bases(lin, rot):
    q = pin.neutral(m); q[iq('t1_linear_joint')] = 1.40; q[iq('t2_linear_joint')] = lin
    jr = m.joints[m.getJointId('t2_rotation_joint')]
    if jr.nq == 2:
        q[jr.idx_q:jr.idx_q + 2] = [np.cos(np.radians(rot)), np.sin(np.radians(rot))]
    else:
        q[jr.idx_q] = np.radians(rot)
    pin.forwardKinematics(m, d, q); pin.updateFramePlacements(m, d)
    return {a: d.oMf[m.getFrameId(f)].translation[:2].copy() for a, f in
            (('arm_1', 't1_a1_base_link'), ('arm_2', 't1_a2_base_link'), ('arm_3', 't2_a1_base_link'), ('arm_4', 't2_a2_base_link'))}


rows = []
for lin, rot in itertools.product(np.arange(0.0, 1.601, 0.05), np.arange(-35, 95.1, 5)):
    b = bases(lin, rot)
    for arm, other in (('arm_3', 'arm_4'), ('arm_4', 'arm_3')):
        dbag = np.linalg.norm(b[arm] - BAG)
        dsep = min(np.linalg.norm(b[x] - b[y]) for x in ('arm_3', 'arm_4') for y in ('arm_1', 'arm_2'))
        rows.append(dict(lin=round(float(lin), 2), rot=int(rot), arm=arm, d_bag=round(float(dbag), 3),
                         min_base_sep_g1=round(float(dsep), 3), base=np.round(b[arm], 3).tolist()))
rows.sort(key=lambda r: r['d_bag'])
for r in rows[:15]:
    print(r)
json.dump(rows, open('docs/results/p1_tas_rak/cand_geom.json', 'w'))
print('rot 0 lin 0 sanity', {k: np.round(v, 3).tolist() for k, v in bases(0, 0).items()})
print('rot 90 lin 0', {k: np.round(v, 3).tolist() for k, v in bases(0, 90).items()})
