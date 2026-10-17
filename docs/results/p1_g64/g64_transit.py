"""G64 0/1: candidate TRANSIT poses T (partial fold) for option A. OFFLINE.
For arm_3 (gantry 2, rot 0 -- yaw-equivariant, the same q6 serves every arm of the same mount side) the tool is lifted
h above its REST position, tool-down, roll free (oracle''' solutions, every branch). Per branch: static gravity
(system bar OFFSET + MARGIN), lowest hull-AABB z of the arm, max horizontal distance of the arm from the gantry rotation
axis (swept radius), interarm distance with the partner at the SAME q6 (both folded) and with the partner at REST.
    python3 g64_transit.py   -> g64_transit.json (Pareto: lowest_z high, radius small, j2 low)
"""
import json
import math
import os
import sys
import warnings

import numpy as np
import pinocchio as pin

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
warnings.simplefilter('ignore')
import g64_diag as D  # noqa: E402

ec, bx = D._env()
from interarm_collision import InterArmChecker  # noqa: E402
ia = InterArmChecker(gantry='gantry_2', margin=0.05)
O2, REST = D.O2, D.REST
g, lin = 2, 0.80
base = D.base_joints(g, lin, 0.0)
q = ec.q_from(base)
pin.framesForwardKinematics(ec.model, ec.data, q)
piv = np.array(ec.data.oMf[ec.model.getFrameId('t2_rotation_link')].translation) if ec.model.existFrame('t2_rotation_link') else None
own3, struct, own4 = D.groups(ec, g, 'arm_3')


def metrics(q6a, q6b):
    j = dict(base)
    j.update({f't2_a1_joint_{k}': float(v) for k, v in enumerate(q6a, 1)})
    j.update({f't2_a2_joint_{k}': float(v) for k, v in enumerate(q6b, 1)})
    oMg = ec.place(ec.q_from(j))
    low = min(ec.world_aabb(i, oMg[i])[0][2] for i in own3)
    rad = 0.0
    for i in own3:
        lo, hi = ec.world_aabb(i, oMg[i])
        for x in (lo[0], hi[0]):
            for y in (lo[1], hi[1]):
                rad = max(rad, math.hypot(x - piv[0], y - piv[1]))
    names = [f't2_a1_joint_{k}' for k in range(1, 7)]
    held = {k: v for k, v in j.items() if k not in names}
    v, d, pair, _ = ia.screen_trajectory(names, [list(map(float, q6a))], held)
    return low, rad, d


tool0 = ec.data.oMf[ec.model.getFrameId('t2_a1_tool_frame')].translation.copy()
low0, rad0, d0 = metrics(REST, REST)
out = [dict(h=0.0, q6=list(REST), j2_raw=float(O2.gravity('arm_3', REST, lin)[1]), lowest_z=round(low0, 3), radius=round(rad0, 3),
            interarm_rest=round(d0, 4))]
print('pivot', piv, 'REST', out[0])
for h in (0.20, 0.25, 0.30, 0.35):
    r = O2.solve(tool0 + [0, 0, h], lin, 'arm_3', tilt=True, envelope=True, keep_solutions=True)
    seen = []
    for (k, q6), tau in zip(r['solutions'], r['solutions_tau']):
        if any(np.max(np.abs(q6 - s)) < 0.15 for s in seen):
            continue
        seen.append(q6)
        if not D.G41_branch_ok(tau):
            continue
        low, rad, d_rest = metrics(q6, REST)
        q6b = q6.copy()
        _, _, d_both = metrics(q6, q6b)
        out.append(dict(h=h, roll_k=int(k), q6=[round(float(v), 5) for v in q6], j2_raw=round(float(tau[1]), 3),
                        lowest_z=round(float(low), 3), radius=round(float(rad), 3), interarm_rest=round(float(d_rest), 4),
                        interarm_both=round(float(d_both), 4)))
json.dump(out, open(os.path.join(HERE, 'g64_transit.json'), 'w'), indent=1)
best = sorted([o for o in out if o['h'] > 0 and o['interarm_both'] >= 0.05 and o['interarm_rest'] >= 0.05],
              key=lambda o: (-o['lowest_z'] + 0.5 * o['radius'], o['j2_raw']))
print(len(out) - 1, 'cabang tersaring torsi; lulus antar-lengan', len(best))
for o in best[:12]:
    print(o)
