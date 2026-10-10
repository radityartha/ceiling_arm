"""G42 A2 K-FK: reach_dwell_probe.goal_pose_error on the saved G41b plan-only task plans (end vs task xyz)."""
import json
import math
import os
import sys
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
import reach_dwell_probe as RP  # noqa: E402

node = NS()
pos, ang, n_bad = [], [], 0
for ln in open(os.path.join(HERE, '../p1_g41/g41b_planonly_plans.jsonl')):
    d = json.loads(ln)
    if d.get('task') is None or not d.get('traj'):
        continue
    tr = NS(joint_trajectory=NS(joint_names=d['traj']['joint_names'],
                                points=[NS(positions=d['traj']['pos'][-1])]))
    x, y, z = d['xyz']
    tgt = NS(pose=NS(position=NS(x=x, y=y, z=z), orientation=NS(x=1.0, y=0.0, z=0.0, w=0.0)))
    e = RP.goal_pose_error(tr, d['arm'], node, tgt, d['start'])
    pos.append(e[0] * 1000)
    ang.append(e[1])
    n_bad += e[0] > RP.GOAL_POS_TOL_M or e[1] > RP.GOAL_AXIS_TOL_DEG
pos, ang = np.array(pos), np.array(ang)
print(f'K-FK: n {len(pos)}; posisi mm median {np.median(pos):.3f} p90 {np.percentile(pos, 90):.3f} maks {pos.max():.3f}; '
      f'sumbu deg median {np.median(ang):.3f} maks {ang.max():.3f}; akan GOAL-MISMATCH {n_bad}/{len(pos)} '
      f'(batas {RP.GOAL_POS_TOL_M * 1000:.1f} mm / {RP.GOAL_AXIS_TOL_DEG} deg)')
