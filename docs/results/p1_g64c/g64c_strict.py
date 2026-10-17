"""G64-C A0.3 (docs/p1_g64c.md): the two screens return_rest does NOT run, for fold / unfold / retract-to-T1 lines.

  torque  every point, every arm in `names`: oracle2 static gravity + OFFSET + MARGIN <= LIM (= the scheduler's e3 /
          g42 _screen rule; MARGIN = g23_calib2.json, the G38 tuple margin)
  box     every point: min distance of the t{g}_ hulls to the grown MoveIt scene boxes >= 10 mm (G41b BOX_MIN)
Env / same-gantry / cross-gantry stay return_rest.plan_retract's own screens. No ROS here.

    check(g, names, pts, placed, env) -> (ok, info)       placed = every CONFIG_JOINTS joint (the held state)
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g24'))
import oracle2 as O2  # noqa: E402

LIM, OFFSET = np.asarray(O2.O.LIM), np.asarray(O2.O.OFFSET)
MARGIN = np.array(json.load(open(os.path.join(R, 'p1_g23/g23_calib2.json')))['margin'])
BOX_MIN = 0.010
ARM_OF = {v: k for k, v in O2.O.PREFIX.items()}            # 't1_a1_' -> 'arm_1'
T1 = json.load(open(os.path.join(R, 'p1_g64/g64_transit_T.json')))['q6']
_BOX = []


def _box():
    if not _BOX:
        sys.path.insert(0, os.path.join(R, 'p1_g40'))
        import g40_make
        _BOX.append(g40_make.BoxChecker())
    return _BOX[0]


def check(g, names, pts, placed, env):
    lin, rot = placed[f't{g}_linear_joint'], placed[f't{g}_rotation_joint']
    tau, worst = -math.inf, None
    for pre in sorted({n[:6] for n in names}):
        arm = ARM_OF[pre]
        ix = [names.index(f'{pre}joint_{i}') for i in range(1, 7)]
        for q in pts:
            ex = O2.gravity(arm, np.array([q[i] for i in ix]), lin, rot) + OFFSET + MARGIN - LIM
            k = int(np.argmax(ex))
            if ex[k] > tau:
                tau, worst = float(ex[k]), f'{arm} j{k + 1}'
    bx, bmin, bgeom = _box(), math.inf, None
    for q in pts:
        j = dict(placed)
        j.update(zip(names, q))
        b, wg = bx.dist(env, env.q_from(j), f't{g}_')
        if b < bmin:
            bmin, bgeom = b, wg
    ok = tau <= 0.0 and bmin >= BOX_MIN
    return ok, dict(tau_excess=round(tau, 4), tau_worst=worst, box_d=round(float(bmin), 5) if bmin != math.inf else None,
                    box_geom=bgeom, ok=ok)
