#!/usr/bin/env python3
"""TAS-RAK luring: semua lengan REST, g1 1.40/0; g2 (rel, rot) grid -> jarak min ke peta env (EnvChecker, margin sama dengan G33)
per stop statis, dan sapuan persegi pose_to_g dari (0,0). Hanya geometri env, BUKAN S28/torsi."""
import json
import math
import sys
import warnings

import numpy as np

sys.path[:0] = ['scripts', 'docs/results/p1_g22', 'docs/results/p1_g29']
warnings.simplefilter('ignore')
import g22_plan as P  # noqa: E402
import g29_rot_screen as R  # noqa: E402
from env_collision import EnvChecker  # noqa: E402

ck = EnvChecker()
st = {f'{p}joint_{j}': P.REST[j - 1] for p in P.PREFIX.values() for j in range(1, 7)}
st.update({'t1_linear_joint': 1.40, 't1_rotation_joint': 0.0, 't2_linear_joint': 0.0, 't2_rotation_joint': 0.0})
J = ['t2_linear_joint', 't2_rotation_joint']
BAG = np.array([0.98, -0.83])
out = []
for lin in np.arange(0.0, 1.601, 0.05):
    for rot in np.arange(-35, 95.1, 5):
        v, d, g, _ = ck.screen_trajectory(J, [(lin, math.radians(rot))], st)
        out.append(dict(lin=round(float(lin), 2), rot=int(rot), verdict=v, d_mm=round(d * 1000, 1), geom=g))
ok = [r for r in out if r['verdict'] == 'CLEAR']
print('stop statis CLEAR: %d / %d' % (len(ok), len(out)))
json.dump(out, open('docs/results/p1_tas_rak/scan_rest_env.json', 'w'))
# arm_4/arm_3 base position for CLEAR stops (2D, gantry pivot at (lin, -0.36), arms +-0.40 along axis rotated by rot)
for r in ok:
    th = math.radians(r['rot'])
    for arm, s in (('arm_3', +1), ('arm_4', -1)):
        b = np.array([r['lin'] + s * 0.40 * math.cos(th), -0.36 + s * 0.40 * math.sin(th)])
        r[arm] = round(float(np.linalg.norm(b - BAG)), 3)
best = sorted(ok, key=lambda r: min(r['arm_3'], r['arm_4']))[:12]
for r in best:
    print(r)
