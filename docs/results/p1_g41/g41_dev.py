"""G41 A3.4 diagnostic: how far the OMPL task trajectory (joint goal) strays from the straight joint line start -> goal.
dev = max over waypoints of min over t in [0, 1] of max_i |p_i - (s_i + t (q_i - s_i))|, deg. OFFLINE.

    python3 g41_dev.py g41_planonly_plans.jsonl
"""
import json
import math
import sys

import numpy as np


def dev(pts, s, q):
    d = q - s
    t = np.linspace(0.0, 1.0, 401)
    line = s[None, :] + t[:, None] * d[None, :]
    return max(float(np.min(np.max(np.abs(line - p[None, :]), axis=1))) for p in pts)


def main(path):
    rows = []
    for ln in open(path):
        r = json.loads(ln)
        if r.get('task') is None or not r.get('traj'):
            continue
        jn = r['traj']['joint_names']
        pts = np.array(r['traj']['pos'])
        s, q = pts[0], pts[-1]
        start_rest = abs(r['start'][jn[0]] - (-0.46352)) < 1e-3
        rows.append((r['seed'], r['sample'], r['task'], r['arm'], math.degrees(dev(pts, s, q)), start_rest,
                     math.degrees(float(np.max(np.abs(q - s)))), len(pts)))
    for x in rows:
        print(f's{x[0]} sampel {x[1] + 1} t{x[2]} {x[3]}: deviasi {x[4]:.1f} deg (gerak maks {x[6]:.0f} deg, '
              f'{x[7]} titik, start REST {x[5]})')
    d = np.array([x[4] for x in rows])
    if len(d):
        print(f'n {len(d)}: deviasi median {np.median(d):.1f}, p90 {np.percentile(d, 90):.1f}, maks {d.max():.1f} deg; '
              f'<= 5 deg {int((d <= 5).sum())}/{len(d)}')


if __name__ == '__main__':
    main(sys.argv[1])
