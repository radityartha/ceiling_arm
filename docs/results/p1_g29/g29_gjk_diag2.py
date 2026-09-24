"""B1b impact: OLD S18 as used G22-G28 (CrossGantryChecker as-is: pseudo-hull, one reused GeometryData,
screen_trajectory) vs TRUTH (RotCrossChecker: true hulls, arm pairs only via `only`), same trajectories as
g29_gjk_diag.py (seed 2929): arm_1 REST -> IK target in the shared strip."""
import json
import numpy as np
import g29_rot_screen as G
from interarm_collision import CrossGantryChecker, _ik

rng = np.random.default_rng(2929)
old, new = CrossGantryChecker(urdf=G.URDF), G.RotCrossChecker()
rows = []
for k in range(200):
    l1, l2 = rng.uniform(0.2, 1.4), rng.uniform(0.0, 1.6)
    st = G.rest_state(l1, 0.0, l2, 0.0)
    q0 = old.q_from(st)
    tgt = np.array([rng.uniform(l1 - 0.6, l1 + 0.6), rng.uniform(-0.35, 0.25), rng.uniform(1.0, 1.75)])
    q1, ok = _ik(old.model, old.data, 't1_a1_', tgt, q0)
    if not ok:
        continue
    qs = [q0 + (q1 - q0) * 0.5 * (1 - np.cos(np.pi * j / 60)) for j in range(1, 61)]
    so = min(old.check(q, 't1_a1_')[0] for q in qs)          # old: warm, reused across ALL trajectories
    sn = min(new.check(q, 't1_a1_')[0] for q in qs)
    rows.append(dict(old=so, true=sn, vo=G._verdict(so, 0.05), vt=G._verdict(sn, 0.05)))
e = np.array([r['old'] - r['true'] for r in rows])
fl = [(r['vt'], r['vo'], round(r['true'] * 1000, 1), round(r['old'] * 1000, 1)) for r in rows if r['vt'] != r['vo']]
near = np.array([r['old'] - r['true'] for r in rows if r['true'] < 0.10])
vt = {}
for r in rows:
    vt[r['vt']] = vt.get(r['vt'], 0) + 1
print(f'{len(rows)} lintasan: lama - benar maks {e.max()*1000:+.2f} / min {e.min()*1000:+.2f} mm, median {np.median(e)*1000:+.2f}; '
      f'benar < 100 mm: {len(near)} (lama - benar maks {near.max()*1000:+.2f}); verdict benar {vt}')
print(f'verdict beda (benar -> lama): {len(fl)}: {fl}')
json.dump(dict(rows=rows, flips=fl), open('g29_gjk_diag2.json', 'w'), indent=1, default=float)
