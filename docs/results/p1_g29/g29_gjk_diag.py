"""G29 post-lock finding: pinocchio/coal computeDistances on a REUSED GeometryData returns wrong hull
distances (both directions; g29_gjk_probe.py). Impact on verdicts of trajectory-like screens. OFFLINE.

Reference = fresh GeometryData per waypoint (== mesh on the worst pair, == GJK 5000 iterations on
36 960 pair-points, g29_gjk_probe.log). 'seq' = what screen_trajectory does: one GeometryData, waypoints
in order.  (A) CrossGantryChecker, arm_1 plans REST -> IK target in the shared strip (only='t1_a1_').
(B) InterArmChecker (same gantry, MESH) arm_1 -> targets near arm_2.
"""
import json, sys, time
import numpy as np
import pinocchio as pin
import g29_rot_screen as G
from interarm_collision import CrossGantryChecker, InterArmChecker, _ik

rng = np.random.default_rng(2929)
N = int(sys.argv[1]) if len(sys.argv) > 1 else 200


def mins(c, qs, only, fresh):
    out = []
    for q in qs:
        if fresh:
            c.geom_data = c.geom.createData()
        out.append(c.check(q, only)[0])
    return np.array(out)


def run(kind):
    c = CrossGantryChecker(urdf=G.URDF) if kind == 'cross' else InterArmChecker(urdf=G.URDF, gantry='gantry_1')
    names = [f't1_a1_joint_{i}' for i in range(1, 7)]
    iq = [c.model.joints[c.model.getJointId(n)].idx_q for n in names]
    rows = []
    for k in range(N):
        l1, l2 = rng.uniform(0.2, 1.4), rng.uniform(0.0, 1.6)
        st = G.rest_state(l1, 0.0, l2, 0.0)
        q0 = c.q_from(st)
        if kind == 'cross':
            tgt = np.array([rng.uniform(l1 - 0.6, l1 + 0.6), rng.uniform(-0.35, 0.25), rng.uniform(1.0, 1.75)])
        else:
            tgt = np.array([rng.uniform(l1 - 0.9, l1 + 0.2), rng.uniform(0.0, 0.7), rng.uniform(1.0, 1.75)])
        q1, ok = _ik(c.model, c.data, 't1_a1_', tgt, q0)
        if not ok:
            continue
        qs = [q0 + (q1 - q0) * 0.5 * (1 - np.cos(np.pi * j / 60)) for j in range(1, 61)]
        c.geom_data = c.geom.createData()
        s = mins(c, qs, 't1_a1_' if kind == 'cross' else None, False)
        f = mins(c, qs, 't1_a1_' if kind == 'cross' else None, True)
        vs, vf = G._verdict(s.min(), 0.05), G._verdict(f.min(), 0.05)
        rows.append(dict(seq=float(s.min()), fresh=float(f.min()), vs=vs, vf=vf,
                         pt_err_max=float((s - f).max()), pt_err_min=float((s - f).min())))
    R = rows
    e = np.array([r['seq'] - r['fresh'] for r in R])
    flips = [(r['vf'], r['vs']) for r in R if r['vf'] != r['vs']]
    near = [r for r in R if r['fresh'] < 0.10]
    en = np.array([r['seq'] - r['fresh'] for r in near]) if near else np.zeros(1)
    pe = np.array([max(abs(r['pt_err_max']), abs(r['pt_err_min'])) for r in R])
    print(f'{kind:5s}: {len(R)} lintasan (IK konvergen), min-lintasan seq - segar: maks {e.max()*1000:+.2f} / '
          f'min {e.min()*1000:+.2f} mm; |err| > 0.1 mm pada {int((np.abs(e) > 1e-4).sum())}; '
          f'per-titik |err| maks {pe.max()*1000:.1f} mm')
    print(f'       segar < 100 mm: {len(near)}, err di sana maks {en.max()*1000:+.2f} / min {en.min()*1000:+.2f} mm; '
          f'verdict beda (segar -> seq): {len(flips)} {flips[:8]}')
    verd = {}
    for r in R:
        verd[r['vf']] = verd.get(r['vf'], 0) + 1
    print(f'       verdict segar: {verd}')
    return dict(n=len(R), rows=R, flips=flips)


NS = int(sys.argv[2]) if len(sys.argv) > 2 else 6
out = {}
out['cross'] = run('cross')
N = NS
out['same'] = run('same')
json.dump(out, open('g29_gjk_diag.json', 'w'), indent=1, default=float)
