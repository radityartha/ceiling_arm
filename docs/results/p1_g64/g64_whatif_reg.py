"""G64 design comparison (docs/p1_g64_joint_plan.md 1.2): REGRESSION of "always T" (option A forced) on the tasks the
CURRENT system accepted -- every task of g55_selected.json R90sel (8 seeds x 6, P1'' schedules, the G57 HW family).
Per task, with the scheduler's own goal q6 (the branch the runner sends as --goal-joints):
  park   REST -> T line of both arms at p0 (g64_whatif.park_ok)
  stop   both arms at T at the stop: env >= 50 mm, box >= BOX_MIN
  trav   rect sweep PREVIOUS stop (p0 for the first) -> stop, both arms at T, env >= 50 mm (G33 rule)
  e1     task arm at goal, partner at T: env >= 50 mm, box >= BOX_MIN
  line   straight cosine line T -> goal, partner at T (torque/env/box/same-gantry = screen_line_any)
Same screens and margins as g64_whatif (run with G64_BOX_MIN=0.010 = system). Static only. OFFLINE, no ROS.

    G64_BOX_MIN=0.010 python3 g64_whatif_reg.py   -> g64_whatif_reg.json + table
"""
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g64_whatif as W  # noqa: E402

D, PREFIX = W.D, W.PREFIX
PLAN = os.path.join(HERE, '..', 'p1_g55', 'g55_selected.json')
ARM_OF = {'arm1': 'arm_1', 'arm2': 'arm_2', 'arm3': 'arm_3', 'arm4': 'arm_4'}


def jobs():
    out = []
    for row in json.load(open(PLAN)):
        assert row['p0'] == {'1': 1.5, '2': 0.0}, row['p0']
        for g in ('1', '2'):
            prev = D.P0[int(g)]
            for k, st in enumerate(row['R90sel']['schedule'][g]):
                cur = (st['rail_m'], math.radians(st['rot_deg']))
                for t, who in st['tasks'].items():
                    arm = ARM_OF[who]
                    q6 = [st['goal'][t][f'{PREFIX[arm]}joint_{i}'] for i in range(1, 7)]
                    out.append(dict(seed=row['seed'], g=int(g), stop=k, task=int(t), arm=arm, lin=cur[0],
                                    rot_deg=st['rot_deg'], prev=prev, q6=q6))
                prev = cur
    return out


def _job(j):
    import g29_rot_screen as RS
    ec, bx = D._env()
    T = W.T_q6()
    g, arm, lin, rot = j['g'], j['arm'], j['lin'], math.radians(j['rot_deg'])
    partner = [x for x in D.ARMS[g] if x != arm][0]
    r = dict(j)
    r['park'] = W.park_ok(g)
    q = ec.q_from(W.held_with(g, lin, rot, {arm: T, partner: T}))
    d, _ = ec.check(q, only=(D.G4.gp(g),))
    b, _ = bx.dist(ec, q, D.G4.gp(g))
    r['stop_ok'] = bool(d >= D.ENV_MIN and b > 0.0 and b >= W.BOX_MIN)
    r['stop_d'] = round(float(d), 4)
    v, dt, _, _ = ec.screen_trajectory([f't{g}_linear_joint', f't{g}_rotation_joint'],
                                       RS.rect_points(tuple(j['prev']), (lin, rot)),
                                       W.held_with(g, *j['prev'], {arm: T, partner: T}))
    r['trav'], r['trav_d'] = v, round(float(dt), 4)
    q = ec.q_from(W.held_with(g, lin, rot, {arm: np.array(j['q6']), partner: T}))
    d, _ = ec.check(q, only=(D.G4.gp(g),))
    b, _ = bx.dist(ec, q, D.G4.gp(g))
    r['e1'] = bool(d >= D.ENV_MIN and b > 0.0 and b >= W.BOX_MIN)
    ok, why = W.screen_line_any(g, arm, lin, rot, W.cos_line(T, j['q6']), W.held_with(g, lin, rot, {arm: T, partner: T}),
                                ec, bx)
    r['line'], r['line_why'] = ok, why
    r['a_ok'] = r['park'] == 'OK' and r['stop_ok'] and v == 'CLEAR' and r['e1'] and ok
    return r


def main():
    J = jobs()
    with Pool(D.NPROC) as p:
        R = p.map(_job, J, chunksize=1)
    json.dump(R, open(os.path.join(HERE, 'g64_whatif_reg.json'), 'w'), indent=1, default=float)
    n = len(R)
    print(f'BOX_MIN {W.BOX_MIN}: {n} tugas sistem-sekarang; lolos paksa-T {sum(r["a_ok"] for r in R)}/{n}')
    for k in ('park', 'stop_ok', 'trav', 'e1', 'line'):
        bad = [r for r in R if (r[k] != 'OK' if k == 'park' else (r[k] != 'CLEAR' if k == 'trav' else not r[k]))]
        print(f'  gagal {k}: {len(bad)}', sorted({(r["seed"], r["g"], r["stop"]) for r in bad})[:12],
              sorted({r.get("line_why") for r in bad}) if k == 'line' else '')
    by = {}
    for r in R:
        by.setdefault(r['seed'], []).append(r['a_ok'])
    print('  seed utuh paksa-T:', {s: f'{sum(v)}/{len(v)}' for s, v in by.items()})


if __name__ == '__main__':
    main()
