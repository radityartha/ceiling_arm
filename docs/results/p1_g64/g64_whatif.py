"""G64 task 1 input: WHAT-IF option A (certified transit pose T) on the g64_diag tuples. OFFLINE, no ROS.

The current system = REST at the stop, REST traverse, straight joint line target <-> REST (g64_diag CURRENT).
Option A replaces REST by T (both arms of the moving gantry) while the gantry moves and as the arm's start/return:
  A_PARK  at the parking pose p0 both arms at T: env >= 50 mm, box-free; and the REST -> T line of each arm (partner
          at REST then at T) passes torque/env/box/same-gantry -- once per gantry
  A_STOP  both arms at T at the stop: env >= 50 mm, box-free (own gantry)
  A_TRAV  rect sweep p0 -> stop with both arms at T, env >= 50 mm (G33 rule)
  A_E1    task-arm end q with the partner at T: env >= 50 mm, box-free
  A_LINE  straight cosine joint line T -> q, partner at T: static torque (OFFSET + MARGIN), env, box, same-gantry
  A_OK    = all of the above for some torque-ok branch (<= ENV_CAP spread, as g64_diag)
Same screens, same margins; only the held configuration changes. Transit dynamics are NOT modelled (static only,
like the scheduler tuple predicate) -> the MoveIt plan screen (RNEA + 7.7) must still pass in mock.

    python3 g64_whatif.py [T_JSON_INDEX]     -> g64_whatif.jsonl + summary
"""
import json
import math
import os
import sys
import time
import warnings
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
warnings.simplefilter('ignore')
import g64_diag as D  # noqa: E402

O2, PREFIX, REST, LIM, OFFSET, MARGIN = D.O2, D.PREFIX, D.REST, D.LIM, D.OFFSET, D.MARGIN
OUT = os.path.join(HERE, 'g64_whatif.jsonl')
T_FILE = os.path.join(HERE, 'g64_transit_T.json')
PER_TARGET = 40                       # SPLIT/OK tuples re-solved per target (lowest stored static j2 first)


def T_q6():
    return np.array(json.load(open(T_FILE))['q6'])


def cos_line(a, b, steps=60):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return [list(a + (b - a) * 0.5 * (1.0 - math.cos(math.pi * k / steps))) for k in range(0, steps + 1)]


def held_with(g, lin, rot, q6s):
    j = D.base_joints(g, lin, rot)
    for arm, q6 in q6s.items():
        j.update({f'{PREFIX[arm]}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
    return j


def screen_line_any(g, arm, lin, rot, pts, held, ec, bx):
    """G41 screen_line for an arbitrary joint line `pts` of `arm`; `held` = every other joint."""
    names = [f'{PREFIX[arm]}joint_{i}' for i in range(1, 7)]
    tau = max(float(np.max(O2.gravity(arm, np.array(q), lin, rot) + OFFSET + MARGIN - LIM)) for q in pts)
    if tau > 0.0:
        return False, 'torsi-garis'
    v, d, _, _ = ec.screen_trajectory(names, pts, held)
    if v != 'CLEAR':
        return False, 'garis-env'
    for q in pts[::3] + [pts[-1]]:
        j = dict(held)
        j.update(zip(names, q))
        b, _ = bx.dist(ec, ec.q_from(j), D.G4.gp(g))
        if b <= 0.0:
            return False, 'garis-kotak'
    v2, _, _, _ = D.G41.same_checker(g).screen_trajectory(names, pts, {k: x for k, x in held.items() if k not in names})
    if v2 != 'CLEAR':
        return False, 'garis-se-gantry'
    return True, ''


def park_ok(g):
    """A_PARK, cached per worker."""
    key = ('park', g)
    if key in D._W:
        return D._W[key]
    ec, bx = D._env()
    T = T_q6()
    lin, rot = D.P0[g]
    a, b = D.ARMS[g]
    res = 'OK'
    ok1, w1 = screen_line_any(g, a, lin, rot, cos_line(REST, T), held_with(g, lin, rot, {a: REST, b: REST}), ec, bx)
    ok2, w2 = screen_line_any(g, b, lin, rot, cos_line(REST, T), held_with(g, lin, rot, {a: T, b: REST}), ec, bx)
    if not ok1 or not ok2:
        res = f'REST->T: {a} {w1 or "ok"}, {b} {w2 or "ok"}'
    D._W[key] = res
    return res


def stop_trav(g, lin, rot):
    key = ('Tstop', g, lin, rot)
    if key in D._W:
        return D._W[key]
    import g29_rot_screen as RS
    ec, bx = D._env()
    T = T_q6()
    a, b = D.ARMS[g]
    j = held_with(g, lin, rot, {a: T, b: T})
    q = ec.q_from(j)
    d, _ = ec.check(q, only=(D.G4.gp(g),))
    bb, _ = bx.dist(ec, q, D.G4.gp(g))
    stop = bool(d >= D.ENV_MIN and bb > 0.0)
    v, dt, _, _ = ec.screen_trajectory([f't{g}_linear_joint', f't{g}_rotation_joint'],
                                       RS.rect_points(D.P0[g], (lin, rot)), held_with(g, *D.P0[g], {a: T, b: T}))
    D._W[key] = (stop, round(float(d), 4), v, round(float(dt), 4))
    return D._W[key]


def _job(row):
    t0 = time.process_time()
    ec, bx = D._env()
    arm, lin, rot_deg = row['arm'], row['lin'], row['rot']
    g = O2.O.GANTRY[arm]
    rot = math.radians(rot_deg)
    T = T_q6()
    partner = [x for x in D.ARMS[g] if x != arm][0]
    out = dict(name=row['name'], arm=arm, lin=lin, rot=rot_deg, current=row['current'], static=row['static'])
    pk = park_ok(g)
    st = stop_trav(g, lin, rot)
    out.update(a_park=pk, a_stop=st[0], a_stop_d=st[1], a_trav=st[2], a_trav_d=st[3])
    why = {}
    a_ok = None
    if pk == 'OK' and st[0] and st[2] == 'CLEAR':
        r = O2.solve(np.asarray(row['xyz'], float), lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
        sols, taus = r['solutions'], r['solutions_tau']
        tq = [i for i in sorted(range(len(sols)), key=lambda i: (float(np.max(np.abs(sols[i][1] - T))), i))
              if D.G41_branch_ok(taus[i])]
        if len(tq) > D.ENV_CAP:
            tq = sorted(set(tq[:10] + [tq[int(k * (len(tq) - 1) / (D.ENV_CAP - 11))] for k in range(D.ENV_CAP - 10)]),
                        key=tq.index)
        n_line = 0
        for i in tq:
            q6 = sols[i][1]
            j = held_with(g, lin, rot, {arm: q6, partner: T})
            q = ec.q_from(j)
            d, _ = ec.check(q, only=(D.G4.gp(g),))
            bb, _ = bx.dist(ec, q, D.G4.gp(g))
            if d < D.ENV_MIN or bb <= 0.0:
                why['e1-T'] = why.get('e1-T', 0) + 1
                continue
            if n_line >= D.LINE_CAP:
                break
            n_line += 1
            ok, w = screen_line_any(g, arm, lin, rot, cos_line(T, q6), held_with(g, lin, rot, {arm: T, partner: T}), ec, bx)
            if ok:
                a_ok = dict(q6=[round(float(v), 5) for v in q6], j2_raw=round(float(taus[i][1]), 4))
                break
            why[w] = why.get(w, 0) + 1
    out.update(a_ok=a_ok is not None, a_goal=a_ok, why=why, cpu_s=round(time.process_time() - t0, 2))
    return out


def main():
    T = {t['name']: t for t in json.load(open(D.TARGETS))}
    rows = {}
    for ln in open(D.OUT):
        d = json.loads(ln)
        if d['name'] in T and d['static']:
            d['xyz'] = T[d['name']]['xyz']
            rows.setdefault(d['name'], []).append(d)
    done = set()
    if os.path.exists(OUT):
        for ln in open(OUT):
            d = json.loads(ln)
            done.add((d['name'], d['arm'], d['lin'], d['rot']))
    todo = []
    for name, rr in rows.items():
        rr = sorted(rr, key=lambda r: r['best_static']['j2_raw'])[:PER_TARGET]
        todo += [r for r in rr if (r['name'], r['arm'], r['lin'], r['rot']) not in done]
    print(f'{len(todo)} tuple statis baru, T = {T_q6().round(4).tolist()}', flush=True)
    with Pool(D.NPROC) as pool, open(OUT, 'a') as f:
        for d in pool.imap_unordered(_job, todo, chunksize=1):
            f.write(json.dumps(d) + '\n')
            f.flush()
    summary()


def summary():
    T = {t['name']: t for t in json.load(open(D.TARGETS))}
    by = {}
    for ln in open(OUT):
        d = json.loads(ln)
        by.setdefault(d['name'], []).append(d)
    res = {}
    for name, rr in by.items():
        cur = any(r['current'] for r in rr)
        a = any(r['a_ok'] for r in rr)
        res[name] = dict(set=T[name]['set'], current=cur, a=a, n=len(rr), n_a=sum(r['a_ok'] for r in rr),
                         a_lost=[(r['arm'], r['lin'], r['rot']) for r in rr if r['current'] and not r['a_ok']])
        print(f"{name:16s} sekarang {cur!s:5s} opsiA {a!s:5s} tuple {len(rr):3d} A-ok {res[name]['n_a']:3d} "
              f"park {rr[0]['a_park']}  hilang-oleh-A {len(res[name]['a_lost'])}")
    json.dump(res, open(os.path.join(HERE, 'g64_whatif_summary.json'), 'w'), indent=1)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'summary':
        summary()
    else:
        main()
