#!/usr/bin/env python3
"""G12 measurement: Delta_full, Delta_arm, Lemma C. docs/p1_g12_armsolve.md A4/A5.

    Delta_struct = coupled(BLOCK only,  c_clear = c_arm + delta) - uncoupled
    Delta_full   = coupled(BLOCK and ARM_BLOCK)                  - uncoupled
    Delta_arm    = Delta_full - Delta_struct

Both sides are run at the SAME c_clear so that the subtraction subtracts
comparable things (A2.2): Lemma B discharges hanging x hanging through the
structural predicate, which means the arm-aware run is at c_clear = c_arm +
delta, and a Delta_struct measured at c_clear = 0 would fold the clearance
change into "the arms". The G11 basis (c_clear = 0) is reported too, as the M5
regression, and never subtracted.

Run:  python3 test/eval_sched_armfull.py cost|lemmac|s1|s2|report
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_arm as sa                # noqa: E402
from reachability_gng import sched_armfull as af            # noqa: E402
from reachability_gng import sched_coll as sc               # noqa: E402
from reachability_gng import sched_coupled as sk            # noqa: E402
from gate_sched_coupled import validate_coupled             # noqa: E402

TIME_BUDGET = 120.0
THRESH = 5.0
C_SWEEP = (0.00, 0.05, 0.10, 0.15, 0.20)
C_MAIN = 0.05


def _path(tag, c):
    # One file per (set, c_arm). p1_g11 measured why it cannot be one file per
    # set: two processes read-modify-writing one JSON silently truncated the S2
    # table from 37 rows to 2. G13 A2.2 runs cells CONCURRENTLY, and a cell is
    # a (set, c_arm) pair, so the key has to carry c as well.
    return Path(f'/tmp/g13_eval_{tag}_{c:.2f}.json')


def _load(tag, c):
    q = _path(tag, c)
    return json.loads(q.read_text()) if q.exists() else {}


def _save(tag, c, o):
    _path(tag, c).write_text(json.dumps(o, indent=1))


def _load1():
    """load average, so a cell's wall can never be read without its machine.

    p1_g10 C: seconds are not comparable across sessions without the load, and
    G13 is the first session to run cells in parallel -- which moves the
    proved-count, i.e. the Delta_lo side of the two-sided verdict, not just the
    wall.  A2.2.
    """
    import os
    return os.getloadavg()[0]


def _instances(setname):
    gen = sched.gen_real if setname == 's1' else sa.gen_real_rotcrowded
    return [(f'n{n}_s{seed}_mr{mr}', n, seed, mr, gen)
            for n in (4, 6) for seed in range(10) for mr in (0, 1)]


# ---------------------------------------------------------------- A2.5
def cost():
    """The evaluation budget, measured BEFORE any sweep. A2.5, and it is the
    FIFTH session running with the same trap."""
    inst = sched.gen_real(6, 3, 1, (1, 2))
    t0 = time.time()
    geom = sa.ArmGeom(inst)
    print(f'A2.5 ArmGeom build: {time.time() - t0:.2f} s')

    s = sk.solve_coupled2(inst, c_clear=0.0, time_budget=60.0)
    stops = {g: sk._stops_with_depart(inst, g, s.stops[g])
             for g in inst.gantries}
    ctx = af.ArmCtx(inst, geom, C_MAIN)

    # 1. whole-schedule gates, for scale
    def bench(fn, reps=20):
        fn()
        t = time.perf_counter()
        for _ in range(reps):
            fn()
        return (time.perf_counter() - t) / reps

    t_slow = bench(lambda: sa.arm_schedule_conflict(inst, geom, stops, C_MAIN))
    t_fast = bench(lambda: af.schedule_arm_conflict(inst, geom, stops, C_MAIN))
    t_str = bench(lambda: sc.schedule_conflict(inst, stops, C_MAIN + 0.004))
    print(f'  whole-schedule gate  SLOW (p1_g11, full walk) {t_slow*1e3:8.2f} ms')
    print(f'  whole-schedule gate  FAST (A2.3 decomposed)   {t_fast*1e3:8.2f} ms'
          f'   -> {t_slow/max(t_fast,1e-9):6.1f}x cheaper')
    print(f'  whole-schedule gate  STRUCT (frozen)          {t_str*1e3:8.2f} ms')

    # 2. THE number A2.5 locks: overhead of one action test over the frozen one
    gs = inst.gantries
    g, h = gs
    trg = sc.Traj(g, tuple(inst.poses[g][inst.p0[g]]))
    trh = sc.Traj(h, tuple(inst.poses[h][inst.p0[h]]))
    st_h = stops[h]
    for x in st_h:
        trh.append(x['depart'], tuple(inst.poses[h][x['pose']]))
    q = tuple(inst.poses[g][stops[g][0]['pose']])
    dur = stops[g][0]['dur']
    t_fs = bench(lambda: sk.feasible_starts(trg, q, dur, 0.0, trh,
                                            C_MAIN + 0.004, sc.EPS_CERT), 20)
    cand = sk.feasible_starts(trg, q, dur, 0.0, trh, C_MAIN + 0.004,
                              sc.EPS_CERT)
    s0, newtr = cand[0]
    nst = [dict(stops[g][0], depart=s0,
                start=s0 + sc.leg_duration(trg.end_pose(), q))]
    t_arm = bench(lambda: ctx.ok(g, newtr, nst, h, trh, st_h, s0), 50)
    print(f'\n  A2.5 ACTION TEST, the number the mounting is judged on:')
    print(f'    feasible_starts (frozen)      {t_fs*1e3:8.3f} ms')
    print(f'    ctx.ok           (arm)        {t_arm*1e3:8.3f} ms')
    print(f'    overhead                      {t_arm/max(t_fs,1e-9):8.2f}x   '
          f'(A2.5 budget: <= 5x)')

    # 3. case (ii) single evaluation
    w = af._wins_of(stops[g])[0]
    w2 = af._wins_of(stops[h])[0]
    A = af.ArmView(geom, g, trg, [w])
    B = af.ArmView(geom, h, trh, [w2])
    t_ii = bench(lambda: af._pair_dist(A, B, w[0] + 1e-6), 200)
    print(f'    one case (ii) evaluation      {t_ii*1e6:8.1f} us  '
          f'(A2.5 budget: <= 30 us)')

    # 4. exhaustive serial UB
    t0 = time.time()
    m, _ = af.arm_serial_ub(inst, ctx, s, C_MAIN + 0.004)
    print(f'    arm_serial_ub (ALL 2376 park poses) {time.time()-t0:6.2f} s  '
          f'-> makespan {m:.3f}  (A2.5 budget: <= 1.0 s)')
    return 0


# ---------------------------------------------------------------- Lemma C
def lemmac(setname='s1', c=C_MAIN, limit=None):
    """A2.4 Lemma C, MEASURED: does a hanging park pose always exist?"""
    rows, zeros = [], []
    for key, n, seed, mr, gen in _instances(setname)[:limit]:
        inst = gen(n, seed, mr, (1, 2))
        geom = sa.ArmGeom(inst)
        ctx = af.ArmCtx(inst, geom, c)
        s0 = sched.solve_exact(inst)
        st = {g: sk._stops_with_depart(inst, g, s0.stops[g])
              for g in inst.gantries}
        out = af.lemma_c(inst, ctx, st)
        for g, j, k, P in out:
            rows.append(k / P)
            if k == 0:
                zeros.append((key, g, j))
        print(f'{key:12s} stops {len(out):2d}  park poses free: '
              f'min {min((k for _, _, k, _ in out), default=0):5d} / '
              f'{out[0][3] if out else 0}', flush=True)
    print(f'\nLEMMA C on {setname}, c_arm = {c}: {len(rows)} stops, '
          f'{len(zeros)} with ZERO park pose')
    if rows:
        p = np.percentile(rows, [0, 5, 50])
        print(f'  fraction of |P| that is free: min {p[0]:.4f}  p5 {p[1]:.4f} '
              f' p50 {p[2]:.4f}')
    print('  -> LEMMA C ' + ('HOLDS (serialisation always exists)' if not zeros
                             else f'FAILS on {len(zeros)}: {zeros[:5]} '
                                  '<- a statement about the CELL'))
    return 0


# ---------------------------------------------------------------- the sweep
def run(setname, cs=(C_MAIN,), budget=TIME_BUDGET, only_n=None):
    for c in cs:
        db = _load(setname, c)
        db.setdefault(setname, {})
        db.setdefault('_load', [_load1(), None])
        _run_cell(setname, c, db, budget, only_n)
        db['_load'][1] = _load1()
        _save(setname, c, db)
        print(f'CELL {setname} c={c:.2f} DONE  load {db["_load"][0]:.2f} -> '
              f'{db["_load"][1]:.2f}', flush=True)
    return 0


def _run_cell(setname, c, db, budget, only_n):
    for key, n, seed, mr, gen in _instances(setname):
        if only_n and n not in only_n:
            continue
        rec = db[setname].get(key)
        inst = geom = None
        for _ in (0,):
            ck = f'{c:.2f}'
            if rec and ck in rec.get('arm', {}):
                continue
            if inst is None:
                inst = gen(n, seed, mr, (1, 2))
                geom = sa.ArmGeom(inst)
            # 🔺 CONTRADICTION 2 (sched_armfull.ArmCtx): NOT c + delta. Lemma B
            # is dropped as a mechanism, so both sides run at c_clear = 0 --
            # the exact p1_g10/p1_g11 basis, which makes Delta_arm a difference
            # in ARMS ALONE and makes the M5 regression the same run.
            cc = 0.0
            ss = sk.solve_coupled2(inst, c_clear=cc, time_budget=budget)
            t0 = time.time()
            sf, ctx = af.solve_armfull(inst, c_arm=c, geom=geom, c_clear=cc,
                                       time_budget=budget)
            wall = time.time() - t0
            errs = (validate_coupled(inst, sf.stops, sf.finish, sf.makespan)
                    if sf.stops and np.isfinite(sf.makespan) else ['no schedule'])
            slow = (sa.arm_schedule_conflict(inst, geom, sf.stops, c)
                    if sf.stops else None)
            if rec is None:
                rec = dict(n=n, seed=seed, mr=mr, lb=float(sf.lb),
                           nP=len(inst.poses[1]), arm={})
            rec['arm'][ck] = dict(
                struct_ub=float(ss.makespan), struct_route=ss.route,
                struct_proved=bool(ss.proved),
                full_ub=float(sf.makespan), full_route=sf.route,
                full_proved=bool(sf.proved), full_nodes=int(sf.n_nodes),
                wall=wall, gate=len(errs), err=[str(x) for x in errs[:2]],
                arm_slow=None if slow is None else float(slow),
                arm_calls=ctx.calls, arm_t=ctx.t_arm)
            db[setname][key] = rec
            _save(setname, c, db)
            print(f'{key:12s} c={ck} unc {rec["lb"]:8.3f} '
                  f'str {ss.makespan:9.3f}({ss.route:7s}) '
                  f'full {sf.makespan:9.3f}({sf.route:7s},pr={int(sf.proved)}) '
                  f'D_arm {sf.makespan-ss.makespan:+8.3f} gate={len(errs)}'
                  f'{"" if slow is None else " ARMVIOL"} {wall:6.1f}s',
                  flush=True)


def _exact(route, proved, ub, lb):
    return route in ('lemma4', 'single', 'dive-lb') or ub <= lb + 1e-9 or proved


def verdict(pairs):
    """pairs = [(lb, ub, exact)] -> two-sided verdict, p1_g10 A3-K3 unchanged."""
    lo, hi = [], []
    for lb, ub, ex in pairs:
        d = 100.0 * (ub - lb) / lb
        lo.append(d if ex else 0.0)
        hi.append(d)
    m_lo, m_hi = float(np.mean(lo)), float(np.mean(hi))
    exp = (m_lo >= THRESH, m_hi >= THRESH)
    v = ('TIDAK DAPAT DITENTUKAN' if exp[0] != exp[1] else
         'MAHAL' if exp[0] else 'BUKAN MAHAL (< 5 %)')
    if v.startswith('BUKAN') and m_hi <= 1e-9:
        v = 'GRATIS'
    return v, m_lo, m_hi


def report():
    for tag in ('s1', 's2'):
        seen = [c for c in C_SWEEP if _path(tag, c).exists()]
        if not seen:
            print(f'\n=== {tag}: TIDAK DIUKUR -- no cell has been run ===')
            continue
        print(f'\n=== {tag} ===')
        for c in C_SWEEP:
            raw = _load(tag, c)
            db, ld = raw.get(tag, {}), raw.get('_load')
            ck = f'{c:.2f}'
            rows = [(k, r, r['arm'][ck]) for k, r in db.items()
                    if ck in r.get('arm', {})]
            if not rows:
                print(f'\n  c_arm = {ck}   TIDAK DIUKUR')
                continue
            print(f'  [load {ld[0]:.2f} -> '
                  f'{"RUNNING" if ld[1] is None else f"{ld[1]:.2f}"}]' if ld
                  else '  [load NOT RECORDED -- A2.2: cell not reported]')
            bad = [k for k, _, a in rows if a['gate'] or a['arm_slow'] is not None]
            none = [k for k, _, a in rows if not np.isfinite(a['full_ub'])]
            st = [(r['lb'], a['struct_ub'],
                   _exact(a['struct_route'], a['struct_proved'],
                          a['struct_ub'], r['lb'])) for _, r, a in rows]
            fu = [(r['lb'], a['full_ub'],
                   _exact(a['full_route'], a['full_proved'], a['full_ub'],
                          r['lb'])) for _, r, a in rows
                  if np.isfinite(a['full_ub'])]
            vs, slo, shi = verdict(st)
            print(f'\n  c_arm = {ck}   ({len(rows)} instances, '
                  f'{len(bad)} gate failures, {len(none)} NO SCHEDULE FOUND)'
                  + ('   [DEGENERATE, p1_g11 B5 pert. 1]' if c == 0.0 else ''))
            print(f'    Delta_struct  mean% [{slo:8.4f}, {shi:8.4f}]  {vs}')
            if none:
                print(f'    Delta_full    NOT MEASURED on {len(none)} of '
                      f'{len(rows)}: {none[:5]}')
                continue
            vf, flo, fhi = verdict(fu)
            print(f'    Delta_full    mean% [{flo:8.4f}, {fhi:8.4f}]  {vf}')
            print(f'    Delta_arm     mean% [{flo-slo:+8.4f}, {fhi-shi:+8.4f}]')
            ex = sum(1 for _, _, e in fu if e)
            rt = {}
            for _, _, a in rows:
                k2 = (a['struct_route'], a['full_route'])
                rt[k2] = rt.get(k2, 0) + 1
            changed = sum(v for k2, v in rt.items() if k2[0] != k2[1])
            w = [a['wall'] for _, _, a in rows]
            print(f'    exact {ex}/{len(fu)};  route changed by arms on '
                  f'{changed}/{len(rows)};  wall mean {np.mean(w):.1f} s '
                  f'max {max(w):.1f} s')
            print(f'    routes (struct -> full): '
                  + ', '.join(f'{a}->{b}:{v}' for (a, b), v in
                              sorted(rt.items(), key=lambda x: -x[1])))
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['report']
    if a[0] in ('s1', 's2'):
        cs = [float(x) for x in a[1:]] or [C_MAIN]
        return run(a[0], cs=cs)
    if a[0] == 'lemmac':
        b = a[1:] or ['s1']
        return lemmac(b[0], *(float(x) for x in b[1:2]), *(int(x) for x in b[2:3]))
    return {'cost': cost, 'report': report}[a[0]]()


if __name__ == '__main__':
    raise SystemExit(main())
