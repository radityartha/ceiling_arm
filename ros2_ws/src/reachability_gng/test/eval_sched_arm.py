#!/usr/bin/env python3
"""G11 measurement: what does ARM-ARM collision add on top of the structure?

docs/p1_g11_arm.md A3-K3/K4.

    Delta_struct = coupled(BLOCK only)           - uncoupled     [G10: <= 1.01 %]
    Delta_full   = coupled(BLOCK and ARM_BLOCK)  - uncoupled
    Delta_arm    = Delta_full - Delta_struct                     <- this session

HOW Delta_full IS OBTAINED WITHOUT A SECOND SOLVER, and why that is rigorous
rather than a shortcut:

  LOWER  Delta_full >= Delta_struct. ARM_BLOCK only ever removes schedules, and
         the uncoupled optimum stays a valid lower bound (Lemma 3, unchanged --
         it does not mention collision at all).
  UPPER  If the schedule solve_coupled2 returns under BLOCK alone ALSO satisfies
         ARM_BLOCK, it is a feasible full-model schedule of that value, so
         Delta_full <= Delta_struct, hence EQUAL. No search needed, and the
         answer is exact wherever Delta_struct was exact.
         Where it does not survive, an arm-aware serialisation is tried as a
         feasible completion; where even that fails the instance is reported
         NOT MEASURED, never "assumed unchanged" (A7).

Run:  python3 test/eval_sched_arm.py probe|s1|s2|cost|report
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
from reachability_gng import sched_coll as sc               # noqa: E402
from reachability_gng import sched_coupled as sk            # noqa: E402
from gate_sched_coupled import validate_coupled             # noqa: E402

OUT = Path('/tmp/g11_eval.json')
TIME_BUDGET = 120.0
THRESH = 5.0
C_SWEEP = (0.00, 0.05, 0.10, 0.15, 0.20)     # A2.3, five values


def _path(tag):
    # One file per set. S1 and S2 are run CONCURRENTLY and a shared
    # read-modify-write file loses whichever process saved first -- measured,
    # the S2 table was silently truncated from 37 rows to 2 before anyone
    # looked at the numbers.
    return OUT.with_name(f'g11_eval_{tag}.json')


def _load(tag):
    q = _path(tag)
    return json.loads(q.read_text()) if q.exists() else {}


def _save(tag, o):
    _path(tag).write_text(json.dumps(o, indent=1))


def _instances(setname):
    gen = sched.gen_real if setname == 's1' else sa.gen_real_rotcrowded
    out = []
    for n in (4, 6):
        for seed in range(10):
            for mr in (0, 1):
                out.append((f'n{n}_s{seed}_mr{mr}', n, seed, mr, gen))
    return out


def serial_arm_ub(inst, geom, sol0, c_arm):
    """Serialise: one gantry parks out of the way, the other works, then swap.

    Lemma 1 has NO arm analogue (L5: zero universally safe poses, because the
    opposing arm reaches 1.06 m across a 0.72 m gantry separation), so the park
    pose cannot be chosen by a rule and has to be SEARCHED -- cheapest traverse
    first, accepting the first pose that is arm-clear AND structure-clear
    against the whole of the other gantry's solo schedule.

    Returns (makespan, stops) or (inf, None). Everything it emits is replayed
    through the wait-aware gate by the caller, so a constructor that returns a
    schedule different from the one it checked (p1_g10 B1.2) cannot hide here.
    """
    gs = inst.gantries
    best = (np.inf, None)
    for lead, park in (gs, gs[::-1]):
        lead_stops = sk._stops_with_depart(inst, lead, sol0.stops[lead])
        t_lead = max((s['start'] + s['dur'] for s in lead_stops), default=0.0)
        P = inst.poses[park]
        cur = P[inst.p0[park]]
        order = np.argsort(sched.traverse_time(P[:, 0] - cur[0],
                                               P[:, 1] - cur[1]),
                           kind='stable')
        for p in order[:60]:
            p = int(p)
            T0 = sc.leg_duration(tuple(cur), tuple(P[p]))
            st = ([] if p == int(inst.p0[park]) else
                  [dict(pose=p, depart=0.0, start=T0, dur=0.0, tasks=0,
                        assign={})])
            trial = {lead: lead_stops, park: st}
            if sa.arm_schedule_conflict(inst, geom, trial, c_arm) is not None:
                continue
            if sc.schedule_conflict(inst, trial) is not None:
                continue
            rest = sk._stops_with_depart(inst, park, sol0.stops[park])
            t, cur2, moved = max(t_lead, T0), p, []
            for s in rest:
                T = sc.leg_duration(tuple(P[cur2]), tuple(P[s['pose']]))
                moved.append(dict(s, depart=t, start=t + T))
                t += T + s['dur']
                cur2 = s['pose']
            full = {lead: lead_stops, park: st + moved}
            if (sa.arm_schedule_conflict(inst, geom, full, c_arm) is None
                    and sc.schedule_conflict(inst, full) is None):
                m = max(t_lead, t)
                if m < best[0]:
                    best = (m, full)
            break
    return best


def run(setname, only_n=None, budget=TIME_BUDGET):
    db = _load(setname)
    db.setdefault(setname, {})
    for key, n, seed, mr, gen in _instances(setname):
        if key in db[setname] or (only_n and n not in only_n):
            continue
        t00 = time.time()
        inst = gen(n, seed, mr, (1, 2))
        s = sk.solve_coupled2(inst, time_budget=budget)
        errs = (validate_coupled(inst, s.stops, s.finish, s.makespan)
                if s.stops and np.isfinite(s.makespan) else ['no schedule'])
        geom = sa.ArmGeom(inst)
        rec = dict(n=n, seed=seed, mr=mr, lb=float(s.lb),
                   ub=float(s.makespan), proved=bool(s.proved), route=s.route,
                   wall=float(s.wall_s), gate=len(errs), err=errs[:2],
                   nP=len(inst.poses[1]), arm={})
        if not errs:
            for c in C_SWEEP:
                t0 = time.time()
                hit = sa.arm_schedule_conflict(inst, geom, s.stops, c)
                r = dict(hit=None if hit is None else float(hit),
                         t=time.time() - t0)
                if hit is not None:
                    m, full = serial_arm_ub(inst, geom, s, c)
                    r['serial'] = float(m)
                    if full is not None:
                        r['serial_gate'] = len(validate_coupled(inst, full))
                rec['arm'][f'{c:.2f}'] = r
        rec['total'] = time.time() - t00
        db[setname][key] = rec
        _save(setname, db)
        hits = ''.join('.' if rec['arm'].get(f'{c:.2f}', {}).get('hit') is None
                       else 'X' for c in C_SWEEP)
        print(f'{key:12s} unc {rec["lb"]:8.3f} cpl {rec["ub"]:9.3f} '
              f'D {rec["ub"]-rec["lb"]:+7.3f} {rec["route"]:8s} '
              f'gate={rec["gate"]} ARM[{hits}] {rec["wall"]:6.1f}s', flush=True)
    return 0


def _exact(r):
    return (r['route'] in ('lemma4', 'single', 'dive-lb')
            or r['ub'] <= r['lb'] + 1e-9 or r['proved'])


def verdict(rows, key_hi=None):
    key_hi = key_hi or (lambda r: r['ub'])
    lo, hi = [], []
    for r in rows:
        d_hi = 100.0 * (key_hi(r) - r['lb']) / r['lb']
        lo.append(d_hi if _exact(r) else 0.0)
        hi.append(d_hi)
    m_lo, m_hi = float(np.mean(lo)), float(np.mean(hi))
    exp = (m_lo >= THRESH, m_hi >= THRESH)
    v = ('TIDAK DAPAT DITENTUKAN' if exp[0] != exp[1] else
         'MAHAL' if exp[0] else 'BUKAN MAHAL (< 5 %)')
    if v.startswith('BUKAN') and m_hi <= 1e-9:
        v = 'GRATIS'
    return v, m_lo, m_hi


def report():
    for tag in ('s1', 's2'):
        db = _load(tag)
        if not db.get(tag):
            continue
        rows = list(db[tag].values())
        ok = [r for r in rows if r['gate'] == 0]
        bad = [r for r in rows if r['gate'] != 0]
        print(f'\n=== {tag}: {len(rows)} instances, {len(bad)} failed the gate, '
              f'{sum(1 for r in ok if _exact(r))} exact ===')
        for r in bad:
            print(f'  GATE FAIL n={r["n"]} s={r["seed"]} mr={r["mr"]}: '
                  f'{r["err"]}')
        v, lo, hi = verdict(ok)
        print(f'  Delta_struct: mean Delta% in [{lo:.4f}, {hi:.4f}] -> {v}')
        routes = {}
        for r in rows:
            routes[r['route']] = routes.get(r['route'], 0) + 1
        print(f'  routes {routes}')
        print('  c_arm | struct schedule vs ARM_BLOCK        | Delta_full '
              'mean %                        | Delta_arm')
        for c in C_SWEEP:
            k = f'{c:.2f}'
            have = [r for r in ok if k in r.get('arm', {})]
            if not have:
                continue
            fail = [r for r in have if r['arm'][k]['hit'] is not None]
            resc = [r for r in fail
                    if np.isfinite(r['arm'][k].get('serial', np.inf))]
            unm = len(fail) - len(resc)

            # Delta_full's bracket is NOT Delta_struct's. Its LOWER end is
            # Delta_struct's own lower end (ARM_BLOCK only removes schedules,
            # so Delta_full >= Delta_struct >= Delta_struct_lo); its UPPER end
            # is whatever feasible full-model schedule we actually hold. Where
            # the struct optimum survives ARM_BLOCK that is the struct optimum
            # itself and the two ends meet; where it does not, the only
            # feasible schedule this session constructs is a SERIALISATION,
            # which is a very loose upper bound and says so.
            lo_f, hi_f = [], []
            for r in have:
                d_struct = 100.0 * (r['ub'] - r['lb']) / r['lb']
                a = r['arm'][k]
                lo_f.append(d_struct if _exact(r) else 0.0)
                hi_f.append(100.0 * ((r['ub'] if a['hit'] is None
                                      else a.get('serial', np.inf))
                                     - r['lb']) / r['lb'])
            if unm == 0:
                lof, hif = float(np.mean(lo_f)), float(np.mean(hi_f))
                exp = (lof >= THRESH, hif >= THRESH)
                vf = ('TIDAK DAPAT DITENTUKAN' if exp[0] != exp[1] else
                      'MAHAL' if exp[0] else 'BUKAN MAHAL (< 5 %)')
                if not exp[1] and hif <= 1e-9:
                    vf = 'GRATIS'
                _, los, his = verdict(have)
                cell = (f'[{lof:8.4f}, {hif:9.4f}] {vf:22s} | '
                        f'+[{lof-los:.4f}, {hif-his:8.4f}]')
            else:
                cell = f'NOT MEASURED on {unm} of {len(have)} instances'
            print(f'  {c:5.2f} | {len(have)-len(fail):3d}/{len(have):3d} '
                  f'survive, {len(fail):3d} lose ({len(resc)} serialisable) | '
                  f'{cell}')
        tt = [r['arm'][k]['t'] for r in ok for k in r.get('arm', {})]
        if tt:
            print(f'  K4.4 arm gate: mean {np.mean(tt)*1e3:.1f} ms, max '
                  f'{max(tt)*1e3:.1f} ms per schedule per c_arm')
    return 0


def cost():
    """K4.4, and A2.4 point 1 says it is measured BEFORE any sweep."""
    from reachability_gng.irm_sweep import polyline_min_dist
    inst = sched.gen_real(6, 3, 1, (1, 2))
    t0 = time.time(); geom = sa.ArmGeom(inst); t_geom = time.time() - t0
    rng = np.random.default_rng(0)
    P = inst.poses[1]
    print(f'K4.4 ArmGeom build (4 arms x {inst.n} tasks x {len(P)} poses): '
          f'{t_geom:.2f} s, {4*inst.n*len(P)*12*8/1e6:.1f} MB')

    def bench(fn, reps=3):
        fn()
        out = []
        for _ in range(reps):
            t = time.perf_counter(); fn(); out.append(time.perf_counter() - t)
        return min(out)

    print(f'{"N":>9} | {"pair_distance (BLOCK)":>24} | '
          f'{"polyline_min_dist (ARM)":>25}')
    for N in (1, 100, 10_000, 200_000):
        a = rng.integers(0, len(P), N); b = rng.integers(0, len(P), N)
        t1 = bench(lambda: sc.pair_distance(P[a, 0], P[a, 1], P[b, 0], P[b, 1]))
        A = rng.normal(size=(N, 4, 3)); B = rng.normal(size=(N, 4, 3))
        t2 = bench(lambda: polyline_min_dist(A, B))
        print(f'{N:>9} | {t1/N*1e6:18.4f} us/pair | {t2/N*1e6:19.4f} us/pair')
    print('  ARM_BLOCK = 4 arm pairs x (configs per arm) polyline calls, so it '
          'is 4-16x the right-hand column.')
    s = sk.solve_coupled2(inst, time_budget=60.0)
    for c in (0.0, 0.10):
        t0 = time.time()
        for _ in range(20):
            sa.arm_schedule_conflict(inst, geom, s.stops, c)
        print(f'  full-schedule ARM gate,    c_arm={c:.2f}: '
              f'{(time.time()-t0)/20*1e3:8.1f} ms')
    t0 = time.time()
    for _ in range(20):
        sc.schedule_conflict(inst, s.stops)
    print(f'  full-schedule STRUCT gate (frozen, for scale): '
          f'{(time.time()-t0)/20*1e3:8.1f} ms')
    return 0


def probe():
    """A3-U2: is the locked probe design buildable, and does the built one bind?"""
    poses, hot, m, live, frac = sa.rot_dominance()
    print('U2 (N1) dominance on the REAL map -- the measurement A3-U2 needed '
          'and did not have:')
    print(f'   hot poses (|sin rot| >= 0.525): {int(hot.sum())}/{len(poses)} = '
          f'{100.0*hot.mean():.1f} %')
    print(f'   live nodes {int(live.sum())};  |G(t)| median '
          f'{int(np.median(m.sum(1)[live]))} of {len(poses)}')
    for q in (1.00, 0.90, 0.80, 0.70):
        print(f'   nodes with hot fraction >= {q:.2f}: '
              f'{int(((frac >= q) & live).sum())}')
    p = np.percentile(frac[live], [5, 50, 95])
    print(f'   hot fraction over live nodes: p5 {p[0]:.3f} p50 {p[1]:.3f} '
          f'p95 {p[2]:.3f}  <- the grid base rate, i.e. NO task forces a '
          'rotation')
    print('   => A3-U2 as locked is UNBUILDABLE. The probe restricts the '
          'CANDIDATE POSE SET instead (p1_g7 A2-K1 language).\n')
    for sin_min in (0.525, 0.7625, 0.85):
        binds = ran = 0
        deltas = []
        for seed in range(10):
            inst = sa.gen_real_rotcrowded(4, seed, 0, (1, 2), sin_min=sin_min)
            s = sk.solve_coupled2(inst, time_budget=45.0)
            deltas.append(s.makespan - s.lb)
            binds += (s.makespan - s.lb) > 1e-9
            ran += s.route not in ('lemma4', 'single')
        print(f'   sin_min={sin_min:.4f}  |P|={len(inst.poses[1]):4d}  '
              f'Delta > 0 on {binds}/10;  coupled search ran on {ran}/10;  '
              f'max Delta {max(deltas):+.4f} s')
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['report']
    if a[0] in ('s1', 's2'):
        return run(a[0], only_n=[int(x) for x in a[1:]] or None)
    return {'cost': cost, 'probe': probe, 'report': report}[a[0]]()


if __name__ == '__main__':
    raise SystemExit(main())
