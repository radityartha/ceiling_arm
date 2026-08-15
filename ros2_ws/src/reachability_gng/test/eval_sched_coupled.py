#!/usr/bin/env python3
"""G10 measurement: what does gantry-gantry coordination actually cost?

docs/p1_g10_sched4.md A3-K3 and K5. Delta = coupled - uncoupled, >= 0 on every
instance by Lemma 3; a negative one is a bug, not a discovery.

Instance sets are p1_g9 A3-K2, reused UNCHANGED so the numbers sit next to
p1_g9 B5/B6 directly:

  S1  gen_real,         n in {4,6}, seed 0-9, 2 gantries, mr in {0,1}   40
  S2  gen_real_crowded, same shape -- ADVERSARIAL PROBE, reported apart  40
  S3  gen_real,         n = 4, seed 0-9, ONE gantry, mr in {0,1}        20

S1 and S2 are NEVER averaged together.

THE VERDICT RULE, locked in A3-K3 before any of this ran, and the reason G10
can say something where p1_g9 B6 could only say UNDETERMINED: an instance the
solver cannot close is carried as a BRACKET into BOTH extremes, and the verdict
is pronounced only if both extremes land on the same side of the 5.0 % line.
That averages over all 40 instances either way, so no subset ever gets to
measure its own definition.

Run:  python3 test/eval_sched_coupled.py s1|s2|s3|cclear|mutex|tour|grid|report
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                      # noqa: E402
from reachability_gng import sched_coll as sc           # noqa: E402
from reachability_gng import sched_coupled as sk        # noqa: E402
from reachability_gng import sched_heur                 # noqa: E402
from gate_sched_coupled import total_wait, validate_coupled  # noqa: E402

OUT = Path('/tmp/g10_eval.json')
TIME_BUDGET = 120.0                                    # A3-K4, same as g7 K1
THRESH = 5.0                                           # A3-K3, not negotiable


def _load():
    return json.loads(OUT.read_text()) if OUT.exists() else {}


def _save(o):
    OUT.write_text(json.dumps(o, indent=1))


def _instances(setname):
    out = []
    if setname in ('s1', 's2'):
        gen = sched.gen_real if setname == 's1' else sc.gen_real_crowded
        for n in (4, 6):
            for seed in range(10):
                for mr in (0, 1):
                    out.append((f'n{n}_s{seed}_mr{mr}', n, seed, mr, (1, 2),
                                gen))
    elif setname == 's3':
        for seed in range(10):
            for mr in (0, 1):
                out.append((f'n4_s{seed}_mr{mr}', 4, seed, mr, (1,),
                            sched.gen_real))
    return out


def run(setname, c_clear=sc.C_CLEAR, tag=None, only_n=None):
    tag = tag or setname
    db = _load()
    db.setdefault(tag, {})
    for key, n, seed, mr, gs, gen in _instances(setname):
        if key in db[tag] or (only_n and n not in only_n):
            continue
        inst = gen(n, seed, mr, gs)
        t0 = time.time()
        s = sk.solve_coupled2(inst, c_clear=c_clear, time_budget=TIME_BUDGET)
        errs = (validate_coupled(inst, s.stops, s.finish, s.makespan,
                                 c_clear=c_clear)
                if s.stops and np.isfinite(s.makespan) else ['no schedule'])
        rec = dict(n=n, seed=seed, mr=mr, ng=len(gs), lb=float(s.lb),
                   ub=float(s.makespan), proved=bool(s.proved), route=s.route,
                   wall=float(s.wall_s), nodes=int(s.n_nodes),
                   evade=int(s.n_evade), gate=len(errs), err=errs[:2],
                   wait=float(total_wait(inst, s.stops)) if s.stops else 0.0,
                   bud=s.budget, total=time.time() - t0)
        db[tag][key] = rec
        _save(db)
        print(f'{key:16s} unc {rec["lb"]:8.3f}  cpl {rec["ub"]:9.3f}  '
              f'D {rec["ub"] - rec["lb"]:+8.3f}  {rec["route"]:8s} '
              f'proved={str(rec["proved"]):5s} gate={rec["gate"]} '
              f'{rec["wall"]:6.1f}s', flush=True)
    return 0


def _exact(r):
    """Optimality that does NOT depend on the search being complete.

    route single/lemma4 -- no second gantry, or the uncoupled optimum is itself
    collision-free (Lemma 4); ub == lb -- a feasible schedule touching a valid
    lower bound (Lemma 3), optimal however it was found; or the solver's own
    proof flag, which now includes the Lemma 8 dive certificate.
    """
    return (r['route'] in ('lemma4', 'single', 'dive-lb')
            or r['ub'] <= r['lb'] + 1e-9 or r['proved'])


def _stats(v):
    v = np.asarray(v, float)
    if not len(v):
        return dict(n=0, mean=float('nan'), median=float('nan'),
                    p90=float('nan'), max=float('nan'))
    return dict(n=len(v), mean=float(v.mean()), median=float(np.median(v)),
                p90=float(np.percentile(v, 90)), max=float(v.max()))


def verdict(rows):
    """A3-K3, two-sided. Returns (verdict, mean_lo, mean_hi, detail)."""
    lo, hi = [], []
    for r in rows:
        d_hi = 100.0 * (r['ub'] - r['lb']) / r['lb']
        d_lo = d_hi if _exact(r) else 0.0
        lo.append(d_lo)
        hi.append(d_hi)
    m_lo, m_hi = float(np.mean(lo)), float(np.mean(hi))

    # A3-K3 locks the rule against the 5.0 % LINE: "kalau mean_lo dan mean_hi
    # jatuh di SISI YANG SAMA dari 5.0 %, VONIS SAH". The first implementation
    # tested a three-way label instead and therefore printed UNDETERMINED for a
    # bracket spanning [0.0000, 0.9354] -- both ends far below 5 %, and the
    # question A3-K3 actually poses fully decided. That was the code being
    # stricter than the criterion, the same way the solver's `proved` flag was
    # in p1_g9 B6; the criterion is what governs.
    expensive = (m_lo >= THRESH, m_hi >= THRESH)
    if expensive[0] != expensive[1]:
        v = 'TIDAK DAPAT DITENTUKAN'
    else:
        v = 'MAHAL' if expensive[0] else 'BUKAN MAHAL (< 5 %)'
    # GRATIS is the strictly stronger claim and needs EVERY instance exact:
    # a bracket leaves Delta > 0 possible however small the mean.
    if v.startswith('BUKAN') and m_hi <= 1e-9:
        v = 'GRATIS'
    return v, m_lo, m_hi, (lo, hi)


def report():
    db = _load()
    for tag in sorted(db):
        if not isinstance(db[tag], dict):     # mutex / tour_* are raw tables
            continue
        rows = list(db[tag].values())
        if not rows:
            continue
        ok = [r for r in rows if r['gate'] == 0]
        bad = [r for r in rows if r['gate'] != 0]
        proved = [r for r in ok if _exact(r)]
        open_ = [r for r in ok if not _exact(r)]
        print(f'\n=== {tag}: {len(rows)} instances, {len(bad)} FAILED THE GATE '
              f'(excluded, A3-K4), {len(proved)} exact, {len(open_)} bracket ===')
        for r in bad:
            print(f'  GATE FAIL n={r["n"]} seed={r["seed"]} mr={r["mr"]}: '
                  f'{r["err"]}')
        d = [r['ub'] - r['lb'] for r in proved]
        dp = [100.0 * (r['ub'] - r['lb']) / r['lb'] for r in proved]
        st, sp = _stats(d), _stats(dp)
        print(f'  exact Delta s : mean {st["mean"]:+.4f}  median '
              f'{st["median"]:+.4f}  p90 {st["p90"]:+.4f}  max {st["max"]:+.4f}')
        print(f'  exact Delta % : mean {sp["mean"]:+.4f}  median '
              f'{sp["median"]:+.4f}  p90 {sp["p90"]:+.4f}  max {sp["max"]:+.4f}')
        print(f'  binds (Delta > 1e-9) on {sum(1 for x in d if x > 1e-9)}/'
              f'{len(proved)} exact instances')
        v, m_lo, m_hi, _ = verdict(ok)
        print(f'  A3-K3 two-sided: mean Delta% in [{m_lo:.4f}, {m_hi:.4f}] '
              f'over all {len(ok)} gated instances, threshold {THRESH} '
              f'-> {v}')
        for r in open_:
            print(f'    BRACKET n={r["n"]} seed={r["seed"]} mr={r["mr"]}: '
                  f'LB {r["lb"]:.3f} UB {r["ub"]:.3f} ratio '
                  f'{r["ub"] / r["lb"]:.3f}')
        routes = {}
        for r in rows:
            routes[r['route']] = routes.get(r['route'], 0) + 1
        print(f'  routes {routes}   (K5.3: lemma4/dive-lb = the search never '
              f'had to run)')
        print(f'  wall mean {np.mean([r["wall"] for r in rows]):.1f} s, max '
              f'{max(r["wall"] for r in rows):.1f} s; waiting used on '
              f'{sum(1 for r in rows if r["wait"] > 1e-9)}/{len(rows)}, '
              f'mean {np.mean([r["wait"] for r in rows]):.2f} s')
        b = {}
        for r in rows:
            for k, v2 in (r.get('bud') or {}).items():
                b[k] = b.get(k, 0) + v2
        print(f'  K5.8 budgets touched: '
              + ', '.join(f'{k}={v2}' for k, v2 in sorted(b.items())))
    return 0


# ---------------------------------------------------------------------------
# K5.4 -- the predicate on the grid. The data path, which p1_state 7.2 notes is
# where the only correct guess ever came from.
# ---------------------------------------------------------------------------
def grid():
    from reachability_gng.capability import CapabilityMap
    cap = CapabilityMap.load('/tmp/cap_g1_rail160.npz')
    poses = np.stack(np.meshgrid(cap.lin, cap.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    P = len(poses)
    t0 = time.time()
    blk = sc.blocked_poses(poses, poses)
    n_blk = int(blk.sum())
    rot = cap.rot
    s = np.abs(np.sin(rot))
    n1 = (s[:, None] + s[None, :]) >= 1.525
    print(f'K5.4 |P| = {P}, {P * P} pose pairs, {n_blk} BLOCK at c_clear = '
          f'0.00 -> {100.0 * n_blk / P / P:.4f} %   ({time.time() - t0:.1f} s)')
    print(f'     (N1) satisfiable on {int(n1.sum())}/{n1.size} = '
          f'{100.0 * n1.mean():.2f} % of (rot1, rot2) pairs -- the algebraic '
          f'ceiling the exact predicate sits under')
    per = blk.any(axis=1)
    print(f'     poses that block SOMETHING: {int(per.sum())}/{P} = '
          f'{100.0 * per.mean():.2f} %;  poses that block NOTHING (Lemma 1 '
          f'safe set): {P - int(per.sum())}')
    for c in (0.05, 0.10):
        b2 = int(sc.blocked_poses(poses, poses, c_clear=c).sum())
        print(f'     c_clear = {c:.2f}: {b2} pairs = '
              f'{100.0 * b2 / P / P:.4f} %  (x{b2 / max(n_blk, 1):.2f})')
    return 0


# ---------------------------------------------------------------------------
# K5.5(a) mutex, K5.5(b) tour order -- both on the COUPLED model
# ---------------------------------------------------------------------------
def mutex(n_list=(4,)):
    """p1_g7 B3 (mutex costs 0.000 s) re-tested under time coupling. D18."""
    rows = []
    for n in n_list:
        for seed in range(10):
            for mr in (0, 1):
                inst = sched.gen_real(n, seed, mr, (1, 2))
                a = sk.solve_coupled2(inst, time_budget=TIME_BUDGET)
                b = sk.solve_coupled2(inst.without_mutex(),
                                      time_budget=TIME_BUDGET)
                rows.append((n, seed, mr, a.makespan, b.makespan,
                             a.proved and b.proved))
                print(f'  n{n} s{seed} mr{mr}: mutex {a.makespan:9.3f}  '
                      f'no-mutex {b.makespan:9.3f}  cost '
                      f'{a.makespan - b.makespan:+7.3f}  '
                      f'both_proved={rows[-1][5]}', flush=True)
    d = np.array([r[3] - r[4] for r in rows])
    both = [r for r in rows if r[5]]
    print(f'K5.5(a) mutex cost under coupling: mean {d.mean():+.4f} s, max '
          f'{d.max():+.4f} s, binds on {int((d > 1e-9).sum())}/{len(d)}; '
          f'{len(both)}/{len(rows)} pairs both proved')
    _save({**_load(), 'mutex': [list(map(float, r[:5])) + [bool(r[5])]
                                for r in rows]})
    return 0


def tour(setname='s1', n_list=(4, 6)):
    """p1_g8 B6 (tour order contributes 0.00 %) re-tested under coupling. D19.

    The heuristic itself is NOT modified -- A0 freezes sched_heur. Each variant
    plans as it always did, and its plan is then placed in time by
    sched_coupled.repair_schedule, which keeps every (pose, task set) choice and
    only moves the clock. So what is measured is what the tour COSTS once the
    two gantries are coupled in time, which is exactly the question p1_g8 B10.1
    left open.
    """
    gen = sched.gen_real if setname == 's1' else sc.gen_real_crowded
    variants = {'full': dict(tour='opt', do_refine=True),
                'nn-only': dict(tour='nn', do_refine=True),
                'cover-order': dict(tour='cover', do_refine=True)}
    rows = []
    for n in n_list:
        for seed in range(5):
            for mr in (0, 1):
                inst = gen(n, seed, mr, (1, 2))
                vals = {}
                for v, kw in variants.items():
                    try:
                        sol = sched_heur.pose_tour(inst, **kw)
                    except Exception as e:                      # noqa: BLE001
                        vals[v] = float('nan')
                        continue
                    m, st = sk.repair_schedule(inst, sol.stops)
                    if st is not None:
                        errs = validate_coupled(inst, st)
                        if errs:
                            m = float('nan')
                    vals[v] = float(m)
                rows.append((n, seed, mr, vals))
                print(f'  n{n} s{seed} mr{mr}: '
                      + '  '.join(f'{v} {vals[v]:9.3f}' for v in variants),
                      flush=True)
    print(f'K5.5(b) tour-order penalty under coupling ({setname}):')
    base = np.array([r[3]['full'] for r in rows])
    for v in variants:
        if v == 'full':
            continue
        x = np.array([r[3][v] for r in rows])
        pen = 100.0 * (x - base) / base
        good = np.isfinite(pen)
        print(f'   {v:12s} mean {np.nanmean(pen[good]):+.4f} %  max '
              f'{np.nanmax(pen[good]):+.4f} %  worse on '
              f'{int((pen[good] > 1e-9).sum())}/{int(good.sum())}')
    _save({**_load(), f'tour_{setname}': [[r[0], r[1], r[2], r[3]]
                                          for r in rows]})
    return 0


def cclear(setname='s1', n_list=(4,)):
    """K5.6 sweep. Reported for the n it was actually run on, as a number."""
    for c in (0.05, 0.10):
        run(setname, c_clear=c, tag=f'{setname}_c{int(c * 100):02d}',
            only_n=n_list)
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:])
    if not a:
        print(__doc__)
        return 1
    cmd = a[0]
    if cmd == 'report':
        return report()
    if cmd == 'grid':
        return grid()
    if cmd == 'mutex':
        return mutex()
    if cmd.startswith('tour'):
        return tour('s2' if cmd.endswith('2') else 's1')
    if cmd.startswith('cclear'):
        return cclear('s2' if cmd.endswith('2') else 's1')
    if cmd in ('s1', 's2', 's3'):
        return run(cmd)
    print(f'unknown command {cmd}')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
