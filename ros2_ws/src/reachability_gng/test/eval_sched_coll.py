#!/usr/bin/env python3
"""G9 measurement: what does gantry-gantry coordination actually cost?

docs/p1_g9_sched3.md A3-K3 and K5. Delta = coupled - uncoupled, and by Lemma 3
it is >= 0 on every instance -- a negative one means a bug, not a discovery.

The instance sets are A3-K2, fixed before any result was looked at:

  S1  gen_real,         n in {4,6}, seed 0-9, 2 gantries, mr in {0,1}   40
  S2  gen_real_crowded, same shape -- ADVERSARIAL PROBE, reported apart  40
  S3  gen_real,         n = 4, seed 0-9, ONE gantry, mr in {0,1}        20
      -> Delta must be exactly 0 on 20/20; there is no second gantry to
         collide with, so anything else is an implementation error.

S1 and S2 are NEVER averaged together (A3-K2, in red).

Instances where the branch & bound cannot close UB vs LB inside the budget are
reported as a BRACKET, never as a Delta -- p1_g8 A3-K3's rule, reused.

Run:  python3 test/eval_sched_coll.py s3 | s1 | s2 | report
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reachability_gng import sched                     # noqa: E402
from reachability_gng import sched_coll as sc          # noqa: E402

OUT = Path('/tmp/g9_eval.json')
TIME_BUDGET = 120.0                                    # A3-K2, same as g7 K1


def _load():
    return json.loads(OUT.read_text()) if OUT.exists() else {}


def _save(obj):
    OUT.write_text(json.dumps(obj, indent=1))


def _instances(setname):
    out = []
    if setname == 's1':
        for n in (4, 6):
            for seed in range(10):
                for mr in (0, 1):
                    out.append((f'{setname}_n{n}_s{seed}_mr{mr}', n, seed, mr,
                                (1, 2), sched.gen_real))
    elif setname == 's2':
        for n in (4, 6):
            for seed in range(10):
                for mr in (0, 1):
                    out.append((f'{setname}_n{n}_s{seed}_mr{mr}', n, seed, mr,
                                (1, 2), sc.gen_real_crowded))
    elif setname == 's3':
        for seed in range(10):
            for mr in (0, 1):
                out.append((f'{setname}_n4_s{seed}_mr{mr}', 4, seed, mr,
                            (1,), sched.gen_real))
    return out


def run(setname, c_clear=sc.C_CLEAR, tag=None):
    tag = tag or setname
    db = _load()
    db.setdefault(tag, {})
    for key, n, seed, mr, gs, gen in _instances(setname):
        if key in db[tag]:
            continue
        inst = gen(n, seed, mr, gs)
        t0 = time.time()
        s = sc.solve_coupled(inst, c_clear=c_clear, time_budget=TIME_BUDGET)
        rec = dict(n=n, seed=seed, mr=mr, ng=len(gs), lb=float(s.lb),
                   ub=float(s.makespan), proved=bool(s.proved), route=s.route,
                   wall=float(s.wall_s), nodes=int(s.n_nodes),
                   evade=int(s.n_evade), total=time.time() - t0)
        db[tag][key] = rec
        _save(db)
        d = rec['ub'] - rec['lb']
        print(f'{key:22s} unc {rec["lb"]:8.3f}  cpl {rec["ub"]:9.3f}  '
              f'D {d:+8.3f}  {rec["route"]:7s} proved={str(rec["proved"]):5s} '
              f'{rec["wall"]:6.1f}s', flush=True)
    return 0


def _stats(v):
    v = np.asarray(v, float)
    if not len(v):
        return dict(n=0)
    return dict(n=len(v), mean=float(v.mean()), median=float(np.median(v)),
                p90=float(np.percentile(v, 90)), max=float(v.max()))


def report():
    db = _load()
    for tag in sorted(db):
        rows = list(db[tag].values())
        # W2b FAILED (p1_g9 B4): the branch & bound could not recover the
        # optimum even with collisions switched off, so NOTHING it proves is
        # quotable. Only Lemma 4 / single-gantry answers are exact, and their
        # exactness rests on Lemma 3 + Lemma 4 + the W1-verified certificate,
        # never on the search. Everything else is a bracket.
        # Exactness that does NOT depend on the search:
        #   route lemma4/single -- Lemma 4, or no second gantry at all;
        #   ub == lb           -- the upper bound touches a valid lower bound
        #                         (Lemma 3), so it is optimal however it was
        #                         found. The solver's own `proved` flag is
        #                         stricter than it needs to be here.
        def _exact(r):
            return (r['route'] in ('lemma4', 'single')
                    or r['ub'] <= r['lb'] + 1e-9)
        proved = [r for r in rows if _exact(r)]
        open_ = [r for r in rows if not _exact(r)]
        print(f'\n=== {tag}: {len(rows)} instances, {len(proved)} PROVED '
              f'(exact Delta), {len(open_)} left as a BRACKET ===')
        if proved:
            d = [r['ub'] - r['lb'] for r in proved]
            dp = [100.0 * (r['ub'] - r['lb']) / r['lb'] for r in proved]
            bind = sum(1 for x in d if x > 1e-9)
            st, sp = _stats(d), _stats(dp)
            print(f'  Delta  s : mean {st["mean"]:+.4f}  median '
                  f'{st["median"]:+.4f}  p90 {st["p90"]:+.4f}  max '
                  f'{st["max"]:+.4f}')
            print(f'  Delta  % : mean {sp["mean"]:+.4f}  median '
                  f'{sp["median"]:+.4f}  p90 {sp["p90"]:+.4f}  max '
                  f'{sp["max"]:+.4f}')
            print(f'  binds on {bind}/{len(proved)} proved instances')
            verdict = ('GRATIS' if st['max'] <= 1e-9 else
                       'MAHAL' if sp['mean'] >= 5.0 else 'TERUKUR TAPI MURAH')
            print(f'  A3-K3 verdict on the PROVED subset: {verdict}')
        for r in open_:
            print(f'  BRACKET: n={r["n"]} seed={r["seed"]} mr={r["mr"]}  '
                  f'LB {r["lb"]:.3f}  UB {r["ub"]:.3f}  '
                  f'ratio {r["ub"] / r["lb"]:.3f}  nodes {r["nodes"]}')
        routes = {}
        for r in rows:
            routes[r['route']] = routes.get(r['route'], 0) + 1
        print(f'  routes: {routes}   (K5.3: how often Lemma 4 answered, i.e. '
              f'how often the search was never exercised)')
        print(f'  wall: mean {np.mean([r["wall"] for r in rows]):.1f} s, '
              f'max {max(r["wall"] for r in rows):.1f} s')
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:])
    if not a:
        print(__doc__)
        return 1
    if a[0] == 'report':
        return report()
    if a[0] in ('s1', 's2', 's3'):
        return run(a[0])
    print(f'unknown command {a[0]}')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
