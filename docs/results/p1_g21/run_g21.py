#!/usr/bin/env python3
"""G21 driver -- E1 (Part I under t_fold) and E4 (mutex on/off under t_fold).

Locked in docs/p1_g21_sched_tfold.md A3/A4 before any solve. E2 and E3 run
through test/eval_sched_heur.py (part2 / part2lb / ablate) with --t-fold.

  E1  g8 K2 Part I, 140 instances: solve_exact + validate_schedule, the four g8
      schedulers through eval._run_all (same K4 gate), and the A4 decomposition
      of the critical gantry for exact AND pose-tour.
  E4  n 6/8 x seed 0-9 x (1,), (1,2) x mr 0/1: exact with / without mutex.

Incremental: an interrupted run resumes from its json.

  python3 run_g21.py e1 --t-fold 50.80
  python3 run_g21.py e4 --t-fold 126.80
  python3 run_g21.py c0            # t_fold = 0 E1 vs the g8 archive
"""

import argparse
import json
import sys
import time
from pathlib import Path


HERE = Path(__file__).resolve().parent
PKG = HERE.parents[2] / 'ros2_ws/src/reachability_gng'
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / 'test'))

from reachability_gng import sched_heur as H                   # noqa: E402
from reachability_gng.sched import (gen_real, solve_exact,     # noqa: E402
                                    traverse_time)
from eval_sched_heur import PART1, _key, _run_all              # noqa: E402
from verify_sched_exact import validate_schedule               # noqa: E402

MAPS = (str(PKG / 'data/cap_g1_rail160.npz'),
        str(PKG / 'data/cap_g2_rail160.npz'))


def decomp(inst, sol):
    """A4.2/A4.3: moves, and traverse / fold / dwell of every gantry.

    A move is a pose change, including the first departure from p0. Traverse
    is T_traverse WITHOUT t_fold (sched.traverse_time with t_fold = 0), fold is
    moves x t_fold, dwell is the sum of stop lengths. On one gantry the three
    add up to its finish time -- the model has no waiting.
    """
    per = {}
    for g, stops in sol.stops.items():
        cur, moves, trav, dwell, at_p0 = inst.p0[g], 0, 0.0, 0.0, False
        for i, st in enumerate(stops):
            p = st['pose']
            if p != cur:
                a, b = inst.poses[g][cur], inst.poses[g][p]
                trav += float(traverse_time(b[0] - a[0], b[1] - a[1], 0.0))
                moves += 1
            elif i == 0:
                at_p0 = True
            dwell += st['dur']
            cur = p
        fold = moves * inst.t_fold
        per[str(g)] = dict(moves=moves, trav=trav, fold=fold, dwell=dwell,
                           finish=float(sol.finish[g]), at_p0=at_p0,
                           resid=float(sol.finish[g]) - (trav + fold + dwell),
                           n_stops=len(stops))
    crit = max(per, key=lambda k: per[k]['finish'])
    return dict(per=per, crit=crit)


def cmd_e1(a):
    out = HERE / f'g21_e1_tf{a.t_fold:g}.json'
    res = json.loads(out.read_text()) if out.exists() else {}
    todo = [(n, s, gs, mr) for n, seeds in PART1 for s in seeds
            for gs in [(1,), (1, 2)] for mr in (0, 1)]
    for n, seed, gs, mr in todo:
        rec = dict(n=n, seed=seed, gantries=list(gs), n_mr=mr,
                   t_fold=a.t_fold)
        k = _key(rec)
        if k in res:
            continue
        inst = gen_real(n, seed, mr, gs, maps=MAPS, t_fold=a.t_fold)
        t0 = time.time()
        sol = solve_exact(inst)
        rec['exact'] = float(sol.makespan)
        rec['exact_wall'] = time.time() - t0
        errs = validate_schedule(inst, sol)
        rec['exact_valid'] = not errs
        rec['exact_errs'] = errs[:4]
        rec['exact_dec'] = decomp(inst, sol)
        rec['sched'] = _run_all(inst)
        pt = H.pose_tour(inst)
        rec['pt_dec'] = decomp(inst, pt)
        res[k] = rec
        out.write_text(json.dumps(res, indent=1))
        d = rec['exact_dec']['per'][rec['exact_dec']['crit']]
        print(f'{k:>16} tf {a.t_fold:g}  exact {rec["exact"]:9.3f} '
              f'({rec["exact_wall"]:6.2f}s valid {rec["exact_valid"]})  '
              f'pt {rec["sched"]["pose-tour"]["makespan"]:9.3f}  '
              f'moves {d["moves"]}', flush=True)
    print('e1 done')
    return 0


def cmd_e4(a):
    out = HERE / f'g21_e4_tf{a.t_fold:g}.json'
    res = json.loads(out.read_text()) if out.exists() else {}
    for n in (6, 8):
        for seed in range(10):
            for gs in [(1,), (1, 2)]:
                for mr in (0, 1):
                    rec = dict(n=n, seed=seed, gantries=list(gs), n_mr=mr)
                    k = _key(rec)
                    if k in res:
                        continue
                    inst = gen_real(n, seed, mr, gs, maps=MAPS,
                                    t_fold=a.t_fold)
                    s1 = solve_exact(inst)
                    s0 = solve_exact(inst.without_mutex())
                    rec.update(mutex=float(s1.makespan),
                               nomutex=float(s0.makespan),
                               valid=not validate_schedule(inst, s1),
                               dec=decomp(inst, s1))
                    res[k] = rec
                    out.write_text(json.dumps(res, indent=1))
                    print(f'{k:>16} tf {a.t_fold:g} mutex {rec["mutex"]:9.3f}'
                          f' nomutex {rec["nomutex"]:9.3f}  '
                          f'cost {rec["mutex"] - rec["nomutex"]:+.4f}',
                          flush=True)
    print('e4 done')
    return 0


def cmd_c0(a):
    """C0: t_fold = 0 E1 must reproduce g8_part1.json to 1e-9, 140/140."""
    ours = json.loads((HERE / 'g21_e1_tf0.json').read_text())
    g8 = json.loads((HERE.parent / 'p1_g8/g8_part1.json').read_text())
    bad, cmp_ = [], 0
    for k, r in g8.items():
        o = ours.get(k)
        if o is None:
            bad.append((k, 'missing'))
            continue
        pairs = [('exact', r['exact'], o['exact'])]
        pairs += [(nm, r['sched'][nm]['makespan'], o['sched'][nm]['makespan'])
                  for nm in ('pose-tour', 'fixed', 'greedy', 'sequential')]
        for nm, x, y in pairs:
            cmp_ += 1
            if abs(x - y) > 1e-9:
                bad.append((k, nm, x, y))
    print(f'C0: {len(g8)} archived instances, {cmp_} makespans compared, '
          f'{len(bad)} mismatches > 1e-9')
    for b in bad[:10]:
        print('  ', b)
    return 0 if not bad else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name, fn in (('e1', cmd_e1), ('e4', cmd_e4)):
        q = sub.add_parser(name)
        q.add_argument('--t-fold', type=float, required=True)
        q.set_defaults(fn=fn)
    sub.add_parser('c0').set_defaults(fn=cmd_c0)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == '__main__':
    raise SystemExit(main())
