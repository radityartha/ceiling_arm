#!/usr/bin/env python3
"""G8 evaluation harness: optimality gap, bracket above n = 10, and the K4 gate.

docs/p1_g8_sched2.md A3 locked the criteria BEFORE this file existed. Nothing
here may soften them:

  K1  gap = (heuristic - exact) / exact * 100 %.  PASS iff mean <= 5.0 % AND
      max <= 10.0 %, over the WHOLE K2 set, no instance dropped.
  K2  Part I  = 140 real instances, n in {4,6,8,10}, 1 and 2 gantries, mr 0/1,
      seeds identical to G7.  Part II = 50 instances, n in {12,16,20,30,50}.
  K3  above n = 10 there is no exact, so what is reported is a BRACKET:
      UB = best valid heuristic, LB = max(LB_subset, LB_analytic), ratio UB/LB.
      Never called a gap.
  K4  EVERY schedule from EVERY scheduler is replayed by validate_schedule()
      out of test/verify_sched_exact.py -- the same function, unmodified, that
      V4 used on the exact solver. A schedule that fails is not reported with a
      footnote; its number is dropped and the failure counted.

Subcommands
-----------
  part1     gap vs exact, n <= 10                      (~35 min, exact-bound)
  part2     heuristics + LB_analytic, n = 12..50       (minutes)
  part2lb   fills LB_subset into a part2 file          (~45 min, exact-bound)
  ablate    A5-D7: what stage 3 is worth vs stage 4
  report    print every table K5 asks for, from the json files
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from reachability_gng import sched_heur as H          # noqa: E402
from reachability_gng.sched import gen_real, solve_exact  # noqa: E402
from verify_sched_exact import (ref_stop_slots, ref_traverse,  # noqa: E402
                                validate_schedule)

NAMES = ['pose-tour', 'fixed', 'greedy', 'sequential']
PLUS = 'pose-tour+wide'      # POST-HOC variant, see p1_g8_sched2.md B2

PART1 = [(4, list(range(10))), (6, list(range(10))),
         (8, list(range(10))), (10, list(range(5)))]
PART2_N = [12, 16, 20, 30, 50]
PART2_SEEDS = list(range(5))


def _key(rec):
    return f'n{rec["n"]}_s{rec["seed"]}_g{len(rec["gantries"])}_mr{rec["n_mr"]}'


def _load(path):
    p = Path(path)
    return json.loads(p.read_text()) if p.exists() else {}


def _save(path, obj):
    Path(path).write_text(json.dumps(obj, indent=1))


def _run_all(inst, names=None, big=False):
    """Every scheduler on ONE instance, each replayed through the K4 gate."""
    out = {}
    for name in (names or NAMES):
        t0 = time.time()
        try:
            sol = H.SCHEDULERS_PLUS[name](inst)
        except Exception as exc:                       # noqa: BLE001
            out[name] = dict(makespan=None, wall=time.time() - t0,
                             valid=False, errs=[f'raised: {exc!r}'])
            continue
        wall = time.time() - t0
        if big:
            errs, brute, over = gate(inst, sol)
        else:
            errs = validate_schedule(inst, sol)
            brute, over = sum(len(v) for v in sol.stops.values()), 0
        out[name] = dict(makespan=float(sol.makespan), wall=wall,
                         valid=not errs, errs=errs[:4],
                         gate_brute=brute, gate_over=over,
                         traverse_share=float(H.traverse_share(inst, sol)),
                         n_stops={str(g): len(v) for g, v in sol.stops.items()})
    return out


# ---------------------------------------------------------------------------
# K4 above n = 10 -- see docs/p1_g8_sched2.md B4 for why this exists
# ---------------------------------------------------------------------------
GATE_BUDGET = 5_000_000


def _enum_cost(inst, tasks, slots):
    """How many placements ref_stop_slots would enumerate for this stop."""
    n_sr = sum(1 for i in tasks if inst.kind[i] != 'MR')
    n_mr = len(tasks) - n_sr
    k = max(int(round(slots)), 1)
    return (2 ** n_sr) * (k ** n_sr) * (k ** n_mr)


def gate(inst, sol, budget=GATE_BUDGET):
    """K4 replay. Returns (errs, n_stops_brute_forced, n_stops_over_budget).

    Identical in structure to validate_schedule(), and it calls the SAME
    independent ref_traverse / ref_stop_slots. The one difference, forced by
    measurement and written up as an A/B conflict in B4: ref_stop_slots is
    exponential in the tasks at one stop (2**|SR| x slots**|SR|), which at
    n >= 20 reaches 1e9..1e24 placements and never returns. Stops above the
    budget therefore have their LENGTH checked against sched_heur.dur_vec
    instead -- one link further from the brute force, since dur_vec was checked
    exhaustively against sched.stop_duration and V1 checked stop_duration
    against ref_stop_slots. Every other rule is still checked the hard way, and
    the number of stops in each category is reported rather than hidden.
    """
    errs, done, brute, over = [], [], 0, 0
    for g, stops in sol.stops.items():
        t, cur = 0.0, inst.p0[g]
        view = H.GantryView(inst, g)
        for st in stops:
            p = st['pose']
            if p != cur:
                t += ref_traverse(inst.poses[g][cur][0], inst.poses[g][cur][1],
                                  inst.poses[g][p][0], inst.poses[g][p][1])
            if abs(t - st['start']) > 1e-9:
                errs.append(f'g{g} stop start {st["start"]} != replay {t}')
            tasks = [i for i in range(inst.n) if st['tasks'] >> i & 1]
            if _enum_cost(inst, tasks, st['dur'] / inst.dwell) <= budget:
                brute += 1
                slots = ref_stop_slots(inst, g, tasks, p)
                ref = None if slots is None else slots * inst.dwell
            else:
                over += 1
                ref = float(H.dur_vec(view, tasks)[p])
                ref = None if not np.isfinite(ref) else ref
            if ref is None:
                errs.append(f'g{g} stop at pose {p} is INFEASIBLE')
            elif abs(ref - st['dur']) > 1e-9:
                errs.append(f'g{g} stop dur {st["dur"]} != replay {ref}')
            for i in tasks:
                a = st['assign'][i]
                if inst.kind[i] == 'MR':
                    if a != 'both' or not inst.hand[g][i, p]:
                        errs.append(f'g{g} MR task {i} not a valid handover')
                elif a not in H.GANTRY_ARMS[g]:
                    errs.append(f'g{g} task {i} assigned to {a}')
                elif not inst.reach[g][i, p, H.GANTRY_ARMS[g].index(a)]:
                    errs.append(f'g{g} task {i} unreachable by {a} at pose {p}')
            done += tasks
            t += st['dur']
            cur = p
        if abs(t - sol.finish[g]) > 1e-9:
            errs.append(f'g{g} finish {sol.finish[g]} != replay {t}')
    if sorted(done) != list(range(inst.n)):
        errs.append(f'tasks done {sorted(done)} != every task exactly once')
    if abs(max(sol.finish.values()) - sol.makespan) > 1e-9:
        errs.append('makespan is not the max over gantry finish times')
    return errs, brute, over


# ---------------------------------------------------------------------------
def cmd_part1(a):
    res = _load(a.out)
    todo = [(n, s, gs, mr) for n, seeds in PART1 for s in seeds
            for gs in [(1,), (1, 2)] for mr in (0, 1)]
    print(f'part1: {len(todo)} instances, {len(res)} already done')
    for n, seed, gs, mr in todo:
        rec = dict(n=n, seed=seed, gantries=list(gs), n_mr=mr)
        k = _key(rec)
        if k in res:
            continue
        inst = gen_real(n, seed, mr, gs, maps=(a.map1, a.map2))
        t0 = time.time()
        sol = solve_exact(inst)
        rec['exact'] = float(sol.makespan)
        rec['exact_wall'] = time.time() - t0
        rec['exact_valid'] = not validate_schedule(inst, sol)
        rec['sched'] = _run_all(inst)
        res[k] = rec
        _save(a.out, res)
        best = rec['sched']['pose-tour']
        gap = (best['makespan'] / rec['exact'] - 1) * 100 if best['valid'] else float('nan')
        print(f'{k:>22}  exact {rec["exact"]:8.3f} ({rec["exact_wall"]:6.2f} s)'
              f'  pose-tour {best["makespan"]:8.3f}  gap {gap:6.2f}%', flush=True)
    print('part1 done')
    return 0


def cmd_part2(a):
    res = _load(a.out)
    todo = [(n, s, mr) for n in PART2_N for s in PART2_SEEDS for mr in (0, 1)]
    print(f'part2: {len(todo)} instances, {len(res)} already done')
    for n, seed, mr in todo:
        rec = dict(n=n, seed=seed, gantries=[1, 2], n_mr=mr)
        k = _key(rec)
        if k in res and 'sched' in res[k]:
            continue
        inst = gen_real(n, seed, mr, (1, 2), maps=(a.map1, a.map2))
        rec['sched'] = _run_all(inst, big=True)
        lb, lr, lw = H.lb_analytic(inst)
        rec['lb_analytic'] = float(lb)
        rec['lb_reach'] = float(lr)
        rec['lb_work'] = float(lw)
        rec.update({kk: vv for kk, vv in res.get(k, {}).items()
                    if kk.startswith('lb_subset')})
        res[k] = rec
        _save(a.out, res)
        pt = rec['sched']['pose-tour']
        print(f'{k:>22}  pose-tour {pt["makespan"]:8.3f}  '
              f'LB_an {lb:8.3f}  ratio {pt["makespan"]/lb:5.3f}', flush=True)
    print('part2 done')
    return 0


def cmd_part2lb(a):
    """LB_subset: 3 seeded subsets of size 8, exact each, take the max (K3)."""
    res = _load(a.out)
    keys = sorted(res, key=lambda k: (res[k]['n'], res[k]['seed']))
    for k in keys:
        rec = res[k]
        if 'lb_subset' in rec:
            continue
        inst = gen_real(rec['n'], rec['seed'], rec['n_mr'],
                        tuple(rec['gantries']), maps=(a.map1, a.map2))
        t0 = time.time()
        lb, used = H.lb_subset(inst, a.sub_size, a.sub_draws, rec['seed'])
        rec['lb_subset'] = float(lb)
        rec['lb_subset_wall'] = time.time() - t0
        rec['lb_subset_sets'] = used
        _save(a.out, res)
        print(f'{k:>22}  LB_subset {lb:8.3f} ({rec["lb_subset_wall"]:6.1f} s) '
              f'vs LB_analytic {rec["lb_analytic"]:8.3f}', flush=True)
    print('part2lb done')
    return 0


def cmd_rescore(a):
    """Re-run every scheduler on an existing file, keeping the cached exact.

    Exists so that every heuristic number in the report comes from ONE version
    of sched_heur.py even though the exact solves (the expensive half) were
    computed once.
    """
    res = _load(a.out)
    names = NAMES + ([PLUS] if a.plus else [])
    for k in sorted(res, key=lambda k: (res[k]['n'], res[k]['seed'])):
        r = res[k]
        inst = gen_real(r['n'], r['seed'], r['n_mr'], tuple(r['gantries']),
                        maps=(a.map1, a.map2))
        r['sched'] = _run_all(inst, names, big=r['n'] > 10)
        if 'lb_analytic' in r:
            lb, lr, lw = H.lb_analytic(inst)
            r['lb_analytic'], r['lb_reach'], r['lb_work'] = (float(lb),
                                                             float(lr),
                                                             float(lw))
        _save(a.out, res)
        print(f'{k:>22}  ' + '  '.join(
            f'{n} {r["sched"][n]["makespan"]:8.3f}' for n in names), flush=True)
    return 0


def cmd_holdout(a):
    """K1 re-run for the POST-HOC variant on seeds that were NOT in K2.

    p1_g8_sched2.md B2: a heuristic designed after reading the gap table has no
    honest claim on the seeds that produced the table. These seeds (10..14) were
    never looked at while `pose-tour+wide` was being written.
    """
    res = _load(a.out)
    todo = [(n, s, gs, mr) for n in a.tasks for s in range(10, 10 + a.seeds)
            for gs in [(1,), (1, 2)] for mr in (0, 1)]
    for n, seed, gs, mr in todo:
        rec = dict(n=n, seed=seed, gantries=list(gs), n_mr=mr)
        k = _key(rec)
        if k in res:
            continue
        inst = gen_real(n, seed, mr, gs, maps=(a.map1, a.map2))
        t0 = time.time()
        sol = solve_exact(inst)
        rec['exact'] = float(sol.makespan)
        rec['exact_wall'] = time.time() - t0
        rec['exact_valid'] = not validate_schedule(inst, sol)
        rec['sched'] = _run_all(inst, NAMES + [PLUS])
        res[k] = rec
        _save(a.out, res)
        print(f'{k:>22}  exact {rec["exact"]:8.3f} | '
              + ' | '.join(f'{n_} {(rec["sched"][n_]["makespan"]/rec["exact"]-1)*100:6.2f}%'
                           for n_ in ('pose-tour', PLUS)), flush=True)
    return 0


def cmd_ablate(a):
    """A5-D7: remove stage 3 (tour) or stage 4 (local repair), measure."""
    variants = {'full': dict(tour='opt', do_refine=True),
                'nn-only': dict(tour='nn', do_refine=True),
                'cover-order': dict(tour='cover', do_refine=True),
                'no-refine': dict(tour='opt', do_refine=False),
                'neither': dict(tour='cover', do_refine=False)}
    res = _load(a.out)
    cases = [(n, s, (1, 2), mr) for n in a.tasks for s in range(a.seeds)
             for mr in (0, 1)]
    print(f'{"case":>20} ' + ' '.join(f'{v:>12}' for v in variants))
    for n, seed, gs, mr in cases:
        k = f'n{n}_s{seed}_g{len(gs)}_mr{mr}'
        if k in res:
            continue
        inst = gen_real(n, seed, mr, gs, maps=(a.map1, a.map2))
        row = {}
        for v, kw in variants.items():
            t0 = time.time()
            sol = H.pose_tour(inst, **kw)
            errs = gate(inst, sol)[0] if n > 10 else validate_schedule(inst, sol)
            row[v] = dict(makespan=float(sol.makespan), wall=time.time() - t0,
                          valid=not errs)
        res[k] = dict(n=n, seed=seed, n_mr=mr, var=row)
        _save(a.out, res)
        print(f'{k:>20} ' + ' '.join(f'{row[v]["makespan"]:12.3f}'
                                     for v in variants), flush=True)

    print('\n--- D7: mean penalty of removing a stage, relative to full ---')
    base = np.array([r['var']['full']['makespan'] for r in res.values()])
    for v in variants:
        if v == 'full':
            continue
        x = np.array([r['var'][v]['makespan'] for r in res.values()])
        pen = (x / base - 1) * 100
        print(f'{v:>14}  mean {pen.mean():+6.2f} %   max {pen.max():+6.2f} %   '
              f'worse on {int((pen > 1e-9).sum())}/{len(pen)}')
    return 0


# ---------------------------------------------------------------------------
def _stats(v):
    v = np.asarray(v, float)
    return (v.mean(), float(np.median(v)), float(np.percentile(v, 90)),
            v.max())


def cmd_report(a):
    p1 = _load(a.part1)
    p2 = _load(a.part2)

    if p1:
        print('=' * 78)
        print('PART I -- optimality gap vs sched.solve_exact (K1, K5.1)')
        print('=' * 78)
        names = NAMES + ([PLUS] if any(PLUS in r['sched']
                                          for r in p1.values()) else [])
        fails = {n: 0 for n in names}
        fails['exact'] = sum(1 for r in p1.values() if not r['exact_valid'])
        gaps = {n: [] for n in names}
        by_cfg = {}
        for r in p1.values():
            cfg = (r['n'], len(r['gantries']), r['n_mr'])
            by_cfg.setdefault(cfg, {n: [] for n in names})
            for name in names:
                s = r['sched'][name]
                if not s['valid']:
                    fails[name] += 1
                    continue
                g = (s['makespan'] / r['exact'] - 1) * 100
                gaps[name].append(g)
                by_cfg[cfg][name].append(g)

        print(f'\n{len(p1)} instances. K4 gate: '
              + ', '.join(f'{k} {v}' for k, v in fails.items())
              + '  (validate_schedule violations)\n')
        hdr = f'{"n":>3} {"G":>2} {"mr":>3} |'
        for name in names:
            hdr += f' {name+" mean":>16} {"max":>7} |'
        print(hdr)
        print('-' * len(hdr))
        for cfg in sorted(by_cfg):
            line = f'{cfg[0]:>3} {cfg[1]:>2} {cfg[2]:>3} |'
            for name in names:
                v = by_cfg[cfg][name]
                line += (f' {np.mean(v):16.2f} {np.max(v):7.2f} |'
                         if v else f' {"--":>16} {"--":>7} |')
            print(line)
        print('-' * len(hdr))
        print(f'\n{"scheduler":>12} {"n":>4} {"mean%":>8} {"median%":>8} '
              f'{"p90%":>8} {"max%":>8} {"wall_ms":>9} {"K1":>6}')
        for name in names:
            v = gaps[name]
            m, md, p90, mx = _stats(v)
            w = np.mean([r['sched'][name]['wall'] for r in p1.values()]) * 1e3
            ok = 'PASS' if (m <= 5.0 and mx <= 10.0) else 'FAIL'
            print(f'{name:>12} {len(v):>4} {m:8.2f} {md:8.2f} {p90:8.2f} '
                  f'{mx:8.2f} {w:9.1f} {ok:>6}')
        ew = np.mean([r['exact_wall'] for r in p1.values()]) * 1e3
        print(f'{"exact":>12} {len(p1):>4} {0.0:8.2f} {0.0:8.2f} {0.0:8.2f} '
              f'{0.0:8.2f} {ew:9.1f} {"--":>6}')

        v = gaps['pose-tour']
        m, _, _, mx = _stats(v)
        print(f'\nK1 VERDICT  mean {m:.3f} % (threshold 5.0) and '
              f'max {mx:.3f} % (threshold 10.0)  -> '
              f'{"TERCAPAI" if m <= 5.0 and mx <= 10.0 else "TIDAK TERCAPAI"}')
        exact_wins = sum(1 for r in p1.values()
                         if r['sched']['pose-tour']['valid'] and
                         abs(r['sched']['pose-tour']['makespan'] - r['exact'])
                         <= 1e-9)
        print(f'pose-tour is EXACTLY optimal on {exact_wins}/{len(p1)} instances')
        below = [k for k, r in p1.items()
                 if r['sched']['pose-tour']['valid'] and
                 r['sched']['pose-tour']['makespan'] < r['exact'] - 1e-9]
        print(f'instances where a heuristic beat the exact solver (must be 0): '
              f'{len(below)} {below[:3]}')

    if p2:
        print()
        print('=' * 78)
        print('PART II -- bracket above n = 10 (K3, K5.5). NOT a gap.')
        print('=' * 78)
        have_sub = [r for r in p2.values() if 'lb_subset' in r]
        print(f'{len(p2)} instances, LB_subset computed on {len(have_sub)}\n')
        print(f'{"n":>3} {"mr":>3} | {"UB mean":>9} {"LB_an":>9} {"LB_sub":>9} '
              f'{"LB":>9} {"UB/LB mean":>11} {"max":>7} | {"trav%":>6}')
        print('-' * 82)
        rows = {}
        for r in p2.values():
            ubs = [r['sched'][n]['makespan'] for n in NAMES
                   if r['sched'][n]['valid']]
            if not ubs:
                continue
            ub = min(ubs)
            lb = max(r['lb_analytic'], r.get('lb_subset', 0.0))
            rows.setdefault((r['n'], r['n_mr']), []).append(
                (ub, r['lb_analytic'], r.get('lb_subset', float('nan')), lb,
                 ub / lb, r['sched']['pose-tour']['traverse_share'] * 100))
        for k in sorted(rows):
            v = np.array(rows[k], float)
            print(f'{k[0]:>3} {k[1]:>3} | {v[:,0].mean():9.3f} '
                  f'{v[:,1].mean():9.3f} {np.nanmean(v[:,2]):9.3f} '
                  f'{v[:,3].mean():9.3f} {v[:,4].mean():11.3f} '
                  f'{v[:,4].max():7.3f} | {v[:,5].mean():6.1f}')

        print(f'\n{"scheduler":>12} {"valid":>6} {"mean UB":>10} '
              f'{"vs pose-tour":>13} {"wall_ms":>9}')
        pt = {k: r['sched']['pose-tour']['makespan'] for k, r in p2.items()}
        names2 = NAMES + ([PLUS] if any(PLUS in r['sched']
                                        for r in p2.values()) else [])
        for name in names2:
            ok = [k for k, r in p2.items() if r['sched'][name]['valid']]
            ms = np.array([p2[k]['sched'][name]['makespan'] for k in ok])
            rel = np.array([p2[k]['sched'][name]['makespan'] / pt[k] - 1
                            for k in ok]) * 100
            w = np.mean([p2[k]['sched'][name]['wall'] for k in ok]) * 1e3
            print(f'{name:>12} {len(ok):>6} {ms.mean():10.3f} '
                  f'{rel.mean():+12.1f}% {w:9.1f}')

        n50 = [r for r in p2.values() if r['n'] == 50]
        if n50:
            ts = np.array([r['sched']['pose-tour']['traverse_share']
                           for r in n50]) * 100
            print(f'\nK5.6 traverse share at n = 50 (2 gantries): '
                  f'mean {ts.mean():.1f} %  min {ts.min():.1f} %  '
                  f'max {ts.max():.1f} %   (B5 said 74-92 % at n <= 10)')
        fails2 = {n: sum(1 for r in p2.values() if not r['sched'][n]['valid'])
                  for n in NAMES}
        print('K4 gate in Part II: '
              + ', '.join(f'{k} {v}' for k, v in fails2.items()))
        bad = [k for k, r in p2.items()
               if 'lb_subset' in r and
               min([r['sched'][n]['makespan'] for n in NAMES
                    if r['sched'][n]['valid']] or [np.inf])
               < max(r['lb_analytic'], r['lb_subset']) - 1e-9]
        print(f'instances with UB < LB (must be 0, would mean an invalid '
              f'bound): {len(bad)} {bad[:3]}')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def maps(q):
        q.add_argument('--map1', default='/tmp/cap_g1_rail160.npz')
        q.add_argument('--map2', default='/tmp/cap_g2_rail160.npz')

    q = sub.add_parser('part1'); maps(q)
    q.add_argument('--out', default='/tmp/g8_part1.json')
    q.set_defaults(fn=cmd_part1)

    q = sub.add_parser('part2'); maps(q)
    q.add_argument('--out', default='/tmp/g8_part2.json')
    q.set_defaults(fn=cmd_part2)

    q = sub.add_parser('part2lb'); maps(q)
    q.add_argument('--out', default='/tmp/g8_part2.json')
    q.add_argument('--sub-size', type=int, default=8)
    q.add_argument('--sub-draws', type=int, default=3)
    q.set_defaults(fn=cmd_part2lb)

    q = sub.add_parser('rescore'); maps(q)
    q.add_argument('--out', default='/tmp/g8_part1.json')
    q.add_argument('--plus', action='store_true')
    q.set_defaults(fn=cmd_rescore)

    q = sub.add_parser('holdout'); maps(q)
    q.add_argument('--out', default='/tmp/g8_holdout.json')
    q.add_argument('--tasks', type=int, nargs='+', default=[4, 6, 8])
    q.add_argument('--seeds', type=int, default=5)
    q.set_defaults(fn=cmd_holdout)

    q = sub.add_parser('ablate'); maps(q)
    q.add_argument('--out', default='/tmp/g8_ablate.json')
    q.add_argument('--tasks', type=int, nargs='+', default=[8, 12, 20, 50])
    q.add_argument('--seeds', type=int, default=5)
    q.set_defaults(fn=cmd_ablate)

    q = sub.add_parser('report')
    q.add_argument('--part1', default='/tmp/g8_part1.json')
    q.add_argument('--part2', default='/tmp/g8_part2.json')
    q.set_defaults(fn=cmd_report)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    raise SystemExit(main())
