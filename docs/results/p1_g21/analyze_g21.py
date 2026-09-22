#!/usr/bin/env python3
"""G21 tables (p1_g21 A4), from the json files next to this script only.

  python3 analyze_g21.py        -> stdout, and g21_tables.txt
"""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TF = (0.0, 50.8, 126.8)
NAMES = ['pose-tour', 'fixed', 'greedy', 'sequential']


def load(stem, tf):
    p = HERE / f'{stem}_tf{tf:g}.json'
    return json.loads(p.read_text()) if p.exists() else {}


def crit(dec):
    return dec['per'][dec['crit']]


def shares(dec):
    c = crit(dec)
    f = c['finish']
    return np.array([c['trav'], c['fold'], c['dwell']]) / f if f > 0 else \
        np.full(3, np.nan)


def tot_moves(dec):
    return sum(v['moves'] for v in dec['per'].values())


out = []


def pr(*a):
    s = ' '.join(str(x) for x in a)
    out.append(s)
    print(s)


# ----------------------------------------------------------------- E1
pr('=' * 100)
pr('E1 -- Part I (g8 K2, 140 instances), exact vs pose-tour, per t_fold')
pr('=' * 100)
E1 = {tf: load('g21_e1', tf) for tf in TF}
summ = {}
for tf in TF:
    d = E1[tf]
    if not d:
        continue
    pr(f'\n--- t_fold = {tf:g} s   ({len(d)} instances) ---')
    pr(f'{"n":>3} {"G":>2} {"mr":>3} {"#":>3} | {"exact":>8} {"moves":>5} '
       f'{"trav%":>6} {"fold%":>6} {"dwell%":>6} {"p0":>3} | {"gap% mean":>9} '
       f'{"max":>6} {"gap s mean":>10} {"max":>6} {"dmv":>4} {"opt":>4}')
    cfgs = sorted({(r['n'], len(r['gantries']), r['n_mr']) for r in d.values()})
    allgap, allgs, mv, sh, bad, dmv_max = [], [], [], [], {n: 0 for n in NAMES}, []
    ex_bad, opt = 0, 0
    for cfg in cfgs:
        rs = [r for r in d.values()
              if (r['n'], len(r['gantries']), r['n_mr']) == cfg]
        ex = np.array([r['exact'] for r in rs])
        m = np.array([crit(r['exact_dec'])['moves'] for r in rs])
        s = np.array([shares(r['exact_dec']) for r in rs])
        p0 = sum(any(v['at_p0'] for v in r['exact_dec']['per'].values())
                 for r in rs)
        pt = np.array([r['sched']['pose-tour']['makespan'] for r in rs])
        gap = (pt / ex - 1) * 100
        gs = pt - ex
        dmv = np.array([tot_moves(r['pt_dec']) - tot_moves(r['exact_dec'])
                        for r in rs])
        o = int((np.abs(gs) <= 1e-9).sum())
        opt += o
        pr(f'{cfg[0]:>3} {cfg[1]:>2} {cfg[2]:>3} {len(rs):>3} | {ex.mean():8.2f} '
           f'{m.mean():5.2f} {s[:,0].mean()*100:6.1f} {s[:,1].mean()*100:6.1f} '
           f'{s[:,2].mean()*100:6.1f} {p0:>3} | {gap.mean():9.2f} {gap.max():6.2f} '
           f'{gs.mean():10.2f} {gs.max():6.2f} {dmv.max():>4} {o:>4}')
        allgap += list(gap)
        allgs += list(gs)
        mv += list(m)
        sh += list(s)
    for r in d.values():
        ex_bad += not r['exact_valid']
        for n in NAMES:
            bad[n] += not r['sched'][n]['valid']
        assert all(abs(v['resid']) < 1e-6 for v in r['exact_dec']['per'].values())
    allgap, allgs, sh = np.array(allgap), np.array(allgs), np.array(sh)
    k = max(d, key=lambda k: d[k]['sched']['pose-tour']['makespan'] / d[k]['exact'])
    r = d[k]
    pr(f'ALL: exact moves (critical) mean {np.mean(mv):.3f}; shares trav/fold/dwell '
       f'{sh[:,0].mean()*100:.1f} / {sh[:,1].mean()*100:.1f} / {sh[:,2].mean()*100:.1f} %'
       f' (gantry = trav+fold, per-instance min {((sh[:,0]+sh[:,1]).min())*100:.1f} %)')
    pr(f'     pose-tour gap mean {allgap.mean():.3f} %  median {np.median(allgap):.3f}'
       f'  max {allgap.max():.3f} %  | gap s mean {allgs.mean():.3f} max {allgs.max():.3f}'
       f'  | exactly optimal {opt}/{len(d)}  | K1 '
       f'{"TERCAPAI" if allgap.mean() <= 5 and allgap.max() <= 10 else "TIDAK TERCAPAI"}')
    pr(f'     max-gap instance {k}: pt moves {tot_moves(r["pt_dec"])} vs exact '
       f'{tot_moves(r["exact_dec"])} (critical {crit(r["pt_dec"])["moves"]} vs '
       f'{crit(r["exact_dec"])["moves"]}), pt stops {r["pt_dec"]["per"]}'[:400])
    pr(f'     K4: exact invalid {ex_bad}; ' + ', '.join(f'{n} {v}' for n, v in bad.items()))
    base = {}
    for n in NAMES[1:]:
        g = [(r['sched'][n]['makespan'] / r['exact'] - 1) * 100 for r in d.values()
             if r['sched'][n]['valid']]
        base[n] = (np.mean(g), np.max(g))
    pr('     baselines gap mean/max: ' + ', '.join(
        f'{n} {v[0]:.2f}/{v[1]:.2f} %' for n, v in base.items()))
    heur_beats = sum(r['sched'][n]['makespan'] < r['sched']['pose-tour']['makespan'] - 1e-9
                     for r in d.values() for n in ('greedy', 'sequential'))
    pr(f'     (greedy|sequential) < pose-tour on {heur_beats} (instance, baseline) pairs')
    summ[tf] = dict(moves=np.mean(mv))

# D74 per n
pr('\nD74 -- gantry share (trav+fold) and fold share of the critical gantry, mean per n:')
for tf in TF:
    d = E1[tf]
    if not d:
        continue
    row = []
    for n in (4, 6, 8, 10):
        s = np.array([shares(r['exact_dec']) for r in d.values() if r['n'] == n])
        if len(s):
            row.append(f'n{n}: gantry {(s[:,0]+s[:,1]).mean()*100:.1f} (min {(s[:,0]+s[:,1]).min()*100:.1f}) fold {s[:,1].mean()*100:.1f}')
    pr(f'  t_fold {tf:g}: ' + ' | '.join(row))
if 0.0 in summ and 50.8 in summ:
    pr(f'\nD73 -- mean critical moves FISIK / 0 = {summ[50.8]["moves"]:.3f} / '
       f'{summ[0.0]["moves"]:.3f} = {summ[50.8]["moves"]/summ[0.0]["moves"]*100:.1f} %')
if 126.8 in summ:
    pr(f'       DINDING / 0 = {summ[126.8]["moves"]/summ[0.0]["moves"]*100:.1f} %')

# total moves per instance, and how often exact never leaves p0
pr('\nmoves distribution (total over gantries) of the exact optimum:')
for tf in TF:
    d = E1[tf]
    if d:
        tm = [tot_moves(r['exact_dec']) for r in d.values()]
        pr(f'  t_fold {tf:g}: ' + ', '.join(f'{k} moves: {tm.count(k)}' for k in sorted(set(tm))))

# ----------------------------------------------------------------- E2
pr('\n' + '=' * 100)
pr('E2 -- Part II (n 12..50, 2 gantries): pose-tour gantry share, bracket UB/LB')
pr('=' * 100)
for tf in TF:
    d = load('g21_e2', tf)
    if not d:
        continue
    have = sum('lb_subset' in r for r in d.values())
    pr(f'\n--- t_fold {tf:g}: {len(d)} instances, LB_subset on {have} ---')
    for n in (12, 16, 20, 30, 50):
        rs = [r for r in d.values() if r['n'] == n]
        ts = np.array([r['sched']['pose-tour']['traverse_share'] for r in rs]) * 100
        ub = np.array([min(r['sched'][k]['makespan'] for k in NAMES
                           if r['sched'][k]['valid']) for r in rs])
        lb = np.array([max(r['lb_analytic'], r.get('lb_subset', 0.0)) for r in rs])
        st = np.mean([sum(r['sched']['pose-tour']['n_stops'].values()) / 2 for r in rs])
        pr(f'  n {n:>2}: gantry share pose-tour mean {ts.mean():5.1f} % (min {ts.min():5.1f},'
           f' max {ts.max():5.1f})  stops/gantry {st:.2f}  UB/LB mean {np.mean(ub/lb):.3f}'
           f' max {np.max(ub/lb):.3f}  UB<LB {int((ub < lb - 1e-9).sum())}')
    pt = {k: r['sched']['pose-tour']['makespan'] for k, r in d.items()}
    pr('  vs pose-tour: ' + ', '.join(
        f'{n} {np.mean([d[k]["sched"][n]["makespan"]/pt[k]-1 for k in d])*100:+.1f} %'
        for n in NAMES[1:]))
    pr('  K4 fails: ' + ', '.join(f'{n} {sum(not r["sched"][n]["valid"] for r in d.values())}'
                                  for n in NAMES))

# ----------------------------------------------------------------- E3
pr('\n' + '=' * 100)
pr('E3 -- stage ablation (g8 B6 set, 40 instances)')
pr('=' * 100)
for tf in TF:
    d = load('g21_e3', tf)
    if not d:
        continue
    base = np.array([r['var']['full']['makespan'] for r in d.values()])
    inv = sum(not v['valid'] for r in d.values() for v in r['var'].values())
    row = []
    for v in ('nn-only', 'cover-order', 'no-refine', 'neither'):
        x = np.array([r['var'][v]['makespan'] for r in d.values()])
        pen = (x / base - 1) * 100
        row.append(f'{v} {pen.mean():+.2f}/{pen.max():+.2f} % worse {int((pen > 1e-9).sum())}/{len(pen)}')
    pr(f'  t_fold {tf:g} ({len(d)}; invalid {inv}): ' + ' | '.join(row))

# ----------------------------------------------------------------- E4
pr('\n' + '=' * 100)
pr('E4 -- mutex on/off (n 6/8, 80 pairs)')
pr('=' * 100)
for tf in TF:
    d = load('g21_e4', tf)
    if not d:
        continue
    c = np.array([r['mutex'] - r['nomutex'] for r in d.values()])
    inv = sum(not r['valid'] for r in d.values())
    pr(f'  t_fold {tf:g}: {len(d)} pairs, cost > 0 on {int((c > 1e-9).sum())}, '
       f'max {c.max():.4f} s, mean {c.mean():.4f} s, invalid {inv}')

(HERE / 'g21_tables.txt').write_text('\n'.join(out) + '\n')
