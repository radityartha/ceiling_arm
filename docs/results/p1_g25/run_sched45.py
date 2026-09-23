"""G25 step 3b -- the 45 G24 (i)-(iii) instances under the measured cost model P1'
(docs/p1_g25_task_cost.md A2, constants g25_constants.json) and the secondary P1'-par.
Everything in A4 items 1-6. Run only after C0' and G0' passed.

    python3 run_sched45.py      -> g25_sched45.json + table on stdout
"""
import itertools
import json
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g24'))
import make_instance_g24 as M  # noqa: E402
import g0_gate as G  # noqa: E402
from calibrate import t_cmd  # noqa: E402

MI, sched = M.MI, M.sched
C = json.load(open(os.path.join(HERE, 'g25_constants.json')))


def p1prime(inst, serial=True):
    return replace(inst, dwell=C['c_task'], t_fold=C['t_fold'], t_fold_first=C['t_fold_first'],
                   rail_cmd=C['r'], serial_arms=serial)


def shape(inst, stops):
    """Per gantry: moves, retracts, rail seconds (r*T_cmd), tasks."""
    out = {}
    for g, st in stops.items():
        cur, mv, ret, rail, n = inst.p0[g], 0, 0, 0.0, 0
        for k, s in enumerate(st):
            if s['pose'] != cur:
                mv += 1
                ret += int(k > 0)          # a retract only after a stop (arms off REST)
                rail += C['r'] * t_cmd(inst.poses[g][s['pose'], 0] - inst.poses[g][cur, 0])
            n += bin(s['tasks']).count('1')
            cur = s['pose']
        out[g] = dict(moves=mv, retracts=ret, rail=rail, tasks=n)
    return out


def sig(sol):
    return {g: [(int(s['pose']), int(s['tasks'])) for s in st] for g, st in sol.stops.items()}


def m_min_total(inst):
    """A4.4: fewest total pose changes over every feasible schedule (sum over gantries)."""
    x = replace(inst, dwell=1e-6, t_fold=1e4)     # dwell 0 -> inf*0 = NaN in _dur_table
    costs = {g: sched.solve_gantry(x, g).cost for g in x.gantries}
    mv = {g: np.where(np.isfinite(c), np.floor(c / 1e4 + 0.5), np.inf) for g, c in costs.items()}
    best = np.inf
    for combo in itertools.product(x.gantries, repeat=x.n):
        masks = {g: sum(1 << i for i in range(x.n) if combo[i] == g) for g in x.gantries}
        best = min(best, sum(mv[g][masks[g]] for g in x.gantries))
    return int(best)


def run():
    cache = M.load_cache()
    cand = json.load(open(os.path.join(R, 'p1_g24/g24_candidates.json')))
    rows = []
    for row in cand:
        if not row.get('ok_ii'):
            continue
        inst, _ = M.instance(row['seed'], row['nodes'], row['rejects'], cache)
        old = sched.solve_exact(MI.with_tfold(inst, MI.T_FOLD['FISIK']))
        assert old.makespan == row['makespan']['FISIK']
        rec = dict(seed=row['seed'], m_min=m_min_total(inst), P1=old.makespan)
        for name, serial in (("P1'", True), ("P1'-par", False)):
            x = p1prime(inst, serial)
            new = sched.solve_exact(x)
            rep_new = G.eval_schedule(x, new.stops)
            assert all(abs(rep_new[g] - new.finish[g]) < 1e-9 for g in x.gantries)
            rep_old = G.eval_schedule(x, old.stops)
            old_ms = max(rep_old.values())
            sh_new, sh_old = shape(inst, new.stops), shape(inst, old.stops)
            crit = max(new.finish, key=new.finish.get)
            sc = sh_new[crit]
            rec[name] = dict(
                opt=new.makespan, old_sched=old_ms, regret_s=old_ms - new.makespan,
                regret_pct=100 * (old_ms - new.makespan) / new.makespan,
                old_not_opt=bool(old_ms > new.makespan + 1e-6),
                same_sched=sig(new) == sig(old),
                moves={str(g): v['moves'] for g, v in sh_new.items()},
                moves_old={str(g): v['moves'] for g, v in sh_old.items()},
                ntask={str(g): v['tasks'] for g, v in sh_new.items()},
                ntask_old={str(g): v['tasks'] for g, v in sh_old.items()},
                serial_sum=sum(new.finish.values()), old_serial_sum=sum(rep_old.values()),
                crit=crit, share_crit=dict(
                    tasks=sc['tasks'] * x.dwell if serial else None,
                    retract=sc['retracts'] * C['c_ret'], overhead=sc['moves'] * C['c_rovh'],
                    rail=sc['rail'], finish=new.finish[crit]),
                sched={str(g): v for g, v in MI.sched_json(x, new).items()})
        rows.append(rec)
        a = rec["P1'"]
        print(f"seed {rec['seed']:2d}  P1 {rec['P1']:7.2f}  P1' {a['opt']:7.2f}  lama@P1' {a['old_sched']:7.2f} "
              f"regret {a['regret_pct']:5.1f}%  pindah {sum(a['moves_old'].values())}->{sum(a['moves'].values())} "
              f"(m_min {rec['m_min']})  n1/n2 {a['ntask_old']['1']}/{a['ntask_old']['2']}->"
              f"{a['ntask']['1']}/{a['ntask']['2']}  same {a['same_sched']}", flush=True)
    json.dump(rows, open(os.path.join(HERE, 'g25_sched45.json'), 'w'), indent=1, default=float)
    return rows


def summary(rows):
    n = len(rows)
    print(f'\n== {n} instance')
    for name in ("P1'", "P1'-par"):
        A = [r[name] for r in rows]
        tot_new = [sum(a['moves'].values()) for a in A]
        tot_old = [sum(a['moves_old'].values()) for a in A]
        mm = [r['m_min'] for r in rows]
        imb_new = [abs(a['ntask']['1'] - a['ntask']['2']) for a in A]
        imb_old = [abs(a['ntask_old']['1'] - a['ntask_old']['2']) for a in A]
        reg = np.array([a['regret_pct'] for a in A])
        print(f"{name}: lama tidak optimum {sum(a['old_not_opt'] for a in A)}/{n}; jadwal identik "
              f"{sum(a['same_sched'] for a in A)}/{n}; regret mean {reg.mean():.2f} % median "
              f"{np.median(reg):.2f} % maks {reg.max():.2f} % ({max(a['regret_s'] for a in A):.1f} s)")
        print(f"   pindah total mean lama {np.mean(tot_old):.3f} -> baru {np.mean(tot_new):.3f}; "
              f"= m_min: baru {sum(t == m for t, m in zip(tot_new, mm))}/{n}, lama "
              f"{sum(t == m for t, m in zip(tot_old, mm))}/{n}; m_min mean {np.mean(mm):.3f}")
        print(f"   |n1-n2| mean lama {np.mean(imb_old):.3f} -> baru {np.mean(imb_new):.3f}; "
              f"pindah naik/sama/turun {sum(a > b for a, b in zip(tot_new, tot_old))}/"
              f"{sum(a == b for a, b in zip(tot_new, tot_old))}/{sum(a < b for a, b in zip(tot_new, tot_old))}")
        if name == "P1'":
            s = {k: np.mean([a['share_crit'][k] / a['share_crit']['finish'] for a in A])
                 for k in ('tasks', 'retract', 'overhead', 'rail')}
            print('   pangsa gantry kritis: ' + ', '.join(f'{k} {v * 100:.1f} %' for k, v in s.items()))
            print(f"   makespan mean P1 {np.mean([r['P1'] for r in rows]):.2f} -> P1' "
                  f"{np.mean([a['opt'] for a in A]):.2f} (x{np.mean([a['opt'] / r['P1'] for a, r in zip(A, rows)]):.2f})")
    s1 = next(r for r in rows if r['seed'] == 1)
    a = s1["P1'"]
    print(f"\nseed 1: P1 {s1['P1']:.2f}; P1' opt {a['opt']:.2f}, jadwal G24b@P1' {a['old_sched']:.2f} "
          f"(serial Σ_g {a['old_serial_sum']:.2f}); jadwal sama {a['same_sched']}")


if __name__ == '__main__':
    summary(run())
