"""G32 A1: seed 36 R10 rescheduled with a rot-0 TIE-BREAK. OFFLINE.

Instance = g31_sched.build(row, R10) unchanged (P1' + rot_cmd 0.9235, lazy-exact oracle''', G31 cache reused).
Tie-break = lexicographic cost: sched.traverse_matrix is wrapped HERE (sched.py untouched) to add EPS for every
move INTO a rot != 0 pose (T and T0 both come from traverse_matrix). The true makespan is recomputed without EPS
from the schedule itself.

    python3 g32_sched.py      -> g32_candidates.json, g32_sched.log (stdout)
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g31'))
import g31_sched as S  # noqa: E402

sched, MI = S.sched, S.MI
SEED = 36
EPS = 1e-6
_tm = sched.traverse_matrix


def tm_eps(poses, t_fold=0.0, rail_cmd=0.0, rot_cmd=0.0):
    T = _tm(poses, t_fold, rail_cmd, rot_cmd)
    to_rot = np.abs(poses[:, 1]) > 1e-9                      # column = destination pose
    return T + np.where((T > 0) & to_rot[None, :], EPS, 0.0)


def true_makespan(inst, sol):
    """Per gantry: sum of traverse (no EPS) + stop durations, from p0. Max over gantries."""
    fin = {}
    for g, stops in sol.stops.items():
        cur, t, first = inst.p0[g], 0.0, True
        for st in stops:
            p = st['pose']
            if p != cur:
                tf = inst.t_fold_first if (first and inst.t_fold_first is not None) else inst.t_fold
                d = inst.poses[g][p] - inst.poses[g][cur]
                t += float(sched.traverse_time(d[0], d[1], tf, inst.rail_cmd, inst.rot_cmd))
            first = False
            t += st['dur']
            cur = p
        fin[g] = t
    return max(fin.values()), fin


def min_gap(inst):
    """Smallest non-zero difference between distinct traverse/stop costs (scale for EPS)."""
    vals = set()
    for g in (1, 2):
        T = _tm(inst.poses[g], inst.t_fold, inst.rail_cmd, inst.rot_cmd)
        vals |= set(np.round(T[T > 0], 9).tolist())
    v = np.array(sorted(vals | {round(inst.dwell, 9)}))
    return float(np.min(np.diff(v)))


def main():
    cache0, crot = S.M.load_cache(), S.load_rot_cache()
    row = {r['seed']: r for r in json.load(open(os.path.join(S.R, 'p1_g26/g26_candidates.json')))}[SEED]
    g31 = {r['seed']: r for r in json.load(open(os.path.join(S.R, 'p1_g31/g31_candidates.json')))}[SEED]
    rec = dict(seed=SEED, nodes=row['nodes'], p0={'1': 0.0, '2': 0.0}, p0_rot_deg={'1': 0.0, '2': 0.0},
               rot_cmd=S.ROT_CMD, eps=EPS)
    with Pool(15) as pool:
        for name, rset, eps in (('R0', S.RSETS['R0'], False), ('R10_noeps', S.RSETS['R10'], False),
                                ('R10', S.RSETS['R10'], True)):
            sched.traverse_matrix = tm_eps if eps else _tm
            print(f'seed {SEED} {name}', flush=True)
            inst, sol, it = S.solve_lazy(row, cache0, crot, rset, pool, lambda s: print(s, flush=True))
            sched.traverse_matrix = _tm
            ms, fin = true_makespan(inst, sol)
            dec = S.describe(inst, sol)
            rec[name] = dict(makespan=ms, makespan_dp=sol.makespan, finish={str(g): v for g, v in fin.items()},
                             iters=it, moves={str(g): v['moves'] for g, v in dec.items()},
                             rot_moves={str(g): v['rot_moves'] for g, v in dec.items()},
                             rot_deg={str(g): v['rot_deg'] for g, v in dec.items()},
                             ntask={str(g): v['ntask'] for g, v in dec.items()},
                             conflict=repr(S.sched_coll.schedule_conflict(inst, sol.stops)),
                             schedule={str(g): v for g, v in MI.sched_json(inst, sol).items()})
            print(f'  {name}: makespan sejati {ms:.6f} (DP {sol.makespan:.9f}), iterasi {it}, '
                  f'rot {rec[name]["rot_deg"]}, pindah {rec[name]["moves"]}', flush=True)
            if name == 'R10':
                gap = min_gap(inst)
                nrot = sum(rec[name]['rot_moves'].values())
                print(f'  selisih biaya bukan-nol terkecil {gap:.6f} s; ε total maks {EPS * 2 * len(inst.poses[1]):.2e} '
                      f'(ε·pindah berotasi terpakai {EPS * nrot:.1e})', flush=True)
                rec['min_cost_gap'] = gap
    sched.traverse_matrix = _tm
    rt = json.loads(json.dumps(rec['R10_noeps']['schedule'], default=float))
    k0 = rec['R10_noeps']['makespan_dp'] == g31['R10']['makespan'] and rt == g31['R10']['schedule']
    print(f"K-S0 tanpa ε == G31 R10 (makespan {g31['R10']['makespan']} + jadwal): {k0}")
    d1 = abs(rec['R10']['makespan'] - rec['R10_noeps']['makespan'])
    print(f"K-S1 |sejati ε − tanpa ε| = {d1:.3e} -> {'LULUS' if d1 <= 1e-9 else 'GAGAL'}")
    print(f"R0 {rec['R0']['makespan']:.6f} G31 R0 {g31['R0']['makespan']:.6f}")
    rec['K_S0'], rec['K_S1'] = k0, d1
    json.dump([rec], open(os.path.join(HERE, 'g32_candidates.json'), 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
