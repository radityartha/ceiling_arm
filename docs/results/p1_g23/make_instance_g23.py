"""G23 A4: g22 A2 (i)-(iii) VERBATIM, with reach := reach AND ORACLE' (B0.1).

Everything except the mask is imported from p1_g22/make_instance.py, so the
rule is the same code. OFFLINE, zero hardware.

    python3 make_instance_g23.py [--p0-lin1 0.55] [--p0-lin2 0.0] [--fallback]
-> g23_candidates.json (same row format as g22_candidates.json)
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
import make_instance as MI  # noqa: E402
from make_instance import sched, sched_coll  # noqa: E402

import oracle as O  # noqa: E402

SLOT_ARM = {(1, 0): 'arm_1', (1, 1): 'arm_2', (2, 0): 'arm_3', (2, 1): 'arm_4'}   # sched.GANTRY_ARMS


def oracle_mask(inst, nodes, orc, margin):
    """reach[g][i, p, s] &= ORACLE'(node_i, lin_p, arm(g, s)). Returns new Instance + counts."""
    ok = O.torq_all(orc['taus'], margin)                       # (node, lin, arm)
    ni = {int(n): k for k, n in enumerate(orc['nodes'])}
    arms = list(orc['arms'])
    reach, cnt = {}, {}
    for g in inst.gantries:
        r = inst.reach[g].copy()
        li = [int(np.argmin(abs(orc['lins'] - L))) for L in inst.poses[g][:, 0]]
        assert all(abs(orc['lins'][k] - L) < 1e-9 for k, L in zip(li, inst.poses[g][:, 0]))
        before = int(r.sum())
        for i, n in enumerate(nodes):
            for s in (0, 1):
                r[i, :, s] &= ok[ni[n], li, arms.index(SLOT_ARM[(g, s)])]
        reach[g] = r
        cnt[g] = (before, int(r.sum()))
    return sched.Instance(inst.kind, inst.poses, reach, inst.zone, inst.hand, inst.p0,
                          inst.xyz, inst.dwell, inst.t_fold, inst.label + "+oracle'",
                          dict(inst.meta)), cnt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--p0-lin1', type=float, default=0.55)
    ap.add_argument('--p0-lin2', type=float, default=0.00)
    ap.add_argument('--seeds', default='0-49')
    ap.add_argument('--fallback', action='store_true')
    ap.add_argument('--out', default=os.path.join(HERE, 'g23_candidates.json'))
    a = ap.parse_args()
    orc = dict(np.load(os.path.join(HERE, 'g23_oracle.npz')))
    margin = np.array(json.load(open(os.path.join(HERE, 'g23_calib2.json')))['margin'])
    lo, hi = (int(x) for x in a.seeds.split('-'))
    p0_lin = {1: a.p0_lin1, 2: a.p0_lin2}
    rows = []
    for seed in range(lo, hi + 1):
        base = sched.gen_real(6, seed, 0, (1, 2), maps=MI.MAPS)
        inst, cnt = oracle_mask(MI.restrict_rot0(base, p0_lin), base.meta['nodes'], orc, margin)
        row = dict(seed=seed, nodes=base.meta['nodes'],
                   reach_true_L1_to_oracle={str(g): v for g, v in cnt.items()})
        try:
            sols = {k: sched.solve_exact(MI.with_tfold(inst, tf)) for k, tf in MI.T_FOLD.items()}
        except ValueError as e:
            dead = [i for i in range(len(inst.kind))
                    if not any(inst.reach[g][i].any() for g in inst.gantries)]
            row.update(ok_i=False, why=str(e), infeasible_tasks=dead)
            rows.append(row)
            print(f'seed {seed:2d}: (i) GAGAL -- tugas tanpa pose layak-oracle {dead}')
            continue
        f = sols['FISIK']
        dec = {k: MI.decompose(inst, s, MI.T_FOLD[k]) for k, s in sols.items()}
        mv = {g: dec['FISIK'][g]['moves'] for g in (1, 2)}
        ntask = {g: bin(f.assign[g]).count('1') for g in (1, 2)}
        ok_ii = all(mv[g] >= 1 for g in (1, 2)) if not a.fallback else \
            (sum(mv.values()) >= 1 and all(ntask[g] >= 1 for g in (1, 2)))
        conf = sched_coll.schedule_conflict(inst, f.stops)
        same = all([(s['pose'], s['tasks']) for s in sols['FISIK'].stops[g]] ==
                   [(s['pose'], s['tasks']) for s in sols['DINDING'].stops[g]] for g in (1, 2))
        row.update(ok_i=True, ok_ii=bool(ok_ii) and conf is None, conflict=repr(conf),
                   moves=mv, ntask=ntask, same_schedule_FISIK_DINDING=same,
                   makespan={k: s.makespan for k, s in sols.items()},
                   decomp={k: {str(g): v for g, v in d.items()} for k, d in dec.items()},
                   schedule_FISIK={str(g): v for g, v in MI.sched_json(inst, f).items()},
                   schedule_DINDING={str(g): v for g, v in MI.sched_json(inst, sols['DINDING']).items()})
        rows.append(row)
        print(f'seed {seed:2d}: pindah g1/g2 {mv[1]}/{mv[2]}, tugas {ntask[1]}/{ntask[2]}, '
              f'makespan FISIK {f.makespan:7.2f} DINDING {sols["DINDING"].makespan:7.2f}, '
              f'jadwal sama {same}, konflik {conf!r}  (ii) {"LOLOS" if row["ok_ii"] else "-"}')
    json.dump(rows, open(a.out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r.get('ok_ii')]
    print(f'\n(i) {sum(r["ok_i"] for r in rows)}/{len(rows)}; lolos (i)-(iii): {len(ok)} seed: {ok}\n  -> {a.out}')


if __name__ == '__main__':
    main()
