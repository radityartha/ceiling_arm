"""G31 A4: G26 instances with gantry ROTATION, P1' + rot_cmd, lazy-exact oracle'''. OFFLINE.

Pose set per gantry = REF.lin x Rset, Rset = {0} (R0, = G26), |rot| <= 10 (R10), |rot| <= 35 (R35, B3 rule (3):
both gantries <= 35 deg is safe for ANY combination and any dx with arms REST -> envelope holds by construction).

reach at rot 0 = the G24 oracle''' cache (as G26). reach at rot != 0 starts as L1 (a SUPERSET of oracle''') and
is verified LAZILY: solve, run oracle''' (g29_oracle_rot._job, ok3) on every rot != 0 (node, g, pose, slot) the
schedule USES, zero the failures, solve again -- until every used tuple is verified. Exact: oracle''' implies L1,
so a relaxation optimum that is entirely feasible is the true optimum.

    python3 g31_sched.py run      -> g31_candidates.json, g31_oracle_rot_cache.jsonl
"""
import json
import math
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g26'))
sys.path.insert(0, os.path.join(R, 'p1_g29'))
import g26_make as GM  # noqa: E402
import g29_oracle_rot as GO  # noqa: E402

RS, M, MI, sched = GM.RS, GM.M, GM.MI, GM.sched
sched_coll = GM.sched_coll
REF = M.REF
ROT_CMD = 0.9235                     # A3: median G30 t_rot / T_cmd, 8 legs
SEEDS = [1, 2, 13, 16, 17, 18, 20, 21, 22, 23, 26, 29, 33, 35, 36, 40, 42]
RDEG = np.degrees(REF.rot)
RSETS = {'R0': [int(np.argmin(np.abs(RDEG)))]}
RSETS['R10'] = [i for i, r in enumerate(RDEG) if abs(r) <= 10 + 1e-9]
RSETS['R35'] = [i for i, r in enumerate(RDEG) if abs(r) <= 35 + 1e-9]
CACHE = os.path.join(HERE, 'g31_oracle_rot_cache.jsonl')
NR = len(REF.rot)
NEIGH = 14                           # extra tuples verified per used one (B: post-lock speed-up)


def load_rot_cache():
    c = {}
    for p in (GO.CACHE, CACHE):          # g29 entries: same function, same predicate (K0 72/72)
        if os.path.exists(p):
            for ln in open(p):
                d = json.loads(ln)
                c[tuple(d['key'])] = d
    return c


def build(row, cache0, crot, rset):
    """Instance with poses REF.lin x rset; reach rot 0 = oracle''' cache, rot != 0 = cached ok3 else L1."""
    nodes = np.array(row['nodes'])
    cols = [li * NR + ri for li in range(len(REF.lin)) for ri in rset]
    full = np.stack(np.meshgrid(REF.lin, REF.rot, indexing='ij'), -1).reshape(-1, 2)
    poses = full[cols]
    reach, zone, hand, unver = {}, {}, {}, {}
    for g in (1, 2):
        r, z, h = sched._gantry_oracles(M.CAPS[g], nodes)
        r, z, h = r[:, cols, :].copy(), z[:, cols, :], h[:, cols]
        unver[g] = np.zeros_like(r)
        for i, n in enumerate(row['nodes']):
            for j, c in enumerate(cols):
                li, ri = divmod(c, NR)
                for s in (0, 1):
                    if not r[i, j, s]:
                        continue
                    if abs(RDEG[ri]) < 1e-9:
                        r[i, j, s] = cache0[(n, g, li, s)]['ok']
                    elif (n, g, li, ri, s) in crot:
                        r[i, j, s] = crot[(n, g, li, ri, s)]['ok3']
                    else:
                        unver[g][i, j, s] = True
        reach[g], zone[g], hand[g] = r, z, h
    p0 = int(np.flatnonzero((np.abs(poses[:, 0]) < 1e-9) & (np.abs(poses[:, 1]) < 1e-9))[0])
    inst = sched.Instance(np.array(['SR'] * 6, dtype='<U2'), {g: poses for g in (1, 2)}, reach, zone, hand,
                          {g: p0 for g in (1, 2)}, REF.nodes[nodes], sched.DWELL, 0.0,
                          f'g31(seed={row["seed"]},{len(rset)}rot)', dict(nodes=nodes.tolist(), cols=cols))
    return replace(RS.p1prime(inst), rot_cmd=ROT_CMD), unver


def used_unverified(inst, sol, unver):
    out = []
    for g, stops in sol.stops.items():
        for st in stops:
            for i, who in st['assign'].items():
                s = sched.GANTRY_ARMS[g].index(who)
                if unver[g][i, st['pose'], s]:
                    li, ri = divmod(inst.meta['cols'][st['pose']], NR)
                    out.append((int(inst.meta['nodes'][i]), g, li, ri, s))
    return sorted(set(out))


def neighbours(inst, unver, used, k=NEIGH):
    """Up to k still-unverified tuples of the same (node, g, slot) nearest (index units) to each used one.
    Verifying more than the DP used only reveals truth sooner -- exactness is the termination test."""
    out = set(used)
    nodes = inst.meta['nodes']
    for (n, g, li, ri, s) in used:
        i = nodes.index(n)
        cand = []
        for j in np.flatnonzero(unver[g][i, :, s]):
            lj, rj = divmod(inst.meta['cols'][j], NR)
            cand.append((abs(lj - li) + abs(rj - ri), (n, g, lj, rj, s)))
        out |= {t for _, t in sorted(cand)[:k]}
    return sorted(out)


def solve_lazy(row, cache0, crot, rset, pool, log):
    it = 0
    while True:
        it += 1
        inst, unver = build(row, cache0, crot, rset)
        sol = sched.solve_exact(inst)
        used = used_unverified(inst, sol, unver)
        log(f'    iterasi {it}: makespan {sol.makespan:.3f}, tuple rot!=0 terpakai belum terverifikasi {len(used)}')
        if not used:
            return inst, sol, it
        todo = neighbours(inst, unver, used)
        with open(CACHE, 'a') as f:
            for d in pool.imap_unordered(GO._job, todo, chunksize=1):
                crot[tuple(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()
        log(f'      oracle\'\'\' terpakai ok3 {sum(crot[t]["ok3"] for t in used)}/{len(used)}; dihitung {len(todo)}, '
            f'ok3 {sum(crot[t]["ok3"] for t in todo)} (ok4 {sum(crot[t]["ok4"] for t in todo)})')


def describe(inst, sol):
    dec = {}
    for g, stops in sol.stops.items():
        cur, mv, rots, rotmoves = inst.p0[g], 0, [], 0
        for st in stops:
            if st['pose'] != cur:
                mv += 1
                rotmoves += int(inst.poses[g][st['pose'], 1] != inst.poses[g][cur, 1])
            rots.append(round(float(np.degrees(inst.poses[g][st['pose'], 1])), 1))
            cur = st['pose']
        dec[g] = dict(moves=mv, rot_moves=rotmoves, rot_deg=rots, ntask=bin(sol.assign[g]).count('1'))
    return dec


def run():
    cache0, crot = M.load_cache(), load_rot_cache()
    cand = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g26/g26_candidates.json')))}
    rows = []
    with Pool(15) as pool:
        for seed in SEEDS:
            row = cand[seed]
            rec = dict(seed=seed, nodes=row['nodes'], p0={'1': 0.0, '2': 0.0}, p0_rot_deg={'1': 0.0, '2': 0.0},
                       rot_cmd=ROT_CMD)
            for name, rset in RSETS.items():
                print(f'seed {seed} {name}', flush=True)
                inst, sol, it = solve_lazy(row, cache0, crot, rset, pool, lambda s: print(s, flush=True))
                dec = describe(inst, sol)
                conf = sched_coll.schedule_conflict(inst, sol.stops)
                used = [(n, g, li, ri, s) for g, sts in sol.stops.items() for st in sts
                        for i, who in st['assign'].items()
                        for n, li, ri, s in [(int(inst.meta['nodes'][i]), *divmod(inst.meta['cols'][st['pose']], NR),
                                              sched.GANTRY_ARMS[g].index(who))] if abs(RDEG[ri]) > 1e-9]
                rec[name] = dict(makespan=sol.makespan, serial=sum(sol.finish.values()), iters=it,
                                 moves={str(g): v['moves'] for g, v in dec.items()},
                                 rot_moves={str(g): v['rot_moves'] for g, v in dec.items()},
                                 rot_deg={str(g): v['rot_deg'] for g, v in dec.items()},
                                 ntask={str(g): v['ntask'] for g, v in dec.items()},
                                 conflict=repr(conf), ok_ii=all(v['moves'] >= 1 for v in dec.values()),
                                 used_rot_tuples=len(used), used_rot_ok4=sum(bool(crot[t]['ok4']) for t in used),
                                 schedule={str(g): v for g, v in MI.sched_json(inst, sol).items()})
                if name == 'R35':
                    mut = sched.solve_exact(replace(inst, rot_cmd=0.0))       # K-G3e: rot_cmd ignored
                    rec['K_G3e_motor_makespan'] = mut.makespan
            a, b, c = rec['R0']['makespan'], rec['R10']['makespan'], rec['R35']['makespan']
            print(f"seed {seed:2d}: P1' R0 {a:8.3f}  R10 {b:8.3f}  R35 {c:8.3f}  pindah R0 "
                  f"{sum(rec['R0']['moves'].values())} R35 {sum(rec['R35']['moves'].values())}; rot R35 "
                  f"{rec['R35']['rot_deg']}; konflik {rec['R35']['conflict']}", flush=True)
            rows.append(rec)
            json.dump(rows, open(os.path.join(HERE, 'g31_candidates.json'), 'w'), indent=1, default=float)
    g26 = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g26/g26_candidates.json')))}
    k = sum(r['R0']['makespan'] == g26[r['seed']]['makespan']["P1'"] for r in rows)
    print(f"\nkontrol R0 == G26 P1' (makespan): {k}/{len(rows)}")
    for name in ('R10', 'R35'):
        imp = [r['seed'] for r in rows if r[name]['makespan'] < r['R0']['makespan'] - 1e-9]
        print(f"{name} < R0 pada {len(imp)}/{len(rows)} seed {imp}; total gain "
              f"{sum(r['R0']['makespan'] - r[name]['makespan'] for r in rows):.2f} s")
    mut = [r['seed'] for r in rows if r['K_G3e_motor_makespan'] != r['R35']['makespan']]
    print(f"K-G3e mutan rot_cmd diabaikan (motor T_rot): makespan berubah pada {len(mut)}/{len(rows)} {mut}")


if __name__ == '__main__':
    if sys.argv[1:] == ['run']:
        run()
