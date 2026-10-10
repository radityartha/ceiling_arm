"""G42 (docs/p1_g42_joint_runner.md A3-A4): G41b + (s42) = (iv'') as a LAZY stop constraint; diagnosis of the seeds
whose (i) fails. OFFLINE, no ROS.

(s42) for a stop configuration c = (g, li, ri, slot-0 nodes in task order, slot-1 nodes in task order): from both arms
REST at the pose, the stop's task events in g31_screen.events order, every transit = straight cosine joint line (61 pts)
from the arm's simulated state to its (c41b) goal, partner as simulated; then the straight both-arm retract. Each line:
EnvChecker CLEAR, same-gantry CLEAR, box >= BOX_MIN, torque (torq2 rule) = g41_make.chain WITHOUT cross-gantry, base
G.base_joints(g, lin, rot). Stops with one task are (e2) of (c41b) exactly and are not computed.

    python3 g42_make.py kstop  -> positive control (G41 s19) + every >= 2-task stop of the 5 G41b seeds
    python3 g42_make.py run    -> g42_candidates.json (+ g42_stop_cache.jsonl, g42_tuple_cache.jsonl, g42_edge_cache)
    python3 g42_make.py iv | chain   -> (iv') / (iv'') of the G42 candidates
    python3 g42_make.py diag   -> g42_diag.json (A4)
"""
import itertools
import json
import math
import os
import random
import shutil
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g41'))
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
import g41_make as G41  # noqa: E402

G41.set_b()                                           # (c41b): box margin 10 mm everywhere
G4 = G41.G4
G, S, MI, sched = G4.G, G4.S, G4.MI, G4.sched
O2, REF, NR, SLOT_ARM, MARGIN, PREFIX = G4.O2, G4.REF, G4.NR, G4.SLOT_ARM, G4.MARGIN, G4.PREFIX
REST, LIM, OFFSET, gp = G41.REST, G41.LIM, G41.OFFSET, G4.gp
VAR = 'G42'
STOP_CACHE = os.path.join(HERE, 'g42_stop_cache.jsonl')
TUPLE_CACHE = os.path.join(HERE, 'g42_tuple_cache.jsonl')     # copy of the G41b cache, new tuples appended here
EDGE_CACHE = os.path.join(HERE, 'g42_edge_cache.jsonl')
NPROC = 8                                                      # mock planning runs next to this (A2)
FIX_CHOICES = False                    # G42f (B3.2): gantry choice per task from ANY finite subset containing it


def _copy_caches():
    if not os.path.exists(TUPLE_CACHE):
        shutil.copy(os.path.join(R, 'p1_g41/g41b_tuple_cache.jsonl'), TUPLE_CACHE)
    if not os.path.exists(EDGE_CACHE):
        shutil.copy(G41.EDGE_CACHE, EDGE_CACHE)
    G41.TUPLE_CACHE = TUPLE_CACHE
    G4.EDGE_CACHE = EDGE_CACHE


# ------------------------------------------------------------------------------------------------------------ (s42)
def _screen(g, names, pts, st, ec, bx):
    """g41_make.chain's screen() without the cross-gantry part. (ok, dict)."""
    held = {n: x for n, x in st.items() if n not in names}
    ve = ec.screen_trajectory(names, pts, st)
    vs = G41.same_checker(g).screen_trajectory(names, pts, held)
    bmin = math.inf
    for q in pts:
        j = dict(st)
        j.update(zip(names, q))
        bmin = min(bmin, bx.dist(ec, ec.q_from(j), gp(g))[0])
    tau = -math.inf
    for a in sorted({n[:6] for n in names}):
        arm = [k for k, p in PREFIX.items() if p == a][0]
        ix = [names.index(f'{a}joint_{i}') for i in range(1, 7)]
        for q in pts:
            tau = max(tau, float(np.max(O2.gravity(arm, np.array([q[i] for i in ix]), st[f't{g}_linear_joint'],
                                                   st[f't{g}_rotation_joint']) + OFFSET + MARGIN - LIM)))
    ok = ve[0] == vs[0] == 'CLEAR' and bmin >= max(G41.BOX_MIN, 1e-12) and tau <= 0.0
    return ok, dict(env=ve[0], env_d=round(float(ve[1]), 5), same=vs[0], same_d=round(float(vs[1]), 5),
                    box_d=round(float(bmin), 5), tau_excess=round(tau, 4), ok=ok)


def _stop_job(arg):
    """(s42) for one stop configuration. goals = {(node, slot): q6}."""
    from return_rest import _rest_line
    t0 = time.process_time()
    key, goals = arg
    g, li, ri, seq0, seq1 = key
    ec, bx = G4.checkers()
    st = G.base_joints(g, float(REF.lin[li]), float(REF.rot[ri]))
    seqs = (list(seq0), list(seq1))
    lines, ok = [], True
    f = [0.5 * (1.0 - math.cos(math.pi * i / 60)) for i in range(0, 61)]
    for j in range(max(len(s) for s in seqs)):
        for s in (0, 1):
            if j >= len(seqs[s]) or not ok:
                continue
            n = seqs[s][j]
            p = PREFIX[SLOT_ARM[(g, s)]]
            names = [f'{p}joint_{i}' for i in range(1, 7)]
            s0 = np.array([st[x] for x in names])
            q1 = np.array(goals[(n, s)])
            frm = 'REST' if np.max(np.abs(s0 - REST)) < math.radians(0.5) else 'ujung tugas sebelumnya'
            ok_, d = _screen(g, names, [list(s0 + (q1 - s0) * x) for x in f], st, ec, bx)
            lines.append(dict(d, what=f'node {n} slot {s} dari {frm}'))
            ok &= ok_
            st.update(zip(names, map(float, q1)))
    if ok:
        names = [f'{PREFIX[SLOT_ARM[(g, s)]]}joint_{i}' for s in (0, 1) for i in range(1, 7)]
        cur = [st[x] for x in names]
        if max(abs(c - x) for c, x in zip(cur, list(REST) * 2)) >= math.radians(0.5):
            ok_, d = _screen(g, names, _rest_line(cur), st, ec, bx)
            lines.append(dict(d, what='retract kedua lengan'))
            ok &= ok_
    return dict(key=[g, li, ri, list(seq0), list(seq1)], ok=bool(ok), lines=lines,
                cpu_s=round(time.process_time() - t0, 2))


def ckey(d):
    g, li, ri, a, b = d
    return (int(g), int(li), int(ri), tuple(int(x) for x in a), tuple(int(x) for x in b))


class Stops:
    def __init__(self):
        self.c = {}
        if os.path.exists(STOP_CACHE):
            for ln in open(STOP_CACHE):
                d = json.loads(ln)
                self.c[ckey(d['key'])] = d

    def compute(self, items, pool):
        items = [(k, gl) for k, gl in items if k not in self.c]
        if not items:
            return
        with open(STOP_CACHE, 'a') as f:
            for d in pool.imap_unordered(_stop_job, items, chunksize=1):
                self.c[ckey(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()

    def forbids(self, inst):
        """{g: {(U, v): [pose idx]}} for the refused configurations whose nodes are in this instance AND whose task order
        is this instance's (transit direction matters)."""
        nodes = [int(x) for x in inst.meta['nodes']]
        col = {c: j for j, c in enumerate(inst.meta['cols'])}
        out = {1: {}, 2: {}}
        for k, d in self.c.items():
            if d['ok']:
                continue
            g, li, ri, a, b = k
            ids = a + b
            if not all(x in nodes for x in ids) or len(set(ids)) != len(ids) or (li * NR + ri) not in col:
                continue
            ti = {x: nodes.index(x) for x in ids}
            if [ti[x] for x in a] != sorted(ti[x] for x in a) or [ti[x] for x in b] != sorted(ti[x] for x in b):
                continue
            U = sum(1 << ti[x] for x in ids)
            slot = {ti[x]: 0 for x in a}
            slot.update({ti[x]: 1 for x in b})
            v = tuple(slot[i] for i in sorted(slot))
            out[g].setdefault((U, v), []).append(col[li * NR + ri])
        return out


def stop_items(inst, sol, F):
    """(config key, goals) of every stop with >= 2 tasks in sol."""
    out = []
    nodes = inst.meta['nodes']
    for g, stops in sol.stops.items():
        for st in stops:
            if len(st['assign']) < 2:
                continue
            li, ri = G4.li_ri(inst, st['pose'])
            seq = ([], [])
            for i in sorted(st['assign']):
                seq[sched.GANTRY_ARMS[g].index(st['assign'][i])].append(int(nodes[i]))
            goals = {(n, s): F.mine[(n, g, li, ri, s)]['goal']['q6'] for s in (0, 1) for n in seq[s]}
            out.append(((g, li, ri, tuple(seq[0]), tuple(seq[1])), goals))
    return out


# ------------------------------------------------------------------------------------------- solver (copies + forbids)
def _dur_table_f(inst, g, forb):
    """sched._dur_table VERBATIM except an (U, v) combination is infeasible at the poses forb[(U, v)] lists."""
    n, P = inst.n, len(inst.poses[g])
    reach, zone, hand = inst.reach[g], inst.zone[g], inst.hand[g]
    is_mr = inst.kind == 'MR'
    dur = np.full((1 << n, P), np.inf)
    dur[0] = 0.0
    for U in range(1, 1 << n):
        idx = [i for i in range(n) if U >> i & 1]
        mr = [i for i in idx if is_mr[i]]
        sr = [i for i in idx if not is_mr[i]]
        feas0 = np.ones(P, bool)
        for i in mr:
            feas0 &= hand[i]
        if not feas0.any():
            continue
        m = len(mr)
        best = np.full(P, np.inf)
        for v in itertools.product((0, 1), repeat=len(sr)):
            feas = feas0.copy()
            zc = np.zeros(P, np.int32)
            for i, a in zip(sr, v):
                feas &= reach[i, :, a]
                zc += zone[i, :, a]
            # --- G42 (s42) ---
            if (U, v) in forb:
                feas[forb[(U, v)]] = False
            # -----------------
            if not feas.any():
                continue
            if inst.serial_arms:
                slots = np.full(P, float(m + len(sr)))
            else:
                load = max(m + v.count(0), m + v.count(1))
                slots = np.maximum(load, m + zc).astype(float)
            np.minimum(best, np.where(feas, slots, np.inf), out=best)
        dur[U] = best * inst.dwell
    return dur


def stop_duration_f(inst, g, U, p, forb):
    """sched.stop_duration VERBATIM except the (s42) forbids."""
    idx = [i for i in range(inst.n) if U >> i & 1]
    mr = [i for i in idx if inst.kind[i] == 'MR']
    sr = [i for i in idx if inst.kind[i] != 'MR']
    if any(not inst.hand[g][i, p] for i in mr):
        return np.inf, None
    m = len(mr)
    best, best_v = np.inf, None
    for v in itertools.product((0, 1), repeat=len(sr)):
        if any(not inst.reach[g][i, p, a] for i, a in zip(sr, v)):
            continue
        if p in forb.get((U, v), ()):                       # G42 (s42)
            continue
        s = [v.count(0), v.count(1)]
        z = sum(inst.zone[g][i, p, a] for i, a in zip(sr, v))
        slots = m + len(sr) if inst.serial_arms else \
            max(m + s[0], m + s[1], m + int(z))
        if slots < best:
            best, best_v = slots, v
    if best_v is None:
        return np.inf, None
    assign = {i: 'both' for i in mr}
    assign.update({i: sched.GANTRY_ARMS[g][a] for i, a in zip(sr, best_v)})
    return best * inst.dwell, assign


def solve_gantry_f(inst, g, E, forb):
    """g40_make.solve_gantry_e VERBATIM except the duration table is _dur_table_f."""
    n = inst.n
    dur_full = _dur_table_f(inst, g, forb)
    usable = np.isfinite(dur_full[1:]).any(axis=0)
    keep = np.flatnonzero(usable)
    p0 = inst.p0[g]
    if p0 not in keep:
        keep = np.sort(np.append(keep, p0))
    r0 = int(np.searchsorted(keep, p0))
    T = sched.traverse_matrix(inst.poses[g][keep], inst.t_fold, inst.rail_cmd, inst.rot_cmd)
    T0 = None if inst.t_fold_first is None else sched.traverse_matrix(
        inst.poses[g][keep], inst.t_fold_first, inst.rail_cmd, inst.rot_cmd)[r0]
    lr = [G4.li_ri(inst, int(j)) for j in keep]
    for a in range(len(keep)):
        for b in range(a + 1, len(keep)):
            if E.get(G4.edge_key(g, lr[a], lr[b])) is False:
                T[a, b] = T[b, a] = np.inf
    if T0 is not None:
        T0 = np.where(np.isinf(T[r0]), np.inf, T0)
    # --- G42 B (K-lazy): two consecutive stops at the SAME pose are one stop for events() (no retract between),
    # so a refused configuration could come back split in two. The first stop may still be at p0 (T0 = row r0);
    # no other self-transition. Without (s42) forbids a merged stop is never worse than a split one (same reach,
    # slots max <= sum), so this changes nothing else.
    if T0 is None:
        T0 = T[r0].copy()
    np.fill_diagonal(T, np.inf)
    # ----------------------------------------
    dur = dur_full[:, keep]
    K = len(keep)
    h = np.full((1 << n, K), np.inf)
    w = np.full((1 << n, K), np.inf)
    h[0] = 0.0
    order = sorted(range(1, 1 << n), key=lambda R_: bin(R_).count('1'))
    for R_ in order:
        wr = w[R_]
        U = R_
        while U:
            np.minimum(wr, dur[U] + h[R_ ^ U], out=wr)
            U = (U - 1) & R_
        cols = np.flatnonzero(np.isfinite(wr))
        if len(cols):
            np.min(T[:, cols] + wr[cols], axis=1, out=h[R_])
    return sched.GantryDP(g, keep, T, dur, h, w, r0, T0)


def _reconstruct_f(inst, dp, A, forb):
    """sched._reconstruct VERBATIM except stop_duration_f."""
    stops, R_, r, t = [], A, dp.r0, 0.0
    while R_:
        wr = dp.w[R_]
        row = dp.T0 if (not stops and dp.T0 is not None) else dp.T[r]
        cand = row + wr
        r2 = int(np.argmin(cand))
        t += row[r2]
        bestU, bestc = None, np.inf
        U = R_
        while U:
            c = dp.dur[U, r2] + dp.h[R_ ^ U, r2]
            if c < bestc:
                bestU, bestc = U, c
            U = (U - 1) & R_
        p = int(dp.keep[r2])
        d, assign = stop_duration_f(inst, dp.g, bestU, p, forb)
        stops.append(dict(pose=p, start=t, dur=d, tasks=bestU, assign=assign))
        t += d
        R_ ^= bestU
        r = r2
    return stops, t


def solve_exact_f(inst, E, forb):
    """g40_make.solve_exact_e VERBATIM except solve_gantry_f / _reconstruct_f."""
    t0 = time.time()
    gs = inst.gantries
    dps = {g: solve_gantry_f(inst, g, E, forb[g]) for g in gs}
    costs = {g: dps[g].cost for g in gs}
    choices = []
    for i in range(inst.n):
        if FIX_CHOICES:                # G42f: with infinite traverse edges task i may need another stop first
            A = np.arange(1 << inst.n)
            ok = [g for g in gs if np.isfinite(costs[g][(A >> i) & 1 == 1]).any()]
        else:
            ok = [g for g in gs if np.isfinite(costs[g][1 << i])]
        if not ok:
            raise ValueError(f'task {i} is infeasible on every gantry')
        choices.append(ok)
    best, best_alloc, n_alloc = np.inf, None, 0
    for combo in itertools.product(*choices):
        n_alloc += 1
        masks = {g: 0 for g in gs}
        for i, g in enumerate(combo):
            masks[g] |= 1 << i
        m = max(costs[g][masks[g]] for g in gs)
        if m < best:
            best, best_alloc = m, masks
    if best_alloc is None or not np.isfinite(best):
        raise ValueError('instance has no feasible schedule')
    stops, finish = {}, {}
    for g in gs:
        stops[g], finish[g] = _reconstruct_f(inst, dps[g], best_alloc[g], forb[g])
    return sched.Solution(float(best), best_alloc, stops, finish, time.time() - t0, n_alloc)


def solve_lazy42(nodes, F, E, SX, pool, log):
    """g40_make.solve_lazy + (s42): stops are verified once the tuples and edges the optimum uses are (goals exist)."""
    it = 0
    while True:
        it += 1
        inst, unver = G4.build(nodes, F)
        forb = SX.forbids(inst)
        sol = solve_exact_f(inst, E, forb)
        ut = S.used_unverified(inst, sol, unver)
        ue = [k for k in G4.used_edges(inst, sol) if E.get(k) is None]
        us = [] if (ut or ue) else [x for x in stop_items(inst, sol, F) if x[0] not in SX.c]
        log(f'    iterasi {it}: makespan {sol.makespan:.3f}, tuple belum terverifikasi {len(ut)}, tepi {len(ue)}, '
            f'stop belum terverifikasi {len(us)}, stop dilarang {sum(len(v) for v in forb.values())}')
        if not ut and not ue and not us:
            return inst, sol, it
        F.compute(S.neighbours(inst, unver, ut) if ut else [], pool)
        E.compute(G4.neighbour_edges(inst, ue, F.pose_ok) if ue else [], pool)
        if us:
            SX.compute(us, pool)
            log('      stop (s42): ' + ', '.join(f"{k} {'ok' if SX.c[k]['ok'] else 'TOLAK'}" for k, _ in us))


def run(tag='g42'):
    _copy_caches()
    out = os.path.join(HERE, f'{tag}_candidates.json')
    F, E, SX = G41.Feas41(G41.pose_ok_b()), G4.Edges(), Stops()
    g41b = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g41/g41b_candidates.json')))}
    rows = []
    log = lambda s: print(s, flush=True)                    # noqa: E731
    with Pool(NPROC) as pool:
        for seed in sorted(g41b):
            rb = g41b[seed]
            rec = {k: rb[k] for k in ('seed', 'drawn', 'nodes', 'kind', 'xyz', 'd_env', 'p0', 'p0_rot_deg', 'rot_cmd',
                                      'g38_makespan', 'g40e_makespan')}
            rec['g41b_makespan'] = rb['G41b'].get('makespan')
            log(f'seed {seed} {VAR}  [{time.strftime("%T")}]')
            t0 = time.time()
            try:
                inst, sol, it = solve_lazy42(rb['nodes'], F, E, SX, pool, log)
            except ValueError as e:
                rec[VAR] = dict(ok_i=False, why=str(e), wall_s=time.time() - t0)
                log(f'  seed {seed}: (i) GAGAL -- {e}')
                rows.append(rec)
                json.dump(rows, open(out, 'w'), indent=1, default=float)
                continue
            dec = S.describe(inst, sol)
            conf = S.sched_coll.schedule_conflict(inst, sol.stops)
            ok_ii = all(v['moves'] >= 1 for v in dec.values())
            sj = {str(g): v for g, v in MI.sched_json(inst, sol).items()}
            for g, gl in G41.goals(inst, sol, F).items():
                for st, gg in zip(sj[str(g)], gl):
                    st['goal'] = gg
            rec[VAR] = dict(ok_i=True, ok_ii=ok_ii, ok_iii=conf is None, makespan=sol.makespan,
                            serial=sum(sol.finish.values()), iters=it, conflict=repr(conf), wall_s=time.time() - t0,
                            moves={str(g): v['moves'] for g, v in dec.items()},
                            ntask={str(g): v['ntask'] for g, v in dec.items()},
                            stops_s42=[dict(key=list(k), ok=SX.c[k]['ok']) for k, _ in stop_items(inst, sol, F)],
                            schedule=sj)
            sjn = json.loads(json.dumps(sj, default=float))
            same = bool(rb['G41b'].get('ok_i')) and all(
                [(s['rail_m'], s['rot_deg'], s['tasks'], s['goal']) for s in sjn[str(g)]] ==
                [(s['rail_m'], s['rot_deg'], s['tasks'], s['goal']) for s in rb['G41b']['schedule'][str(g)]]
                for g in (1, 2))
            rec[VAR]['same_as_g41b'] = bool(same)
            log(f"  seed {seed}: makespan {sol.makespan:.2f} (G41b {rec['g41b_makespan']}), stop sama G41b {same}, "
                f"konflik {conf!r}, (ii) {ok_ii}, iterasi {it}  [{time.strftime('%T')}]")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r[VAR].get('ok_i') and r[VAR]['ok_ii'] and r[VAR]['ok_iii']]
    print(f"\nlolos (i)-(iii) {VAR}: {len(ok)}/{len(rows)} seed {ok}; tuple {len(F.mine)}, tepi {len(E.c)}, "
          f"stop (s42) {len(SX.c)} (tolak {sum(not d['ok'] for d in SX.c.values())})", flush=True)


def kstop():
    """A3 positive control (G41 s19 g2 stop with t3 -> t4) + every >= 2-task stop of the 5 G41b seeds."""
    items = []
    for var, fn in (('G41', 'g41_candidates.json'), ('G41b', 'g41b_candidates.json')):
        for r in json.load(open(os.path.join(R, 'p1_g41', fn))):
            v = r[var]
            if not (v.get('ok_i') and v['ok_ii'] and v['ok_iii']) or (var == 'G41' and r['seed'] != 19):
                continue
            for g in (1, 2):
                for k, st in enumerate(v['schedule'][str(g)]):
                    if len(st['tasks']) < 2:
                        continue
                    li, ri = G4.idx_of(st['rail_m'], st['rot_deg'])
                    seq = ([], [])
                    goals = {}
                    for t in sorted(st['tasks'], key=int):
                        s = sched.GANTRY_ARMS[g].index(st['tasks'][t])
                        n = int(r['nodes'][int(t)])
                        seq[s].append(n)
                        p = PREFIX[SLOT_ARM[(g, s)]]
                        goals[(n, s)] = [st['goal'][t][f'{p}joint_{i}'] for i in range(1, 7)]
                    items.append((f"{var} s{r['seed']} g{g} stop {k} tugas {sorted(st['tasks'], key=int)}",
                                  ((g, li, ri, tuple(seq[0]), tuple(seq[1])), goals)))
    with Pool(NPROC) as pool:
        res = pool.map(_stop_job, [x for _, x in items], chunksize=1)
    for (lab, _), d in zip(items, res):
        print(f"{lab}: {'ok' if d['ok'] else 'TOLAK'} ({d['cpu_s']} s)")
        for ln in d['lines']:
            print(f"    {ln['what']}: env {ln['env']} {ln['env_d'] * 1000:.1f}, se {ln['same']} {ln['same_d'] * 1000:.1f}, "
                  f"kotak {ln['box_d'] * 1000:.2f} mm, torsi lebih {ln['tau_excess']:+.2f} -> "
                  f"{'ok' if ln['ok'] else 'TOLAK'}")
    pos = [d['ok'] for (lab, _), d in zip(items, res) if lab.startswith('G41 ')]
    neg = [d['ok'] for (lab, _), d in zip(items, res) if lab.startswith('G41b ')]
    print(f'kontrol positif G41 s19: tolak {sum(not x for x in pos)}/{len(pos)}; G41b stop >= 2 tugas diterima '
          f'{sum(neg)}/{len(neg)}')


# --------------------------------------------------------------------------------------------------- diagnosis (A4)
DIAG_TORQ = os.path.join(HERE, 'g42_diag_torque.jsonl')
DIAG_C41 = os.path.join(HERE, 'g42_diag_c41.jsonl')
DIAG_CAP = 600


def _torque_job(key):
    n, g, li, ri, s = key
    r = O2.solve(G4.X[n], float(REF.lin[li]), SLOT_ARM[(g, s)], tilt=True, envelope=True, keep_solutions=True,
                 rot=float(REF.rot[ri]))
    return dict(key=[int(v) for v in key], ok3=bool(O2.torq2(r, MARGIN)), taumax=r['taumax'].tolist(),
                n_sol=r['n_sol'])


def _c41_job(key):
    """G41._tuple_job's branch loop WITHOUT the torque gate (A4 marginal): (c41b) alone."""
    import g41_make as G41_
    t0 = time.process_time()
    n, g, li, ri, s = key
    arm, lin, rot = SLOT_ARM[(g, s)], float(REF.lin[li]), float(REF.rot[ri])
    r = O2.solve(G4.X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ec, bx = G4.checkers()
    base, p = G.base_joints(g, lin, rot), PREFIX[arm]
    order = sorted(range(len(r['solutions'])), key=lambda i: (float(np.max(np.abs(r['solutions'][i][1] - REST))), i))
    ok, whys = False, {}
    for i in order:
        q6 = r['solutions'][i][1]
        j = dict(base)
        j.update({f'{p}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
        q = ec.q_from(j)
        d, _ = ec.check(q, only=(gp(g),))
        if d < G4.ENV_MIN:
            whys['e1-env'] = whys.get('e1-env', 0) + 1
            continue
        b, _ = bx.dist(ec, q, gp(g))
        if b <= 0.0 or b < G41_.BOX_MIN:
            whys['e1-kotak'] = whys.get('e1-kotak', 0) + 1
            continue
        sl = G41_.screen_line(g, arm, lin, rot, q6, ec, bx)
        if not sl['ok']:
            whys[sl['why']] = whys.get(sl['why'], 0) + 1
            continue
        ok = True
        break
    return dict(key=[int(v) for v in key], ok41=ok, whys=whys, n_sol=r['n_sol'],
                cpu_s=round(time.process_time() - t0, 2))


def _load(path):
    out = {}
    if os.path.exists(path):
        for ln in open(path):
            d = json.loads(ln)
            out[tuple(d['key'])] = d
    return out


def _compute(path, job, keys, pool):
    have = _load(path)
    keys = sorted(k for k in set(keys) if k not in have)
    with open(path, 'a') as f:
        for d in pool.imap_unordered(job, keys, chunksize=1):
            have[tuple(d['key'])] = d
            f.write(json.dumps(d) + '\n')
            f.flush()
    return have


def diag(count_only=False, tag='g42'):
    """A4: for every task of a (i)-failing seed that no gantry can serve -- the L1 tuples, classified
    L1 -> pose (a) -> exact torque -> (c41b), plus the marginals."""
    _copy_caches()
    pose_ok = G41.pose_ok_b()
    F, E = G41.Feas41(pose_ok), G4.Edges()
    rows = [r for r in json.load(open(os.path.join(HERE, f'{tag}_candidates.json'))) if not r[VAR].get('ok_i')]
    tq = _load(DIAG_TORQ)
    c4 = _load(DIAG_C41)

    def torque(key):
        for src in (F.mine, F.g40, F.g41, tq):
            if key in src:
                return bool(src[key]['ok3'])
        return F.exact_torque(key)

    def c41(key):
        """(True/False/None, why dict)"""
        d = F.mine.get(key)
        if d is not None and d['ok3']:
            return bool(d['ok41']), d['whys']
        d = F.g40.get(key)
        if d is not None and d['ok3'] and not d['env2']:
            return False, {'e1 (G40 env2)': 1}
        d = F.g41.get(key)
        if d is not None and d['ok3'] and not d['ok41']:
            return False, d['whys']
        d = c4.get(key)
        if d is not None:
            return bool(d['ok41']), d['whys']
        return None, {}

    tasks = []
    for r in rows:
        nodes = r['nodes']
        inst, unver = G4.build(nodes, F)
        cols = inst.meta['cols']
        costs = {g: solve_gantry_f(inst, g, E, {}).cost for g in (1, 2)}
        A = np.arange(1 << len(nodes))
        for i, n in enumerate(nodes):
            if any(np.isfinite(costs[g][(A >> i) & 1 == 1] if FIX_CHOICES else costs[g][1 << i]).any()
                   for g in (1, 2)):
                continue
            l1 = []
            for g in (1, 2):
                raw, _, _ = sched._gantry_oracles(G4.G.M.CAPS[g], np.array(nodes))
                raw = raw[:, cols, :]
                for j, s in np.argwhere(raw[i]):
                    li, ri = divmod(cols[j], NR)
                    l1.append((int(n), g, int(li), int(ri), int(s)))
            filt = int(sum(inst.reach[g][i].sum() for g in (1, 2)))
            tasks.append(dict(seed=r['seed'], task=i, node=int(n), kind=r['kind'][i], xyz=r['xyz'][i], l1=l1,
                              n_reach_after_filters=filt))
    need_t = [k for t in tasks for k in t['l1'] if torque(k) is None]
    print(f'diag: {len(rows)} seed gagal, {len(tasks)} tugas tak layak, tuple L1 {sum(len(t["l1"]) for t in tasks)}, '
          f'torsi belum diketahui {len(need_t)}')
    if count_only:
        return
    with Pool(NPROC) as pool:
        tq.update(_compute(DIAG_TORQ, _torque_job, need_t, pool))
        cand = sorted({k for t in tasks for k in t['l1'] if c41(k)[0] is None and
                       (pose_ok[k[1]][k[2], k[3]] != torque(k))})        # exactly one of pose / torque fails
        if len(cand) > DIAG_CAP:
            random.Random(42).shuffle(cand)
            print(f'  (c41b) marginal: {len(cand)} kandidat > {DIAG_CAP} -> sampel acak {DIAG_CAP} (seed 42)')
            cand = sorted(cand[:DIAG_CAP])
        # B3.1: the pose-fail AND torque-ok stratum (the only one that can show "fails ONLY pose") in FULL
        s1 = sorted({k for t in tasks for k in t['l1'] if c41(k)[0] is None and not pose_ok[k[1]][k[2], k[3]]
                     and torque(k)})
        print(f'  strata pose gagal & torsi ok: {len(s1)} (sudah di sampel {len(set(s1) & set(cand))}) -> penuh')
        cand = sorted(set(cand) | set(s1))
        print(f'  (c41b) marginal dihitung: {len(cand)}  [{time.strftime("%T")}]', flush=True)
        c4.update(_compute(DIAG_C41, _c41_job, cand, pool))
    out = []
    for t in tasks:
        cl = dict(L1=len(t['l1']), pose_fail=0, torque_fail=0, c41_fail=0, ok_all=0, c41_unknown=0)
        why, only_pose, only_torque, only_c41, pose_ok_n, torq_ok_n = {}, 0, 0, 0, 0, 0
        for k in t['l1']:
            pz, tz = bool(pose_ok[k[1]][k[2], k[3]]), torque(k)
            cz, wz = c41(k)
            pose_ok_n += pz
            torq_ok_n += bool(tz)
            if not pz:
                cl['pose_fail'] += 1
            elif not tz:
                cl['torque_fail'] += 1
            elif cz is None:
                cl['c41_unknown'] += 1
            elif not cz:
                cl['c41_fail'] += 1
                for w, v in wz.items():
                    why[w] = why.get(w, 0) + v
            else:
                cl['ok_all'] += 1
            only_pose += (not pz) and bool(tz) and cz is True
            only_torque += pz and (tz is False) and cz is True
            only_c41 += pz and bool(tz) and cz is False
        out.append(dict({k: v for k, v in t.items() if k != 'l1'}, hier=cl, c41_why=why, pose_ok=pose_ok_n,
                        torque_ok=torq_ok_n, only_pose=only_pose, only_torque=only_torque, only_c41=only_c41))
        print(f"s{t['seed']} t{t['task']} node {t['node']} {t['kind']} {t['xyz']}: L1 {cl['L1']} | pose gagal "
              f"{cl['pose_fail']}, torsi gagal {cl['torque_fail']}, (c41b) gagal {cl['c41_fail']} {why}, belum "
              f"{cl['c41_unknown']}, lolos {cl['ok_all']} | marginal: pose-ok {pose_ok_n}, torsi-ok {torq_ok_n}; "
              f"HANYA pose {only_pose}, HANYA torsi {only_torque}, HANYA (c41b) {only_c41}", flush=True)
    json.dump(out, open(os.path.join(HERE, f'{tag}_diag.json'), 'w'), indent=1)


def iv(cand):
    """(iv') g41_make.iv copied, on `cand` (serial cross-gantry traverses, the walk order with --final-retract)."""
    import warnings
    sys.path.insert(0, os.path.join(R, 'p1_g29'))
    sys.path.insert(0, os.path.join(R, 'p1_g31'))
    import g31_screen as G31S
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        import g29_rot_screen as RS
        chk = RS.RotCrossChecker()
    res = {}
    for r in json.load(open(os.path.join(HERE, cand))):
        v = r[VAR]
        if not (v.get('ok_i') and v['ok_ii'] and v['ok_iii']):
            continue
        p0 = {g: r['p0'][str(g)] for g in (1, 2)}
        st = {'t1_linear_joint': p0[1], 't2_linear_joint': p0[2], 't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
        for p in PREFIX.values():
            st.update({f'{p}joint_{i}': float(x) for i, x in enumerate(REST, 1)})
        out = []
        for e in G31S.events(v['schedule'], p0, final_retract=True):
            if e['kind'] != 'traverse':
                continue
            g = e['gantry']
            sw = RS.sweep_rot(chk, g, e['frm'], e['to'], dict(st))
            out.append(sw['verdict'])
            print(f"  s{r['seed']} g{g} ({e['frm'][0]:.2f},{math.degrees(e['frm'][1]):+.0f})->({e['to'][0]:.2f},"
                  f"{math.degrees(e['to'][1]):+.0f}): {sw['verdict']} {sw['d'] * 1000:.1f} mm {sw['pair']}")
            st[f't{g}_linear_joint'], st[f't{g}_rotation_joint'] = e['to']
        res[r['seed']] = all(x == 'CLEAR' for x in out)
    print(f"(iv') lolos {sum(res.values())}/{len(res)} {res}")


if __name__ == '__main__':
    if sys.argv[1].endswith('_f'):
        FIX_CHOICES = True
    {'kstop': kstop, 'run': run, 'run_f': lambda: run('g42f'), 'diag': diag, 'diag_count': lambda: diag(True),
     'diag_f': lambda: diag(tag='g42f'), 'iv_f': lambda: iv('g42f_candidates.json'),
     'chain_f': lambda: G41.chain(VAR, os.path.join(HERE, 'g42f_candidates.json'))}[sys.argv[1]]()
