"""G40 (docs/p1_g40_env_sched.md A1-A3): ENVIRONMENT-AWARE scheduler for the G38 hard instances. OFFLINE, no ROS.

Model M40, per gantry g (geometry = every t{g}_ body, the other gantry left out -- it holds still and its own held
state passed its own predicate):
  (a) pose (lin, rot) feasible <=> both arms REST: EnvChecker.check >= 50 mm AND no grown MoveIt box hit (precomputed,
      all 33 x 72 poses)
  (b) traverse p -> p' feasible <=> EnvChecker.screen_trajectory over g29 rect_points(p, p'), arms REST, = CLEAR;
      an EDGE constraint, T = inf in a copy of sched.solve_gantry; verified lazily, cached symmetric
  (c) tuple feasible <=> L1 AND G39 table ok AND (a) AND exact torq2 AND some oracle IK solution with partner REST
      is >= 50 mm (EnvChecker) AND hits no grown box; lazily verified, cached
Lazy loop: unknown tuple / edge = feasible (relaxation ⊇ feasible), solve, verify what the optimum USES, repeat until
it uses nothing unverified -> optimum of M40 (direct traverses between stops only, see A2 for what is NOT exact).

    python3 g40_make.py k          -> K1 (traverse verdicts of the G38 plan-only logs), K2 (torque bit-identical)
    python3 g40_make.py poses      -> g40_pose_table.json
    python3 g40_make.py explain    -> (c) on the 114 tuples the G38 R35 schedules used + confusion vs the G38 logs
    python3 g40_make.py run        -> g40_candidates.json (+ caches), log on stdout
"""
import json
import math
import os
import re
import sys
import time
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
for d in ('p1_g38', 'p1_g39', 'p1_g29'):
    sys.path.insert(0, os.path.join(R, d))
import g38_make as G  # noqa: E402
import g39_table as T39  # noqa: E402

S, M, MI, sched = G.S, G.M, G.MI, G.sched
O2, REF, NR, RDEG, SLOT_ARM, MARGIN, PREFIX = G.O2, G.REF, G.NR, G.RDEG, G.SLOT_ARM, G.MARGIN, G.PREFIX
X, ENV_MIN, P0_LIN = G.X, G.ENV_MIN, G.P0_LIN
NL = len(REF.lin)
RSET = S.RSETS['R35']
VAR = 'G40'
POSE_TABLE = os.path.join(HERE, 'g40_pose_table.json')
TUPLE_CACHE = os.path.join(HERE, 'g40_tuple_cache.jsonl')
EDGE_CACHE = os.path.join(HERE, 'g40_edge_cache.jsonl')
NPROC = 15

_W = {}


def checkers():
    """(EnvChecker, BoxChecker), one per process."""
    if not _W:
        _W['env'] = G.env_checker()
        _W['box'] = BoxChecker()
    return _W['env'], _W['box']


class BoxChecker:
    """The MoveIt scene object: env_static_map_pub.column_boxes (grown 5 cm per side). min coal distance of the
    t{g}_ true hulls to the grown boxes near them; <= 0 = penetrating = MoveIt collision (hulls contain the meshes)."""

    def __init__(self):
        import env_collision as E
        from env_static_map_pub import column_boxes
        c, s = column_boxes(E.load_env_map()['centers'].astype(float))
        self.c, self.s = c, s
        self.lo, self.hi = c - s / 2, c + s / 2

    def dist(self, ec, q, only):
        import coal
        oMg = ec.place(q)
        best, worst = math.inf, None
        req = coal.DistanceRequest()
        for i, g in enumerate(ec.geom.geometryObjects):
            if not g.name.startswith(only):
                continue
            Mi = oMg[i]
            Rm, t = np.array(Mi.rotation), np.array(Mi.translation)
            lo, hi = ec.world_aabb(i, Mi)
            cand = np.flatnonzero(((self.lo <= hi + 1e-3) & (self.hi >= lo - 1e-3)).all(1))
            Tg = coal.Transform3s(Rm, t)
            for k in cand:
                res = coal.DistanceResult()
                d = coal.distance(g.geometry, Tg, coal.Box(*self.s[k]),
                                  coal.Transform3s(np.eye(3), self.c[k]), req, res)
                if d < best:
                    best, worst = d, g.name
        return best, worst


def gp(g):
    return f't{g}_'


# ---------------------------------------------------------------------------------------------------------- (a) poses
def _pose_job(key):
    g, li, ri = key
    ec, bx = checkers()
    q = ec.q_from(G.base_joints(g, float(REF.lin[li]), float(REF.rot[ri])))
    d, wg = ec.check(q, only=(gp(g),))
    b, wb = bx.dist(ec, q, gp(g))
    return dict(key=[g, li, ri], d_env=round(float(d), 5), geom=wg,
                d_box=None if b == math.inf else round(float(b), 5), geom_box=wb,
                ok=bool(d >= ENV_MIN and b > 0.0))


def poses():
    keys = [(g, li, ri) for g in (1, 2) for li in range(NL) for ri in range(NR)]
    t = time.time()
    with Pool(NPROC) as p:
        rows = p.map(_pose_job, keys, chunksize=8)
    json.dump(rows, open(POSE_TABLE, 'w'))
    print(f'pose: {len(rows)} dalam {time.time() - t:.0f} s')
    pose_report(rows)


def pose_report(rows):
    ok = load_pose_ok(rows)
    r0 = int(np.argmin(np.abs(RDEG)))
    for g in (1, 2):
        a = ok[g][:, RSET]
        env = np.array([[d['d_env'] >= ENV_MIN for d in rows if d['key'][0] == g]]).reshape(NL, NR)[:, RSET]
        print(f'g{g}: R35 layak {int(a.sum())}/{a.size} ({100 * a.mean():.1f} %), hanya EnvChecker {int(env.sum())}; '
              f'rot 0 {int(ok[g][:, r0].sum())}/{NL}; semua 72 rot {int(ok[g].sum())}/{ok[g].size}')
        for ri in RSET:
            bad = [round(float(REF.lin[li]), 2) for li in range(NL) if not ok[g][li, ri]]
            print(f'   rot {RDEG[ri]:+5.0f}: tak layak {len(bad):2d} {bad}')
    for g in (1, 2):
        li = int(np.argmin(np.abs(REF.lin - P0_LIN[g])))
        print(f'p0 g{g} ({P0_LIN[g]}, 0): layak {bool(ok[g][li, r0])}')


def load_pose_ok(rows=None):
    rows = rows if rows is not None else json.load(open(POSE_TABLE))
    ok = {g: np.zeros((NL, NR), bool) for g in (1, 2)}
    for d in rows:
        g, li, ri = d['key']
        ok[g][li, ri] = d['ok']
    return ok


# ------------------------------------------------------------------------------------------------------ (b) traverse
def edge_key(g, a, b):
    """Canonical (symmetric) key: a, b = (li, ri)."""
    a, b = tuple(int(v) for v in a), tuple(int(v) for v in b)
    return (g,) + (a + b if a <= b else b + a)


def _edge_job(key):
    import g29_rot_screen as RS29
    g, l0, r0, l1, r1 = key
    ec, _ = checkers()
    frm, to = (float(REF.lin[l0]), float(REF.rot[r0])), (float(REF.lin[l1]), float(REF.rot[r1]))
    pts = RS29.rect_points(frm, to)
    v, d, wg, k = ec.screen_trajectory([f't{g}_linear_joint', f't{g}_rotation_joint'], pts,
                                       G.base_joints(g, *frm), only=(gp(g),))
    return dict(key=list(key), verdict=v, d=round(float(d), 5), geom=wg, n=len(pts))


class Edges:
    def __init__(self):
        self.c = {}
        if os.path.exists(EDGE_CACHE):
            for ln in open(EDGE_CACHE):
                d = json.loads(ln)
                self.c[tuple(d['key'])] = d

    def get(self, key):
        return None if key not in self.c else self.c[key]['verdict'] == 'CLEAR'

    def compute(self, keys, pool):
        keys = sorted({k for k in keys if k not in self.c})
        if not keys:
            return
        with open(EDGE_CACHE, 'a') as f:
            for d in pool.imap_unordered(_edge_job, keys, chunksize=1):
                self.c[tuple(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()


# --------------------------------------------------------------------------------------------------------- (c) tuples
def _tuple_job_full(key):
    return _tuple_job(key, full=True)


def _tuple_job(key, full=False):
    """full=False (run): stop at the first solution passing env2 (counts are then partial, n_checked says how
    many solutions were looked at). full=True (explain): every solution, plus the G38 task-arm-only predicate."""
    n, g, li, ri, s = key
    arm, lin, rot = SLOT_ARM[(g, s)], float(REF.lin[li]), float(REF.rot[ri])
    r = O2.solve(X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ok3 = bool(O2.torq2(r, MARGIN))
    n_env = n_box = n_both = n_chk = 0
    best_env = best_box = -math.inf
    arm_only_env = False                          # the G38 predicate (task arm only, Euclid), for comparison
    if ok3:
        ec, bx = checkers()
        base, p = G.base_joints(g, lin, rot), PREFIX[arm]
        for _, q6 in r['solutions']:
            j = dict(base)
            j.update({f'{p}joint_{i}': float(v) for i, v in enumerate(q6, 1)})
            q = ec.q_from(j)
            d, _ = ec.check(q, only=(gp(g),))
            b, _ = bx.dist(ec, q, gp(g))
            if full:
                da, _ = ec.check(q, only=(p,))
                arm_only_env |= da >= ENV_MIN
            n_chk += 1
            best_env, best_box = max(best_env, d), max(best_box, b)
            n_env += d >= ENV_MIN
            n_box += b > 0.0
            n_both += (d >= ENV_MIN) and (b > 0.0)
            if n_both and not full:
                break
    rnd = lambda v: None if v == -math.inf else (1e9 if v == math.inf else round(float(v), 5))  # noqa: E731
    return dict(key=[int(v) for v in key], n_sol=r['n_sol'], saturated=bool(r['saturated']),
                taumax=r['taumax'].tolist(), ok3=ok3, n_env=int(n_env), n_box=int(n_box), n_both=int(n_both),
                env_best_m=rnd(best_env), box_best_m=rnd(best_box), n_checked=int(n_chk), full=full,
                g38_env=bool(arm_only_env) if full else None, env2=bool(n_both > 0))


class Feas2:
    """(c): False if table / pose says no; True/False once computed; None = unknown (relaxation: feasible)."""

    def __init__(self, pose_ok, torque='table'):
        self.pose_ok, self.torque = pose_ok, torque
        if torque == 'exact':                     # B (G40e): G38's exact torque caches, table NOT used
            f38 = G.Feas()
            self.t0, self.trot, self.m38 = f38.t0, f38.trot, f38.mine
        done, ok = T39.table_arrays(T39.load_table())
        self.tab = (done, ok)
        self.mine = {}
        if os.path.exists(TUPLE_CACHE):
            for ln in open(TUPLE_CACHE):
                d = json.loads(ln)
                self.mine[tuple(d['key'])] = d

    def table_ok(self, keys):
        k = np.asarray(keys)
        return T39.lookup(T39.rel_of(k[:, 0], k[:, 1], k[:, 2], k[:, 3], k[:, 4]), *self.tab)

    def exact_torque(self, key):
        """Known exact oracle''' torque verdict (G24 rot 0 / G29+G31 / G38 caches) or None."""
        if key in self.m38:
            return bool(self.m38[key]['ok3'])
        n, g, li, ri, s = key
        d = self.t0.get((n, g, li, s)) if ri == G.ROT0 else self.trot.get(key)
        return None if d is None else bool(d['ok'] if ri == G.ROT0 else d['ok3'])

    def get(self, key, tab):
        if self.torque == 'table' and not tab:
            return False
        if not self.pose_ok[key[1]][key[2], key[3]]:
            return False
        if key in self.mine:
            d = self.mine[key]
            return bool(d['ok3'] and d['env2'])
        if self.torque == 'exact' and self.exact_torque(key) is False:
            return False
        return None

    def compute(self, keys, pool, full=False):
        keys = sorted({k for k in keys if k not in self.mine or (full and not self.mine[k].get('full'))})
        if not keys:
            return
        with open(TUPLE_CACHE, 'a') as f:
            for d in pool.imap_unordered(_tuple_job_full if full else _tuple_job, keys, chunksize=1):
                self.mine[tuple(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()


# ------------------------------------------------------------------------------------------------- instance + solver
def build(nodes, F):
    """g38_make.build with (a) + (c): reach = L1 AND table AND pose AND (verified env2 | unknown)."""
    nodes = np.array(nodes)
    cols = [li * NR + ri for li in range(NL) for ri in RSET]
    full = np.stack(np.meshgrid(REF.lin, REF.rot, indexing='ij'), -1).reshape(-1, 2)
    poses_ = full[cols]
    reach, zone, hand, unver = {}, {}, {}, {}
    for g in (1, 2):
        r, z, h = sched._gantry_oracles(M.CAPS[g], nodes)
        r, z, h = r[:, cols, :].copy(), z[:, cols, :], h[:, cols]
        unver[g] = np.zeros_like(r)
        idx = np.argwhere(r)
        keys = [(int(nodes[i]), g, *divmod(cols[j], NR), int(s)) for i, j, s in idx]
        tab = F.table_ok(keys) if keys else []
        for (i, j, s), key, t in zip(idx, keys, tab):
            f = F.get(key, t)
            if f is None:
                unver[g][i, j, s] = True
            else:
                r[i, j, s] = f
        reach[g], zone[g], hand[g] = r, z, h
    p0 = {g: int(np.flatnonzero((np.abs(poses_[:, 0] - P0_LIN[g]) < 1e-9) & (np.abs(poses_[:, 1]) < 1e-9))[0])
          for g in (1, 2)}
    inst = sched.Instance(np.array(['SR'] * len(nodes), dtype='<U2'), {g: poses_ for g in (1, 2)}, reach, zone,
                          hand, p0, X[nodes], sched.DWELL, 0.0, f'g40({len(RSET)}rot)',
                          dict(nodes=nodes.tolist(), cols=cols))
    return replace(S.RS.p1prime(inst), rot_cmd=S.ROT_CMD), unver


def li_ri(inst, j):
    return divmod(inst.meta['cols'][j], NR)


def solve_gantry_e(inst, g, E):
    """sched.solve_gantry VERBATIM except T / T0 = inf on edges E says are infeasible (b)."""
    n = inst.n
    dur_full = sched._dur_table(inst, g)
    usable = np.isfinite(dur_full[1:]).any(axis=0)
    keep = np.flatnonzero(usable)
    p0 = inst.p0[g]
    if p0 not in keep:
        keep = np.sort(np.append(keep, p0))
    r0 = int(np.searchsorted(keep, p0))
    T = sched.traverse_matrix(inst.poses[g][keep], inst.t_fold, inst.rail_cmd, inst.rot_cmd)
    T0 = None if inst.t_fold_first is None else sched.traverse_matrix(
        inst.poses[g][keep], inst.t_fold_first, inst.rail_cmd, inst.rot_cmd)[r0]
    # --- G40 (b) ---
    lr = [li_ri(inst, int(j)) for j in keep]
    for a in range(len(keep)):
        for b in range(a + 1, len(keep)):
            if E.get(edge_key(g, lr[a], lr[b])) is False:
                T[a, b] = T[b, a] = np.inf
    if T0 is not None:
        T0 = np.where(np.isinf(T[r0]), np.inf, T0)
    # ---------------
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


def solve_exact_e(inst, E):
    """sched.solve_exact VERBATIM except the gantry DP is solve_gantry_e."""
    t0 = time.time()
    gs = inst.gantries
    dps = {g: solve_gantry_e(inst, g, E) for g in gs}
    costs = {g: dps[g].cost for g in gs}
    choices = []
    for i in range(inst.n):
        ok = [g for g in gs if np.isfinite(costs[g][1 << i])]
        if not ok:
            raise ValueError(f'task {i} is infeasible on every gantry')
        choices.append(ok)
    best, best_alloc, n_alloc = np.inf, None, 0
    import itertools
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
        stops[g], finish[g] = sched._reconstruct(inst, dps[g], best_alloc[g])
    return sched.Solution(float(best), best_alloc, stops, finish, time.time() - t0, n_alloc)


def used_edges(inst, sol):
    out = []
    for g, stops in sol.stops.items():
        cur = inst.p0[g]
        for st in stops:
            if st['pose'] != cur:
                out.append(edge_key(g, li_ri(inst, cur), li_ri(inst, st['pose'])))
            cur = st['pose']
    return sorted(set(out))


def neighbour_edges(inst, ue, pose_ok, d=2):
    """For each used unverified edge a-b, also the edges a-b' and a'-b with b' / a' within d grid steps (lin and rot)
    of b / a and pose-feasible. Verifying more than the DP used only reveals truth sooner (exactness = termination)."""
    out = set(ue)
    ris = list(RSET)
    for (g, la, ra, lb, rb) in ue:
        for (l0, r0), (l1, r1) in (((la, ra), (lb, rb)), ((lb, rb), (la, ra))):
            k1 = ris.index(r1)
            for dl in range(-d, d + 1):
                for dk in range(-d, d + 1):
                    l2, k2 = l1 + dl, k1 + dk
                    if 0 <= l2 < NL and 0 <= k2 < len(ris) and pose_ok[g][l2, ris[k2]] and (l2, ris[k2]) != (l0, r0):
                        out.add(edge_key(g, (l0, r0), (l2, ris[k2])))
    return sorted(out)


def solve_lazy(nodes, F, E, pool, log):
    it = 0
    while True:
        it += 1
        inst, unver = build(nodes, F)
        sol = solve_exact_e(inst, E)                        # ValueError = (i) fails
        ut = S.used_unverified(inst, sol, unver)
        ue = [k for k in used_edges(inst, sol) if E.get(k) is None]
        log(f'    iterasi {it}: makespan {sol.makespan:.3f}, tuple belum terverifikasi {len(ut)}, tepi {len(ue)}')
        if not ut and not ue:
            return inst, sol, it
        F.compute(S.neighbours(inst, unver, ut) if ut else [], pool)
        E.compute(neighbour_edges(inst, ue, F.pose_ok) if ue else [], pool)
        if ut:
            log(f'      tuple terpakai lolos (c) {sum(F.get(t, True) is True for t in ut)}/{len(ut)}')
        if ue:
            log(f'      tepi terpakai CLEAR {sum(E.get(k) for k in ue)}/{len(ue)} '
                + ' '.join(f"{E.c[k]['verdict']}:{E.c[k]['d'] * 1000:.0f}" for k in ue if not E.get(k)))


def run(torque='table'):
    var = VAR if torque == 'table' else VAR + 'e'
    out = os.path.join(HERE, f'{var.lower()}_candidates.json')
    pose_ok = load_pose_ok()
    F, E = Feas2(pose_ok, torque), Edges()
    g38 = json.load(open(os.path.join(R, 'p1_g38/g38_candidates.json')))
    rows = []
    log = lambda s: print(s, flush=True)                    # noqa: E731
    with Pool(NPROC) as pool:
        for r38 in g38:
            seed, nodes = r38['seed'], r38['nodes']
            rec = dict(seed=seed, drawn=True, nodes=nodes, kind=r38['kind'], xyz=r38['xyz'], d_env=r38['d_env'],
                       p0={str(g): v for g, v in P0_LIN.items()}, p0_rot_deg={'1': 0.0, '2': 0.0}, rot_cmd=S.ROT_CMD,
                       g38_makespan=r38.get('R35', {}).get('makespan'))
            log(f'seed {seed} {var}  [{time.strftime("%T")}]')
            t0 = time.time()
            try:
                inst, sol, it = solve_lazy(nodes, F, E, pool, log)
            except ValueError as e:
                rec[var] = dict(ok_i=False, why=str(e), wall_s=time.time() - t0)
                log(f'  seed {seed}: (i) GAGAL -- {e}')
                rows.append(rec)
                json.dump(rows, open(out, 'w'), indent=1, default=float)
                continue
            dec = S.describe(inst, sol)
            conf = S.sched_coll.schedule_conflict(inst, sol.stops)
            ok_ii = all(v['moves'] >= 1 for v in dec.values())
            rec[var] = dict(ok_i=True, ok_ii=ok_ii, ok_iii=conf is None, makespan=sol.makespan,
                            serial=sum(sol.finish.values()), iters=it, conflict=repr(conf), wall_s=time.time() - t0,
                            moves={str(g): v['moves'] for g, v in dec.items()},
                            rot_deg={str(g): v['rot_deg'] for g, v in dec.items()},
                            ntask={str(g): v['ntask'] for g, v in dec.items()},
                            served={str(k): v for k, v in G.served(inst, sol, nodes).items()},
                            edges=[dict(key=list(k), **{x: E.c[k][x] for x in ('verdict', 'd', 'geom')})
                                   for k in used_edges(inst, sol)],
                            schedule={str(g): v for g, v in MI.sched_json(inst, sol).items()})
            log(f"  seed {seed}: makespan {sol.makespan:.2f} (G38 {rec['g38_makespan']}), pindah {rec[var]['moves']}, "
                f"rot {rec[var]['rot_deg']}, tugas {rec[var]['ntask']}, konflik {conf!r}, (ii) {ok_ii}, "
                f"iterasi {it}  [{time.strftime('%T')}]")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r[var].get('ok_i') and r[var]['ok_ii'] and r[var]['ok_iii']]
    print(f"\nlolos (i)-(iii) {var}: {len(ok)}/{len(rows)} seed {ok}; tuple {len(F.mine)}, tepi {len(E.c)}", flush=True)


# ------------------------------------------------------------------------------------------------------------ checks
TRAV = re.compile(r'\[(\d+)\] traverse g(\d) \(([\d.]+), ([+-]\d+)\) -> \(([\d.]+), ([+-]\d+)\): (\S+)')
TENV = re.compile(r'\[(\d+)\] traverse g(\d) vs lingkungan: (\S+) min (-?[\d.]+) mm \((\S+)\)')


def idx_of(lin, deg):
    li, ri = int(np.argmin(np.abs(REF.lin - lin))), int(np.argmin(np.abs(RDEG - deg)))
    assert abs(REF.lin[li] - lin) < 1e-3 and abs(RDEG[ri] - deg) < 0.51, (lin, deg)
    return li, ri


def k_checks():
    rows = []
    for f in ('g38_planonly.log', 'g38_planonly_s19k3.log'):
        pend = None
        for ln in open(os.path.join(R, 'p1_g38', f)):
            m = TRAV.search(ln)
            if m:
                pend = m.groups()
                continue
            m = TENV.search(ln)
            if m and pend and m.group(1) == pend[0]:
                g = int(pend[1])
                a, b = idx_of(float(pend[2]), float(pend[3])), idx_of(float(pend[4]), float(pend[5]))
                rows.append(dict(f=f, g=g, a=a, b=b, v=m.group(3), d=float(m.group(4)) / 1000, geom=m.group(5)))
                pend = None
    print(f'K1: {len(rows)} baris traverse-lingkungan di log G38')
    bad = 0
    with Pool(NPROC) as pool:
        # the screen ran frm -> to; compute exactly that direction (not the canonical key) for K1
        res = pool.map(_edge_job, [(r['g'], *r['a'], *r['b']) for r in rows])
    for r, d in zip(rows, res):
        own = r['geom'].startswith(gp(r['g']))
        same_v = d['verdict'] == r['v']
        dd = abs(d['d'] - r['d'])
        ok = same_v and (dd <= 1e-3 if own else True)
        bad += not ok
        print(f"  g{r['g']} {r['a']}->{r['b']}: log {r['v']} {r['d'] * 1000:.1f} ({r['geom']})  G40 {d['verdict']} "
              f"{d['d'] * 1000:.1f} ({d['geom']}) {'ok' if ok else 'BEDA'}{'' if own else ' [geom log bukan t{g}_]'}")
    # K2: torque fields bit-identical to the G38 cache
    F38 = G.Feas()
    keys = [k for k, d in F38.mine.items() if d['ok3']][:3] + [k for k, d in F38.mine.items() if not d['ok3']][:1]
    with Pool(4) as pool:
        res = pool.map(_tuple_job_full, keys)
    for k, d in zip(keys, res):
        same = d['taumax'] == F38.mine[k]['taumax'] and d['ok3'] == F38.mine[k]['ok3']
        bad += not same
        print(f'K2 {k}: ok3 {d["ok3"]} taumax identik {same}; g38_env {F38.mine[k]["env"]} -> ulang {d["g38_env"]}, '
              f'env2 {d["env2"]} (n_env {d["n_env"]} n_box {d["n_box"]} n_both {d["n_both"]}/{d["n_sol"]})')
    print('K: LULUS' if not bad else f'K: GAGAL ({bad})')
    return not bad


TASK = re.compile(r'\[(\d+)\] task t(\d) (arm_\d) -> \[.*\] @\(([\d.]+), ([+-]\d+)\): (\S+)')


def g38_tuples():
    """(seed, task) -> tuple key used by the G38 R35 schedule."""
    out = {}
    for r in json.load(open(os.path.join(R, 'p1_g38/g38_candidates.json'))):
        if not r.get('R35', {}).get('ok_i'):
            continue
        for i, n in enumerate(r['nodes']):
            v = r['R35']['served'][str(n)]
            li, ri = idx_of(v['rail'], v['rot_deg'])
            out[(r['seed'], i)] = (int(n), v['gantry'], li, ri, sched.GANTRY_ARMS[v['gantry']].index(v['arm']))
    return out


def explain():
    pose_ok = load_pose_ok()
    F = Feas2(pose_ok)
    tup = g38_tuples()
    keys = sorted(set(tup.values()))
    tab = dict(zip(keys, F.table_ok(keys)))
    with Pool(NPROC) as pool:
        F.compute(keys, pool, full=True)
    # verdicts of tasks in the G38 plan-only logs (last attempt line per task occurrence)
    seen = {}
    for f in ('g38_planonly.log', 'g38_planonly_s19k3.log'):
        seed = None
        for ln in open(os.path.join(R, 'p1_g38', f)):
            m = re.search(r'seed (\d+) R35 sampel', ln)
            if m:
                seed = int(m.group(1))
            m = TASK.search(ln)
            if m:
                seen.setdefault((seed, int(m.group(2))), []).append(m.group(6))
    print(f'tuple terpakai G38 R35: {len(keys)} unik (dari {len(tup)} tugas)')
    cnt = dict(tab=0, pose=0, ok3=0, env2=0, g38env=0, all=0)
    for k in keys:
        d = F.mine[k]
        cnt['tab'] += bool(tab[k])
        cnt['pose'] += bool(pose_ok[k[1]][k[2], k[3]])
        cnt['ok3'] += d['ok3']
        cnt['g38env'] += d['ok3'] and d['g38_env']
        cnt['env2'] += d['ok3'] and d['env2']
        cnt['all'] += bool(tab[k] and pose_ok[k[1]][k[2], k[3]] and d['ok3'] and d['env2'])
    print(f"  lolos: tabel {cnt['tab']}, pose(a) {cnt['pose']}, torsi tepat {cnt['ok3']}, env G38 (ulang) "
          f"{cnt['g38env']}, env2 {cnt['env2']}, semua (c) {cnt['all']} -> (c) menolak {len(keys) - cnt['all']}")
    print('konfusi vs verdict tugas log G38 (seed, tugas): verdict -> (c)')
    for (seed, t), vs in sorted(seen.items()):
        k = tup[(seed, t)]
        d = F.mine[k]
        c = bool(tab[k] and pose_ok[k[1]][k[2], k[3]] and d['ok3'] and d['env2'])
        print(f'  s{seed:2d} t{t} {k}: {"/".join(vs)} -> (c) {"terima" if c else "TOLAK"} (tabel {bool(tab[k])}, '
              f"pose {bool(pose_ok[k[1]][k[2], k[3]])}, n_env {d['n_env']} n_box {d['n_box']} n_both {d['n_both']}"
              f"/{d['n_sol']}, env {d['env_best_m']} box {d['box_best_m']})")
    json.dump(dict(keys=[list(k) for k in keys], counts=cnt,
                   confusion={f'{s}:{t}': dict(log=vs, key=list(tup[(s, t)])) for (s, t), vs in seen.items()}),
              open(os.path.join(HERE, 'g40_explain.json'), 'w'), indent=1)


if __name__ == '__main__':
    {'k': lambda: sys.exit(0 if k_checks() else 1), 'poses': poses, 'explain': explain, 'run': run,
     'run_exact': lambda: run('exact')}[sys.argv[1]]()
