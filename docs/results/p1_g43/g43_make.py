"""G43 (docs/p1_g43_rot180.md): the G42f scheduler (g42_make: solve_exact_f + FIX_CHOICES + (s42) + (c41b) box 10 mm)
with the rotation set widened from R35 to R180 = every grid rotation with |rot| < 180 deg. OFFLINE, no ROS.

Nothing in the predicates changes: g42_make / g41_make / g40_make functions are called as they are; only the module
global g40_make.RSET (read at call time by build() and neighbour_edges()) is swapped, and the caches are COPIES of the
G42 caches with new rows appended (the G42 files are never written).

    python3 g43_make.py k_r35   -> R35 through this wrapper on the 20 seeds == g42f_candidates.json (bit-identical)
    python3 g43_make.py run     -> g43_candidates.json (R180)
    python3 g43_make.py iv | chain -> (iv') / (iv'') of the G43 candidates
"""
import json
import os

import numpy as np
import shutil
import sys
import time
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g42'))
import g42_make as K  # noqa: E402

G41, G4, S, MI, sched = K.G41, K.G4, K.S, K.MI, K.sched
RDEG = S.RDEG
RSETS = {'R35': list(S.RSETS['R35']),
         'R180': [i for i, r in enumerate(RDEG) if abs(r) < 180.0 - 1e-6]}      # A1: -180 lies ON the joint limit
NPROC = int(os.environ.get('G43_NPROC', 15))                                  # A2; 8 + nice 19 while a mock runs
G42F = os.path.join(R, 'p1_g42/g42f_candidates.json')
MIRROR = os.environ.get('G43_MIRROR', '1') == '1'                               # B0; K-R35 ran without it
G46 = os.environ.get('G46', '0') == '1'                                        # E1+E2+E3 (docs/p1_g46_sched_speed.md)


def setup(rset, tag):
    """Point every cache at a G43 copy and select the rotation set. Returns the candidates path."""
    K.FIX_CHOICES = True
    K.VAR = 'G43'
    G4.RSET = RSETS[rset]
    for name, src in (('tuple', K.TUPLE_CACHE), ('edge', K.EDGE_CACHE), ('stop', K.STOP_CACHE)):
        dst = os.path.join(HERE, f'g43_{name}_cache.jsonl')
        if not os.path.exists(dst):
            shutil.copy(src, dst)
    K.TUPLE_CACHE = G41.TUPLE_CACHE = os.path.join(HERE, 'g43_tuple_cache.jsonl')
    K.EDGE_CACHE = G4.EDGE_CACHE = os.path.join(HERE, 'g43_edge_cache.jsonl')
    K.STOP_CACHE = os.path.join(HERE, 'g43_stop_cache.jsonl')
    return os.path.join(HERE, f'{tag}_candidates.json')


# ------------------------------------------------------------------------- B0: 180-deg mirror (memory only)
def mirror_tuple(key):
    n, g, li, ri, s = key
    return (n, g, li, ri - 36 if RDEG[ri] >= 0 else ri + 36, 1 - s)


def mirror_edge(key):
    g, l0, r0, l1, r1 = key
    d = -36 if (r0 >= 36 and r1 >= 36) else (36 if (r0 < 36 and r1 < 36) else None)
    return None if d is None else G4.edge_key(g, (l0, r0 + d), (l1, r1 + d))


def _mirror_into(c, keys, mfn):
    for k in keys:
        m = mfn(k)
        if m is not None and m not in c:
            d = dict(c[k])
            d['key'], d['mirror_of'] = list(m), list(k)
            c[m] = d


class Feas43(G41.Feas41):
    """Feas41 + every COMPUTED tuple also known at its 180-deg mirror (same verdict, same q6: K-SYM 40/40)."""

    def __init__(self, pose_ok):
        super().__init__(pose_ok)
        _mirror_into(self.mine, list(self.mine), mirror_tuple)

    def compute(self, keys, pool, full=False):
        before = set(self.mine)
        super().compute(keys, pool, full)
        _mirror_into(self.mine, [k for k in self.mine if k not in before], mirror_tuple)


class Edges43(G4.Edges):
    """Edges + every computed edge with both rotations on one side also known shifted by 180 deg (K-SYM-E 20/20)."""

    def __init__(self):
        super().__init__()
        _mirror_into(self.c, list(self.c), mirror_edge)

    def compute(self, keys, pool):
        before = set(self.c)
        super().compute(keys, pool)
        _mirror_into(self.c, [k for k in self.c if k not in before], mirror_edge)


# ------------------------------------------------------------------- G46 E1: edge verdicts from the fine pose table
FINE = os.path.join(R, 'p1_g46/g46_fine_pose.npz')


def _sparse_min(D):
    """st[a][b] = min of D over [i, i + 2^a) x [j, j + 2^b)."""
    st = [[D]]
    for b in range(1, int(np.log2(D.shape[1])) + 1):
        p = st[0][b - 1]
        st[0].append(np.minimum(p[:, :-(1 << (b - 1))], p[:, (1 << (b - 1)):]))
    for a in range(1, int(np.log2(D.shape[0])) + 1):
        row = []
        for b in range(len(st[0])):
            p = st[a - 1][b]
            row.append(np.minimum(p[:-(1 << (a - 1))], p[(1 << (a - 1)):]))
        st.append(row)
    return st


class EdgeMatrix:
    """G46 E1 (docs/p1_g46_sched_speed.md A1): (b) for EVERY pair of grid poses = min d over all fine lattice points
    (10 mm x 1 deg) of the closed lin x rot rectangle, CLEAR iff >= 50 mm. K-E1: 8196/8196 = the sweep verdicts."""

    def __init__(self):
        z = np.load(FINE)
        nl, nr = len(K.REF.lin), len(RDEG)
        fl = np.round(np.asarray(K.REF.lin) / 0.01).astype(int)          # coarse lin -> fine row
        fr = np.round(RDEG).astype(int) + 180                              # coarse rot -> fine col
        li, ri = np.divmod(np.arange(nl * nr), nr)
        self.ok, self.c = {}, {}
        for g in (1, 2):
            st = _sparse_min(z[f'd{g}'])
            A, B = np.meshgrid(np.arange(nl * nr), np.arange(nl * nr), indexing='ij')
            i0, i1 = np.minimum(fl[li[A]], fl[li[B]]), np.maximum(fl[li[A]], fl[li[B]])
            j0, j1 = np.minimum(fr[ri[A]], fr[ri[B]]), np.maximum(fr[ri[A]], fr[ri[B]])
            ka = np.floor(np.log2(i1 - i0 + 1)).astype(int)
            kb = np.floor(np.log2(j1 - j0 + 1)).astype(int)
            out = np.full(A.shape, np.inf)
            for a in np.unique(ka):
                for b in np.unique(kb):
                    m = (ka == a) & (kb == b)
                    if not m.any():
                        continue
                    T = st[a][b]
                    x0, x1, y0, y1 = i0[m], i1[m] - (1 << a) + 1, j0[m], j1[m] - (1 << b) + 1
                    out[m] = np.minimum(np.minimum(T[x0, y0], T[x1, y0]), np.minimum(T[x0, y1], T[x1, y1]))
            self.ok[g] = out >= 0.05
            self.d = getattr(self, 'd', {})
            self.d[g] = out

    def get(self, key):
        g, l0, r0, l1, r1 = key
        nr = len(RDEG)
        return bool(self.ok[g][l0 * nr + r0, l1 * nr + r1])

    def compute(self, keys, pool):
        pass                                     # every edge is known


def solve_gantry_e1(inst, g, E, forb):
    """g42_make.solve_gantry_f VERBATIM except the python pair loop over E.get is one EdgeMatrix index."""
    n = inst.n
    dur_full = K._dur_table_f(inst, g, forb)
    usable = np.isfinite(dur_full[1:]).any(axis=0)
    keep = np.flatnonzero(usable)
    p0 = inst.p0[g]
    if p0 not in keep:
        keep = np.sort(np.append(keep, p0))
    r0 = int(np.searchsorted(keep, p0))
    T = sched.traverse_matrix(inst.poses[g][keep], inst.t_fold, inst.rail_cmd, inst.rot_cmd)
    T0 = None if inst.t_fold_first is None else sched.traverse_matrix(
        inst.poses[g][keep], inst.t_fold_first, inst.rail_cmd, inst.rot_cmd)[r0]
    gid = np.asarray(inst.meta['cols'])[keep]
    T[~E.ok[g][np.ix_(gid, gid)]] = np.inf
    if T0 is not None:
        T0 = np.where(np.isinf(T[r0]), np.inf, T0)
    if T0 is None:
        T0 = T[r0].copy()
    np.fill_diagonal(T, np.inf)
    dur = dur_full[:, keep]
    Kp = len(keep)
    h = np.full((1 << n, Kp), np.inf)
    w = np.full((1 << n, Kp), np.inf)
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


# ---------------------------------------------------------------------- G46 E2: certified-epsilon lazy loop
EPS = float(os.environ.get('G46_EPS', 0.03))


def solve_lazy46(nodes, F, E, SX, pool, log):
    """g42_make.solve_lazy42 + UB: unknown tuples INFEASIBLE (verified facts only), its >= 2-task stops verified by
    (s42) before it counts. Stop when UB <= (1 + EPS) * LB; the UB schedule is fully verified."""
    from dataclasses import replace
    it = 0
    while True:
        it += 1
        inst, unver = G4.build(nodes, F)
        forb = SX.forbids(inst)
        sol = K.solve_exact_f(inst, E, forb)                         # LB (ValueError = (i) fails)
        lb = sol.makespan
        ut = S.used_unverified(inst, sol, unver)
        us = [] if ut else [x for x in K.stop_items(inst, sol, F) if x[0] not in SX.c]
        if not ut and not us:
            log(f'    iterasi {it}: LB = UB {lb:.3f} (relaksasi terverifikasi)')
            return inst, sol, it, dict(lb=lb, ub=lb, gap=0.0)
        iu = replace(inst, reach={g: inst.reach[g] & ~unver[g] for g in inst.reach})
        ub, usol = np.inf, None
        while True:
            try:
                usol = K.solve_exact_f(iu, E, SX.forbids(iu))
            except ValueError:
                usol = None
                break
            new = [x for x in K.stop_items(iu, usol, F) if x[0] not in SX.c]
            if not new:
                break
            SX.compute(new, pool)
        if usol is not None:
            ub = usol.makespan
        log(f'    iterasi {it}: LB {lb:.3f}, UB {ub:.3f}, gap {100 * (ub / lb - 1):.2f} %, tuple belum terverifikasi '
            f'{len(ut)}, stop {len(us)}')
        if usol is not None and ub <= (1 + EPS) * lb:
            return iu, usol, it, dict(lb=lb, ub=ub, gap=ub / lb - 1)
        F.compute(S.neighbours(inst, unver, ut) if ut else [], pool)
        if us:
            SX.compute(us, pool)


def rot_stats(sj, p0):
    """max |rot| over stops and max |drot| per traverse (from p0 rot 0, incl. the final return is NOT part of a
    schedule: run_g32 ends where the last stop is, --final-retract only retracts arms)."""
    mx, dmx = 0.0, 0.0
    for g in ('1', '2'):
        cur = 0.0
        for st in sj[g]:
            mx = max(mx, abs(st['rot_deg']))
            dmx = max(dmx, abs(st['rot_deg'] - cur))
            cur = st['rot_deg']
    return mx, dmx


def run(rset='R180', tag='g43', seeds=None):
    out = setup(rset, tag)
    F, E, SX = (Feas43(G41.pose_ok_b()), Edges43(), K.Stops()) if MIRROR else \
        (G41.Feas41(G41.pose_ok_b()), G4.Edges(), K.Stops())
    if G46:                                          # docs/p1_g46_sched_speed.md: E1 + E2 + E3
        E = EdgeMatrix()
        K.solve_gantry_f = solve_gantry_e1
        G41.STOP_TORQ = True
    g42f = {r['seed']: r for r in json.load(open(G42F))}
    rows = json.load(open(out)) if os.path.exists(out) else []           # resumable: finished seeds are kept
    done = {r['seed'] for r in rows}
    log = lambda s: print(s, flush=True)                                 # noqa: E731
    with Pool(NPROC) as pool:
        first = [0, 3, 7, 8, 10, 18]               # the six G42f seeds first: Gerbang 1 + HW candidates (order only)
        for seed in (first + [x for x in sorted(g42f) if x not in first]) if seeds is None else seeds:
            if seed in done:
                continue
            rb = g42f[seed]
            rec = {k: rb[k] for k in ('seed', 'drawn', 'nodes', 'kind', 'xyz', 'd_env', 'p0', 'p0_rot_deg', 'rot_cmd')}
            rec['g42f_makespan'] = rb['G42'].get('makespan') if rb['G42'].get('ok_i') else None
            log(f'seed {seed} {rset}  [{time.strftime("%T")}]')
            t0 = time.time()
            try:
                if G46:
                    inst, sol, it, bnd = solve_lazy46(rb['nodes'], F, E, SX, pool, log)
                else:
                    (inst, sol, it), bnd = K.solve_lazy42(rb['nodes'], F, E, SX, pool, log), None
            except ValueError as e:
                rec[rset] = dict(ok_i=False, why=str(e), wall_s=time.time() - t0)
                log(f'  seed {seed}: (i) GAGAL -- {e}  [{time.strftime("%T")}]')
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
            sjn = json.loads(json.dumps(sj, default=float))
            mx, dmx = rot_stats(sjn, rec['p0'])
            rec[rset] = dict(ok_i=True, ok_ii=ok_ii, ok_iii=conf is None, makespan=sol.makespan,
                             serial=sum(sol.finish.values()), iters=it, conflict=repr(conf), wall_s=time.time() - t0,
                             moves={str(g): v['moves'] for g, v in dec.items()},
                             ntask={str(g): v['ntask'] for g, v in dec.items()},
                             stops_s42=[dict(key=list(k), ok=SX.c[k]['ok']) for k, _ in K.stop_items(inst, sol, F)],
                             rot_max_deg=mx, drot_max_deg=dmx, bounds=bnd, schedule=sj)
            same = bool(rb['G42'].get('ok_i')) and all(
                [(s['rail_m'], s['rot_deg'], s['tasks'], s['goal']) for s in sjn[str(g)]] ==
                [(s['rail_m'], s['rot_deg'], s['tasks'], s['goal']) for s in rb['G42']['schedule'][str(g)]]
                for g in (1, 2))
            rec[rset]['same_as_g42f'] = bool(same)
            log(f"  seed {seed}: makespan {sol.makespan:.2f} (G42f {rec['g42f_makespan']}), LB/UB {bnd}, sama G42f {same}, "
                f"|rot| maks {mx:.0f}, drot maks {dmx:.0f}, konflik {conf!r}, (ii) {ok_ii}, iterasi {it}, "
                f"{time.time() - t0:.0f} s  [{time.strftime('%T')}]")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r[rset].get('ok_i') and r[rset]['ok_ii'] and r[rset]['ok_iii']]
    print(f"\nlolos (i)-(iii) {rset}: {len(ok)}/{len(rows)} seed {ok}; tuple {len(F.mine)}, tepi {len(getattr(E, 'c', {}))}, "
          f"stop (s42) {len(SX.c)} (tolak {sum(not d['ok'] for d in SX.c.values())})", flush=True)


def k_r35():
    """R35 through this wrapper must reproduce G42f exactly (stops, assignment, goals, makespan) on all 20 seeds."""
    run('R35', 'g43_kr35')
    rows = json.load(open(os.path.join(HERE, 'g43_kr35_candidates.json')))
    g42f = {r['seed']: r['G42'] for r in json.load(open(G42F))}
    bad = []
    for r in rows:
        a, b = r['R35'], g42f[r['seed']]
        if a.get('ok_i') != b.get('ok_i') or (a.get('ok_i') and not a['same_as_g42f']) or \
                (not a.get('ok_i') and a['why'] != b['why']):
            bad.append(r['seed'])
    print(f'K-R35: {20 - len(bad)}/20 identik G42f, beda {bad}')


if __name__ == '__main__':
    {'k_r35': k_r35, 'run': run}[sys.argv[1]]()
