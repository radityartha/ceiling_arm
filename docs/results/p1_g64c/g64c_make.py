"""G64-C (docs/p1_g64c.md A0.6-A0.10): the P1'' scheduler with a per-gantry mode REST | T1. OFFLINE, no ROS.

Mode REST = the G55 chain as it is (Feas49, G46 EdgeMatrix, (s42) Stops, solve_lazy46; caches = COPIES here).
Mode T1 of gantry g = both arms of g at T1 wherever REST appears (A0.4): fold at p0, T1 traverse, task lines T1 <-> q,
retract to T1, return to p0, unfold at p0. Objects (new files, g46/g40/g55 never written):
  (a) g64c_fine_T1.npz     = g46 fine lattice with both arms of g at T1          -> EdgeMatrixT
  (b) g64c_pose_T1.json    = both arms T1 at (lin, rot): env >= 50 mm AND box >= 10 mm
  (c) _tuple_job49T        = _tuple_job49 with branch order max|q - T1|, partner T1, line T1 -> q (61 pts, torque / env /
                             box EVERY point / same-gantry); cache g64c_tuple49T_cache.jsonl; no 180-deg mirror
  (d) _stop_jobT           = g42 _stop_job starting from both arms T1, final retract to T1; cache g64c_stopT_cache.jsonl
Solver: 4 mode combinations; F / E / SX split per gantry to the objects of its mode; solve_lazy46 unchanged. A T1 gantry
with >= 1 task pays C_FOLD + C_UNFOLD (on T0) and the return to p0 (h[0][r] = T[r, r0]). T1 only where the fold at p0 is
certified (g64c_cert.json). G64C_ENV=1 masks rotations outside the HW envelope in BOTH modes (default 0 = system now).

    python3 g64c_make.py fine | poses          -> (a) / (b)
    python3 g64c_make.py krest <base> S..      -> REST/REST through this wrapper, base g55R90 | g55R35 | g52R90
    python3 g64c_make.py combos <base> S..     -> 4-combination solve of seeds S..
"""
import itertools
import json
import math
import os
import shutil
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g55'))
import g55_make as G55  # noqa: E402

M, K, G41, G4, W, S = G55.M, G55.K, G55.G41, G55.G4, G55.W, G55.S
sched, REF, NR, RDEG = K.sched, G4.REF, G4.NR, M.RDEG
O2, PREFIX, SLOT_ARM, MARGIN = G4.O2, G4.PREFIX, G4.SLOT_ARM, G4.MARGIN
REST, LIM, OFFSET = G41.REST, G41.LIM, G41.OFFSET
T1 = np.array(json.load(open(os.path.join(R, 'p1_g64/g64_transit_T.json')))['q6'])
ARMS = {g: [SLOT_ARM[(g, 0)], SLOT_ARM[(g, 1)]] for g in (1, 2)}
FINE_T = os.path.join(HERE, 'g64c_fine_T1.npz')
POSE_T = os.path.join(HERE, 'g64c_pose_T1.json')
TUPLE_T = os.path.join(HERE, 'g64c_tuple49T_cache.jsonl')
STOP_T = os.path.join(HERE, 'g64c_stopT_cache.jsonl')
CERT = os.path.join(HERE, 'g64c_cert.json')
ENVELOPE = json.load(open(os.path.join(R, 'p1_g48/rot_envelope.json')))
C_FOLD_ENV = os.environ.get('G64C_C_FOLD')                     # measured in mock (A0.9); unset = PROVISIONAL a_r
C_FOLD = C_UNFOLD = float(C_FOLD_ENV) if C_FOLD_ENV else float(G55.P2['a_r'])
PROVISIONAL = C_FOLD_ENV is None
ENV = os.environ.get('G64C_ENV', '0') == '1'
NPROC = int(os.environ.get('G64C_NPROC', 15))
COMBOS = [('REST', 'REST'), ('REST', 'T1'), ('T1', 'REST'), ('T1', 'T1')]   # tie -> fewer T1 (stable min)


def held_T(g, lin, rot):
    """base_joints (other gantry as the predicates hold it) with both arms of g at T1."""
    j = G4.G.base_joints(g, lin, rot)
    for a in ARMS[g]:
        j.update({f'{PREFIX[a]}joint_{i}': float(v) for i, v in enumerate(T1, 1)})
    return j


def cos_line(a, b, steps=60):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return [list(a + (b - a) * 0.5 * (1.0 - math.cos(math.pi * k / steps))) for k in range(steps + 1)]


# --------------------------------------------------------------------------------------------- (a) fine lattice T1
def _fine_row(arg):
    import g46_make as G46
    g, li = arg
    ec, _ = G4.checkers()
    out = np.empty(len(G46.ROT))
    for k, r in enumerate(G46.ROT):
        out[k] = ec.check(ec.q_from(held_T(g, float(G46.LIN[li]), math.radians(r))), only=(G4.gp(g),))[0]
    return g, li, out


def fine():
    sys.path.insert(0, os.path.join(R, 'p1_g46'))
    import g46_make as G46
    t0 = time.time()
    D = {g: np.full((len(G46.LIN), len(G46.ROT)), np.nan) for g in (1, 2)}
    with Pool(NPROC) as p:
        for g, li, row in p.imap_unordered(_fine_row, [(g, li) for g in (1, 2) for li in range(len(G46.LIN))]):
            D[g][li] = row
    np.savez(FINE_T, d1=D[1], d2=D[2], lin=G46.LIN, rot=G46.ROT)
    print(f'(a) T1: {2 * D[1].size} pose dalam {time.time() - t0:.0f} s; d min g1 {np.nanmin(D[1]):.4f} '
          f'g2 {np.nanmin(D[2]):.4f}', flush=True)


class EdgeMatrixT(M.EdgeMatrix):
    def __init__(self):
        old, M.FINE = M.FINE, FINE_T
        try:
            super().__init__()
        finally:
            M.FINE = old


# ------------------------------------------------------------------------------------------------- (b) pose table T1
def _pose_job(key):
    g, li, ri = key
    ec, bx = G4.checkers()
    q = ec.q_from(held_T(g, float(REF.lin[li]), float(REF.rot[ri])))
    d, wg = ec.check(q, only=(G4.gp(g),))
    b, wb = bx.dist(ec, q, G4.gp(g))
    return dict(key=[g, li, ri], d_env=round(float(d), 5), geom=wg, d_box=None if b == math.inf else round(float(b), 5),
                geom_box=wb, ok=bool(d >= G4.ENV_MIN and b > 0.0 and b >= G41.BOX_MIN))


def poses():
    keys = [(g, li, ri) for g in (1, 2) for li in range(len(REF.lin)) for ri in range(NR)]
    t0 = time.time()
    with Pool(NPROC) as p:
        rows = p.map(_pose_job, keys, chunksize=8)
    json.dump(rows, open(POSE_T, 'w'))
    okT, okR = G4.load_pose_ok(rows), G41.pose_ok_b()
    for g in (1, 2):
        rs = M.RSETS['R90']
        print(f'(b) g{g}: R90 layak T1 {int(okT[g][:, rs].sum())} vs REST {int(okR[g][:, rs].sum())} '
              f'(dari {okT[g][:, rs].size}); T1-saja {int((okT[g] & ~okR[g])[:, rs].sum())}, REST-saja '
              f'{int((okR[g] & ~okT[g])[:, rs].sum())}  [{time.time() - t0:.0f} s]', flush=True)


def load_pose_T():
    return G4.load_pose_ok(json.load(open(POSE_T)))


def envelope(pose_ok):
    """A0.10: a copy with every rotation outside the HW envelope infeasible."""
    out = {}
    for g in (1, 2):
        lo, hi = ENVELOPE[str(g)]
        out[g] = pose_ok[g] & ((RDEG >= lo - 1e-6) & (RDEG <= hi + 1e-6))[None, :]
    return out


# ------------------------------------------------------------------------------------------------- (c) tuple T1
def screen_line_T(g, arm, lin, rot, q6, ec, bx):
    """G41.screen_line with the line T1 -> q6 (61 pts incl. both ends), partner T1; order torque -> env -> box -> same."""
    base = held_T(g, lin, rot)
    names = [f'{PREFIX[arm]}joint_{i}' for i in range(1, 7)]
    pts = cos_line(T1, q6)
    tau = max(float(np.max(O2.gravity(arm, np.array(q), lin, rot) + OFFSET + MARGIN - LIM)) for q in pts)
    out = dict(tau_excess=round(tau, 5))
    if tau > 0.0:
        return dict(out, ok=False, why='torsi-garis')
    placed = dict(base)
    placed.update(zip(names, pts[0]))
    v, d, wg, _ = ec.screen_trajectory(names, pts, placed)
    out.update(env=v, env_d=round(float(d), 5), env_geom=wg)
    if v != 'CLEAR':
        return dict(out, ok=False, why='garis-env')
    bmin = math.inf
    for q in pts:
        j = dict(base)
        j.update(zip(names, q))
        b, _ = bx.dist(ec, ec.q_from(j), G4.gp(g))
        bmin = min(bmin, b)
        if b <= 0.0 or b < G41.BOX_MIN:
            break
    out['box_line_d'] = round(float(bmin), 5)
    if bmin <= 0.0 or bmin < G41.BOX_MIN:
        return dict(out, ok=False, why='garis-kotak')
    held = {n: x for n, x in placed.items() if n not in names}
    v2, d2, pair, _ = G41.same_checker(g).screen_trajectory(names, pts, held)
    out.update(same=v2, same_d=round(float(d2), 5), same_pair=list(pair) if not isinstance(pair, str) else pair)
    if v2 != 'CLEAR':
        return dict(out, ok=False, why='garis-se-gantry')
    return dict(out, ok=True, why='')


def _tuple_job49T(key):
    t0 = time.process_time()
    n, g, li, ri, s = key
    arm, lin, rot = SLOT_ARM[(g, s)], float(REF.lin[li]), float(REF.rot[ri])
    r = O2.solve(G4.X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ok3 = bool(O2.torq2(r, MARGIN))
    goal, n_e1, n_chk, n_tq, whys = None, 0, 0, 0, {}
    ec, bx = G4.checkers()
    base, p = held_T(g, lin, rot), PREFIX[arm]
    sols, taus = r['solutions'], r['solutions_tau']
    order = sorted(range(len(sols)), key=lambda i: (float(np.max(np.abs(sols[i][1] - T1))), i))
    for i in order:
        roll, q6 = sols[i]
        n_chk += 1
        if not W._branch_ok(taus[i]):
            whys['torsi-cabang'] = whys.get('torsi-cabang', 0) + 1
            continue
        n_tq += 1
        j = dict(base)
        j.update({f'{p}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
        q = ec.q_from(j)
        d, _ = ec.check(q, only=(G4.gp(g),))
        if d < G4.ENV_MIN:
            whys['e1-env'] = whys.get('e1-env', 0) + 1
            continue
        b, _ = bx.dist(ec, q, G4.gp(g))
        if b <= 0.0 or b < G41.BOX_MIN:
            whys['e1-kotak'] = whys.get('e1-kotak', 0) + 1
            continue
        n_e1 += 1
        sl = screen_line_T(g, arm, lin, rot, q6, ec, bx)
        if not sl['ok']:
            whys[sl['why']] = whys.get(sl['why'], 0) + 1
            continue
        goal = dict(roll_k=int(roll), q6=[float(v) for v in q6], dist_T1_deg=round(math.degrees(
            float(np.max(np.abs(q6 - T1)))), 2), line=sl, tau_branch=[float(v) for v in taus[i]])
        break
    return dict(key=[int(v) for v in key], mode='T1', n_sol=r['n_sol'], saturated=bool(r['saturated']),
                taumax=r['taumax'].tolist(), ok3=ok3, n_checked=n_chk, n_torque_ok=n_tq, n_e1=n_e1, whys=whys,
                ok41=goal is not None, ok49=goal is not None, goal=goal, cpu_s=round(time.process_time() - t0, 2))


class FeasT:
    """Mode-T1 tuple facts. get: False if (b) T1 says no; cached verdict; None = unknown (relaxation)."""

    def __init__(self, pose_ok):
        self.pose_ok, self.mine = pose_ok, {}
        if os.path.exists(TUPLE_T):
            for ln in open(TUPLE_T):
                d = json.loads(ln)
                self.mine[tuple(d['key'])] = d

    def get(self, key, tab):
        if not self.pose_ok[key[1]][key[2], key[3]]:
            return False
        d = self.mine.get(key)
        return None if d is None else bool(d['ok41'])

    def compute(self, keys, pool, full=False):
        keys = sorted({k for k in keys if k not in self.mine})
        if not keys:
            return
        with open(TUPLE_T, 'a') as f:
            for d in pool.imap_unordered(_tuple_job49T, keys, chunksize=1):
                self.mine[tuple(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()


# ------------------------------------------------------------------------------------------------- (d) stops T1
def _stop_jobT(arg):
    """K._stop_job with both arms starting at T1 and the final both-arm retract to T1."""
    t0 = time.process_time()
    key, goals = arg
    g, li, ri, seq0, seq1 = key
    ec, bx = G4.checkers()
    st = held_T(g, float(REF.lin[li]), float(REF.rot[ri]))
    seqs = (list(seq0), list(seq1))
    lines, ok = [], True
    f = [0.5 * (1.0 - math.cos(math.pi * i / 60)) for i in range(0, 61)]
    for j in range(max(len(s) for s in seqs)):
        for s in (0, 1):
            if j >= len(seqs[s]) or not ok:
                continue
            n = seqs[s][j]
            names = [f'{PREFIX[SLOT_ARM[(g, s)]]}joint_{i}' for i in range(1, 7)]
            s0 = np.array([st[x] for x in names])
            q1 = np.array(goals[(n, s)])
            frm = 'T1' if np.max(np.abs(s0 - T1)) < math.radians(0.5) else 'ujung tugas sebelumnya'
            ok_, d = K._screen(g, names, [list(s0 + (q1 - s0) * x) for x in f], st, ec, bx)
            lines.append(dict(d, what=f'node {n} slot {s} dari {frm}'))
            ok &= ok_
            st.update(zip(names, map(float, q1)))
    if ok:
        names = [f'{PREFIX[SLOT_ARM[(g, s)]]}joint_{i}' for s in (0, 1) for i in range(1, 7)]
        cur = [st[x] for x in names]
        if max(abs(c - x) for c, x in zip(cur, list(T1) * 2)) >= math.radians(0.5):
            ok_, d = K._screen(g, names, cos_line(cur, list(T1) * 2)[1:], st, ec, bx)
            lines.append(dict(d, what='retract kedua lengan ke T1'))
            ok &= ok_
    return dict(key=[g, li, ri, list(seq0), list(seq1)], mode='T1', ok=bool(ok), lines=lines,
                cpu_s=round(time.process_time() - t0, 2))


class StopsT(K.Stops):
    def __init__(self):
        self.c = {}
        if os.path.exists(STOP_T):
            for ln in open(STOP_T):
                d = json.loads(ln)
                self.c[K.ckey(d['key'])] = d

    def compute(self, items, pool):
        items = [(k, gl) for k, gl in items if k not in self.c]
        if not items:
            return
        with open(STOP_T, 'a') as f:
            for d in pool.imap_unordered(_stop_jobT, items, chunksize=1):
                self.c[K.ckey(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()


# ---------------------------------------------------------------------------------------- per-gantry composites
class _ByG:
    """Read-only mapping over the per-gantry dicts; the gantry is key[gi]."""

    def __init__(self, by, gi):
        self.by, self.gi = by, gi

    def __contains__(self, k):
        return k in self.by[k[self.gi]]

    def __getitem__(self, k):
        return self.by[k[self.gi]][k]

    def get(self, k, default=None):
        return self.by[k[self.gi]].get(k, default)


class CompF:
    def __init__(self, Fg, F_rest):
        self.Fg, self.F_rest = Fg, F_rest
        self.pose_ok = {g: Fg[g].pose_ok[g] for g in (1, 2)}
        self.mine = _ByG({g: Fg[g].mine for g in (1, 2)}, 1)

    def table_ok(self, keys):
        return self.F_rest.table_ok(keys)

    def get(self, key, tab):
        return self.Fg[key[1]].get(key, tab)

    def compute(self, keys, pool, full=False):
        for g in (1, 2):
            sub = [k for k in keys if k[1] == g]
            if sub:
                self.Fg[g].compute(sub, pool)


class CompE:
    def __init__(self, Eg):
        self.Eg = Eg
        self.ok = {g: Eg[g].ok[g] for g in (1, 2)}
        self.d = {g: Eg[g].d[g] for g in (1, 2)}

    def get(self, key):
        return self.Eg[key[0]].get(key)

    def compute(self, keys, pool):
        pass


class CompSX:
    def __init__(self, Sg):
        self.Sg = Sg
        self.c = _ByG({g: Sg[g].c for g in (1, 2)}, 0)

    def forbids(self, inst):
        return {g: self.Sg[g].forbids(inst)[g] for g in (1, 2)}

    def compute(self, items, pool):
        for g in (1, 2):
            sub = [x for x in items if x[0][0] == g]
            if sub:
                self.Sg[g].compute(sub, pool)


# ------------------------------------------------------------------------------------------------------ gantry DP
MODES = {1: 'REST', 2: 'REST'}
_E1 = M.solve_gantry_e1


def solve_gantry_T(inst, g, E, forb):
    """M.solve_gantry_e1 VERBATIM except (A0.8): T0 += C_FOLD + C_UNFOLD and h[0][r] = T[r, r0] (return to p0)."""
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
    # --- G64-C ---
    T0 = T0 + (C_FOLD + C_UNFOLD)
    h0 = T[:, r0].copy()
    h0[r0] = 0.0
    # -------------
    dur = dur_full[:, keep]
    Kp = len(keep)
    h = np.full((1 << n, Kp), np.inf)
    w = np.full((1 << n, Kp), np.inf)
    h[0] = h0
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


def solve_gantry_mode(inst, g, E, forb):
    return (solve_gantry_T if MODES[g] == 'T1' else _E1)(inst, g, E, forb)


# ------------------------------------------------------------------------------------------------------- setup
def setup(base, cache_dir=HERE):
    """The REST chain of `base` with caches COPIED to cache_dir (T1 caches there too). Returns the rotation set."""
    global TUPLE_T, STOP_T
    TUPLE_T, STOP_T = (os.path.join(cache_dir, os.path.basename(p)) for p in (TUPLE_T, STOP_T))
    K.FIX_CHOICES = True
    K.VAR = 'G49'
    rset, src_dir, stop_src, tup_src = {
        'g55R90': ('R90', 'p1_g55', 'g55_stop49_cache.jsonl', 'g55_tuple49_cache.jsonl'),
        'g55R35': ('R35', 'p1_g55', 'g55_stop49_cache.jsonl', 'g55_tuple49_cache.jsonl'),
        'g52R90': ('R90', 'p1_g52', 'g52_stop49_cache.jsonl', 'g52_tuple49_cache.jsonl')}[base]
    if base.startswith('g55'):                       # = g55_make.run: P1'' cost
        S.RS.p1prime = G55.p1pp
        S.ROT_CMD = G55.P2['g'] * G55.ROT_CMD0
    G4.RSET = M.RSETS[rset]
    tag = 'g55' if base.startswith('g55') else 'g52'
    for src, name in ((stop_src, f'{tag}_stop'), (tup_src, f'{tag}_tuple')):
        dst = os.path.join(cache_dir, f'g64c_{name}_cache.jsonl')
        if not os.path.exists(dst):
            shutil.copy(os.path.join(R, src_dir, src), dst)
    K.STOP_CACHE = os.path.join(cache_dir, f'g64c_{tag}_stop_cache.jsonl')
    W.TUPLE_CACHE = G41.TUPLE_CACHE = os.path.join(cache_dir, f'g64c_{tag}_tuple_cache.jsonl')
    K.solve_gantry_f = solve_gantry_mode
    G41.STOP_TORQ = True
    return rset


def certified(g):
    """A0.3 / C1: T1 mode only at a p0 whose fold is certified (any other-gantry rail)."""
    rows = json.load(open(CERT))['rows']
    return any(r['ok'] and r['g'] == g and abs(r['rail'] - G4.P0_LIN[g]) < 1e-9 for r in rows)


def objects():
    """{'REST': (F, E, SX), 'T1': (F, E, SX)}; REST = the G55 objects exactly as g43_make.run builds them."""
    pr = G41.pose_ok_b()
    pt = load_pose_T()
    if ENV:
        pr, pt = envelope(pr), envelope(pt)
    return {'REST': (W.Feas49(pr), M.EdgeMatrix(), K.Stops()), 'T1': (FeasT(pt), EdgeMatrixT(), StopsT())}


class Pruned(Exception):
    pass


def solve_lazy46_cut(nodes, F, E, SX, pool, log, cutoff):
    """M.solve_lazy46 VERBATIM except: stop (Pruned) as soon as the certified LB >= cutoff -- the combination can then at
    best tie the incumbent, and a tie goes to the earlier combination (fewer T1). Not used for REST/REST."""
    from dataclasses import replace
    it = 0
    while True:
        it += 1
        inst, unver = G4.build(nodes, F)
        forb = SX.forbids(inst)
        sol = K.solve_exact_f(inst, E, forb)
        lb = sol.makespan
        if lb >= cutoff - 1e-9:
            raise Pruned(f'PANGKAS: LB {lb:.3f} >= terbaik {cutoff:.3f} (iterasi {it})')
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
        if usol is not None and ub <= (1 + M.EPS) * lb:
            return iu, usol, it, dict(lb=lb, ub=ub, gap=ub / lb - 1)
        F.compute(S.neighbours(inst, unver, ut) if ut else [], pool)
        if us:
            SX.compute(us, pool)


def solve_combos(nodes, O, pool, log, combos=COMBOS, prune=True):
    """Every combination; REST/REST with the plain REST objects (bit-identity). -> (results, best index | None)."""
    res = []
    for m in combos:
        modes = {1: m[0], 2: m[1]}
        bad = [g for g in (1, 2) if modes[g] == 'T1' and not certified(g)]
        if bad:
            res.append(dict(modes=modes, ok=False, why=f'T1 tidak tersertifikasi di p0 gantry {bad}'))
            log(f'  mode {m}: LEWATI -- {res[-1]["why"]}')
            continue
        MODES.update(modes)
        if m == ('REST', 'REST'):
            F, E, SX = O['REST']
        else:
            F = CompF({g: O[modes[g]][0] for g in (1, 2)}, O['REST'][0])
            E = CompE({g: O[modes[g]][1] for g in (1, 2)})
            SX = CompSX({g: O[modes[g]][2] for g in (1, 2)})
        log(f'  mode {m}  [{time.strftime("%T")}]')
        t0 = time.time()
        inc = min([r['makespan'] for r in res if r['ok']], default=None)
        try:
            if m == ('REST', 'REST') or not prune or inc is None:
                inst, sol, it, bnd = M.solve_lazy46(nodes, F, E, SX, pool, log)
            else:
                inst, sol, it, bnd = solve_lazy46_cut(nodes, F, E, SX, pool, log, inc)
        except Pruned as e:
            res.append(dict(modes=modes, ok=False, why=str(e), pruned=True, wall_s=time.time() - t0))
            log(f'  mode {m}: {e}')
            continue
        except ValueError as e:
            res.append(dict(modes=modes, ok=False, why=str(e), wall_s=time.time() - t0))
            log(f'  mode {m}: GAGAL -- {e}')
            continue
        res.append(dict(modes=modes, ok=True, inst=inst, sol=sol, iters=it, bounds=bnd, F=F, SX=SX,
                        makespan=sol.makespan, wall_s=time.time() - t0))
        log(f'  mode {m}: makespan {sol.makespan:.3f}, LB/UB {bnd}')
    MODES.update({1: 'REST', 2: 'REST'})
    ok = [i for i, r in enumerate(res) if r['ok']]
    best = min(ok, key=lambda i: (res[i]['makespan'], i)) if ok else None
    return res, best


def record(r, extra=None):
    """Candidates entry of one solved combination (= g43_make.run fields) + mode."""
    inst, sol, F, SX = r['inst'], r['sol'], r['F'], r['SX']
    dec = S.describe(inst, sol)
    conf = S.sched_coll.schedule_conflict(inst, sol.stops)
    sj = {str(g): v for g, v in M.MI.sched_json(inst, sol).items()}
    for g, gl in G41.goals(inst, sol, F).items():
        for st, gg in zip(sj[str(g)], gl):
            st['goal'] = gg
    sjn = json.loads(json.dumps(sj, default=float))
    mx, dmx = M.rot_stats(sjn, None)
    mode = {str(g): (r['modes'][g] if dec[g]['ntask'] else 'REST') for g in (1, 2)}
    return dict(ok_i=True, ok_ii=all(v['moves'] >= 1 for v in dec.values()), ok_iii=conf is None,
                makespan=sol.makespan, serial=sum(sol.finish.values()), iters=r['iters'], conflict=repr(conf),
                wall_s=r['wall_s'], moves={str(g): v['moves'] for g, v in dec.items()},
                ntask={str(g): v['ntask'] for g, v in dec.items()},
                stops_s42=[dict(key=list(k), ok=SX.c[k]['ok']) for k, _ in K.stop_items(inst, sol, F)],
                rot_max_deg=mx, drot_max_deg=dmx, bounds=r['bounds'], schedule=sj, mode=mode,
                c_fold=C_FOLD, c_unfold=C_UNFOLD, cost_provisional=PROVISIONAL, envelope=ENV, **(extra or {}))


def _key(v):
    return [(t['rail_m'], t['rot_deg'], t['tasks'], t['goal']) for g in ('1', '2') for t in v['schedule'][g]]


def run(base, seeds, combos, tag):
    rset = setup(base)
    src = {'g55R90': ('p1_g55/g55_R90_candidates.json', 'R90'), 'g55R35': ('p1_g55/g55_R35_candidates.json', 'R35'),
           'g52R90': ('p1_g52/g52_candidates.json', 'R90')}[base]
    ref = {r['seed']: r for r in json.load(open(os.path.join(R, src[0])))}
    out = os.path.join(HERE, f'g64c_{tag}_{base}.json')
    rows = json.load(open(out)) if os.path.exists(out) else []
    done = {r['seed'] for r in rows}
    O = objects()
    log = lambda s: print(s, flush=True)                                      # noqa: E731
    with Pool(NPROC) as pool:
        for seed in seeds:
            if seed in done:
                continue
            rb = ref[seed]
            assert rb['p0'] == {'1': 1.5, '2': 0.0} and G4.P0_LIN == {1: 1.5, 2: 0.0}, (rb['p0'], G4.P0_LIN)
            log(f'seed {seed} {base} {rset}  [{time.strftime("%T")}]')
            res, best = solve_combos(rb['nodes'], O, pool, log, combos)
            rec = dict(seed=seed, base=base, combos=[dict(modes={str(g): v for g, v in x['modes'].items()}, ok=x['ok'],
                                                          why=x.get('why'), makespan=x.get('makespan'),
                                                          bounds=x.get('bounds')) for x in res])
            if best is not None:
                rec['best'] = record(res[best])
                rr = [i for i, x in enumerate(res) if x['ok'] and x['modes'] == {1: 'REST', 2: 'REST'}]
                if rr:
                    v = record(res[rr[0]])
                    a = json.loads(json.dumps(v, default=float))
                    b = rb[src[1]]
                    rec['rest_identical'] = bool(_key(a) == _key(b) and a['makespan'] == b['makespan'])
                    log(f"  K-REST s{seed}: makespan {a['makespan']:.3f} vs sumber {b['makespan']:.3f}, "
                        f"bit-identik {rec['rest_identical']}")
                log(f"  TERPILIH mode {rec['best']['mode']} makespan {rec['best']['makespan']:.3f} (sementara "
                    f"{PROVISIONAL})")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'fine':
        fine()
    elif cmd == 'poses':
        poses()
    elif cmd in ('krest', 'combos'):
        run(sys.argv[2], [int(x) for x in sys.argv[3:]], COMBOS[:1] if cmd == 'krest' else COMBOS, cmd)
