"""G60b (docs/p1_g60_excl_sched.md B): G60 with (1) a richer candidate family -- one min-Tr chain per (task set, ordered
stop SIZES c = (|U_1|, ..., |U_k|), first stop at p0) instead of per (task set, #traverses, f0) -- and (2) a second DP
that only admits sequences with at least one stop at |rot| > WIDE (35 deg) -> LB_wide, a bound on EVERY schedule that
uses a wide rotation (the A2.3 question with a certificate). Same model (fastsim), same bound (LB = Tr_1 + Tr_2 +
max(W_1, W_2); W depends only on (#tasks, #traverses, f0), so LB* is unchanged: min over sizes of the min-Tr = min-Tr per
#traverses), same feasibility, predicates, lazy loop and cache COPIES as g60_make.

    python3 g60b_make.py k              -> K-B: per (A, m, f0), min over sizes of tr == g60_make.GantryM.tr (verified G55 s7 s10)
    python3 g60b_make.py run R90 S..  /  run R35 S..
"""
import json
import os
import sys
import time
from dataclasses import replace
from itertools import combinations
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g60_make as Z  # noqa: E402

M, K, G41, G4, W, S, MI = Z.M, Z.K, Z.G41, Z.G4, Z.W, Z.S, Z.MI
SIMF, w_of, events_of, to_solution, _Sol = Z.SIMF, Z.w_of, Z.events_of, Z.to_solution, Z._Sol
WIDE = 35.0


def compositions(c):
    if c == 0:
        return [()]
    return [(k,) + r for k in range(1, c + 1) for r in compositions(c - k)]


class GantryC:
    """h[R, sizes][q] / w[R, sizes][p] as GantryM with the stop sizes fixed; hw / ww = same restricted to sequences
    with >= 1 stop at |rot| > WIDE."""

    def __init__(self, inst, g, E, forb):
        n = inst.n
        dur = K._dur_table_f(inst, g, forb)
        keep = np.flatnonzero(np.isfinite(dur[1:]).any(axis=0))
        p0 = inst.p0[g]
        if p0 not in keep:
            keep = np.sort(np.append(keep, p0))
        self.g, self.n, self.keep, self.r0 = g, n, keep, int(np.searchsorted(keep, p0))
        T = Z.sched.traverse_matrix(inst.poses[g][keep], Z.C0, inst.rail_cmd, inst.rot_cmd)
        gid = np.asarray(inst.meta['cols'])[keep]
        T[~E.ok[g][np.ix_(gid, gid)]] = np.inf
        np.fill_diagonal(T, np.inf)
        self.T = T
        self.feas = np.isfinite(dur[:, keep])
        self.wide = np.abs(np.degrees(inst.poses[g][keep, 1])) > WIDE + 1e-6
        Kp, inf = len(keep), np.full(len(keep), np.inf)
        self.h, self.w, self.hw, self.ww = {(0, ()): np.zeros(Kp)}, {}, {(0, ()): inf}, {}
        sub = {}
        for R_ in sorted(range(1, 1 << n), key=lambda x: bin(x).count('1')):
            bits = [i for i in range(n) if R_ >> i & 1]
            c = len(bits)
            for comp in compositions(c):
                k0, rest = comp[0], comp[1:]
                key = (k0, R_)
                if key not in sub:
                    sub[key] = [sum(1 << i for i in u) for u in combinations(bits, k0)]
                wv, wwv = inf.copy(), inf.copy()
                for U in sub[key]:
                    f = self.feas[U]
                    if not f.any():
                        continue
                    hr, hwr = self.h[R_ ^ U, rest], self.hw[R_ ^ U, rest]
                    np.minimum(wv, np.where(f, hr, np.inf), out=wv)
                    np.minimum(wwv, np.where(f, np.where(self.wide, hr, hwr), np.inf), out=wwv)
                self.w[R_, comp], self.ww[R_, comp] = wv, wwv
                self.h[R_, comp], self.hw[R_, comp] = self._step(wv), self._step(wwv)

    def _step(self, v):
        cols = np.flatnonzero(np.isfinite(v))
        if not len(cols):
            return np.full(len(v), np.inf)
        return np.min(self.T[:, cols] + v[cols], axis=1)

    def tr(self, A, comp, f0, wide=False):
        a = (self.ww if wide else self.w) if f0 else (self.hw if wide else self.h)
        return float(a[A, comp][self.r0])

    def chain(self, A, comp, f0, wide=False):
        out, R_, q, rest, need = [], A, self.r0, comp, wide
        first = True
        while R_:
            if first and f0:
                p = self.r0
            else:
                p = int(np.argmin(self.T[q] + (self.ww if need else self.w)[R_, rest]))
            target = (self.ww if need else self.w)[R_, rest][p]
            k0, nxt = rest[0], rest[1:]
            bu = None
            for u in combinations([i for i in range(self.n) if R_ >> i & 1], k0):
                U = sum(1 << i for i in u)
                if not self.feas[U, p]:
                    continue
                v = (self.h if (not need or self.wide[p]) else self.hw)[R_ ^ U, nxt][p]
                if abs(v - target) < 1e-6:
                    bu = U
                    break
            assert bu is not None
            out.append((p, bu))
            if need and self.wide[p]:
                need = False
            R_ ^= bu
            rest, q, first = nxt, p, False
        assert not need
        return out

    def keys(self, A):
        c = bin(A).count('1')
        out = []
        for comp in compositions(c):
            for f0 in (False, True):
                m = len(comp) - int(f0)
                if m < 1:                                            # (ii): the gantry moves
                    continue
                out.append((comp, f0, m))
        return out


def search_b(inst, E, forb, wide=False, dps=None, fullfn=None):
    """wide=False: G60 search over the size family. wide=True: only pairs where >= 1 gantry uses a wide chain.
    -> dict(sol, mk, lb, key, lb_star, n_eval, n_cand, conflicts, cand, dps) or None."""
    full = (1 << inst.n) - 1
    if dps is None:
        dps = {g: GantryC(inst, g, E, forb[g]) for g in (1, 2)}
    tab = {}
    for g in (1, 2):
        for A in range(1, full + 1):
            row = []
            for comp, f0, m in dps[g].keys(A):
                t, tw = dps[g].tr(A, comp, f0), dps[g].tr(A, comp, f0, True)
                if np.isfinite(t) or np.isfinite(tw):
                    row.append((comp, f0, w_of(bin(A).count('1'), m, f0), t, tw))
            tab[g, A] = row
    cand = []
    for A1 in range(1, full):
        A2 = full ^ A1
        for c1, f1, w1, t1, tw1 in tab[1, A1]:
            for c2, f2, w2, t2, tw2 in tab[2, A2]:
                ww = max(w1, w2)
                if not wide:
                    if np.isfinite(t1) and np.isfinite(t2):
                        cand.append((t1 + t2 + ww, A1, c1, f1, False, A2, c2, f2, False))
                else:
                    if np.isfinite(tw1) and np.isfinite(t2):
                        cand.append((tw1 + t2 + ww, A1, c1, f1, True, A2, c2, f2, False))
                    if np.isfinite(t1) and np.isfinite(tw2):
                        cand.append((t1 + tw2 + ww, A1, c1, f1, False, A2, c2, f2, True))
    if not cand:
        return None
    cand.sort(key=lambda x: x[0])
    best, n_eval, conflicts, n_full, n_blocked = None, 0, 0, 0, 0
    for c in cand:
        lb, A1, c1, f1, x1, A2, c2, f2, x2 = c
        if best is not None and lb >= best['mk'] - 1e-9:
            break
        chains = {1: dps[1].chain(A1, c1, f1, x1), 2: dps[2].chain(A2, c2, f2, x2)}
        mk, fin, trace = SIMF.fastsim({g: events_of(dps[g], chains[g]) for g in (1, 2)})
        n_eval += 1
        assert lb <= mk + 1e-6, (lb, mk)
        if best is not None and mk >= best['mk'] - 1e-9:
            continue
        sol = to_solution(inst, dps, chains, forb, trace)
        conf = S.sched_coll.schedule_conflict(inst, sol.stops)
        if conf is not None:
            conflicts += 1
            continue
        if fullfn is not None:                   # G60c: value = g54_sim WITH reservation screens (>= LB still)
            mk0, mk = mk, fullfn(inst, sol)
            n_full += 1
            if mk > mk0 + 1e-6:
                n_blocked += 1
            if best is not None and mk >= best['mk'] - 1e-9:
                continue
        best = dict(sol=sol, mk=mk, lb=lb, key=c[1:])
    if best is None:
        return dict(sol=None, mk=None, lb_star=cand[0][0], n_eval=n_eval, n_cand=len(cand), conflicts=conflicts,
                    cand=cand, dps=dps)
    best.update(lb_star=cand[0][0], n_eval=n_eval, n_cand=len(cand), conflicts=conflicts, cand=cand, dps=dps,
                n_full=n_full, n_blocked=n_blocked)
    return best


def chain_stops_b(inst, dps, forb, c):
    _, A1, c1, f1, x1, A2, c2, f2, x2 = c
    out = {}
    for g, (A, cc, f0, x) in ((1, (A1, c1, f1, x1)), (2, (A2, c2, f2, x2))):
        out[g] = []
        for p, U in dps[g].chain(A, cc, f0, x):
            _, assign = K.stop_duration_f(inst, g, U, int(dps[g].keep[p]), forb[g])
            out[g].append(dict(pose=int(dps[g].keep[p]), assign=assign, tasks=U))
    return _Sol(out)


FULL = os.environ.get('G60C', '0') == '1'
_CROSS = []


def make_fullfn(F, p0, seed):
    """G60c: g54_sim (reservation screens, CrossGantryChecker) of a candidate solution; DEADLOCK = inf."""
    import tempfile
    import warnings
    sys.path.insert(0, os.path.join(Z.R, 'p1_g54'))
    import g54_sim as SIM
    if not _CROSS:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            from interarm_collision import CrossGantryChecker
            _CROSS.append(CrossGantryChecker())

    def f(inst, sol):
        sj = {str(g): v for g, v in MI.sched_json(inst, sol).items()}
        for g, gl in G41.goals(inst, sol, F).items():
            for st, gg in zip(sj[str(g)], gl):
                st['goal'] = gg
        row = dict(seed=seed, p0=p0, X=dict(schedule=json.loads(json.dumps(sj, default=float)), makespan=0.0, serial=0.0))
        fd, path = tempfile.mkstemp(suffix='.json', dir=HERE)
        with os.fdopen(fd, 'w') as fh:
            json.dump([row], fh, default=float)
        try:
            out = SIM.sim(seed, 'X', path, _CROSS[0], log=lambda x: None)
        finally:
            os.remove(path)
        return np.inf if out['deadlock'] else out['makespan']
    return f


def solve60b(nodes, F, E, SX, pool, log, cap=300, fullfn=None):
    """= g60_make.solve60 with search_b; then LB_wide on the relaxed instance (a bound, no verification needed) and the
    best wide schedule on the verified instance (for the record)."""
    it = 0
    while True:
        it += 1
        inst, unver = G4.build(nodes, F)
        iu = replace(inst, reach={g: inst.reach[g] & ~unver[g] for g in inst.reach})
        while True:
            ver = search_b(iu, E, SX.forbids(iu))
            if ver is None or ver['sol'] is None:
                break
            new = [x for x in K.stop_items(iu, ver['sol'], F) if x[0] not in SX.c]
            if not new:
                break
            SX.compute(new, pool)
        forb = SX.forbids(inst)
        rel = search_b(inst, E, forb)
        if rel is None or ver is None or ver['sol'] is None:
            raise ValueError('no allocation with (ii)+(iii)')
        target = ver['mk'] / (1 + Z.EPS)
        ut, us, nk = set(), [], 0
        for c in rel['cand']:
            if c[0] >= target or nk >= cap:
                break
            nk += 1
            sol = chain_stops_b(inst, rel['dps'], forb, c)
            u = S.used_unverified(inst, sol, unver)
            ut |= set(u)
            if not u:
                us += [x for x in K.stop_items(inst, sol, F) if x[0] not in SX.c and x not in us]
        log(f"    iterasi {it}: terverifikasi {ver['mk']:.3f} (LB*ver {ver['lb_star']:.3f}), LB*rel {rel['lb_star']:.3f}, "
            f"kunci < target {nk}, tuple belum {len(ut)}, stop belum {len(us)}  [{time.strftime('%T')}]")
        if not ut and not us:
            if fullfn is not None:
                ver = search_b(iu, E, SX.forbids(iu), dps=ver['dps'], fullfn=fullfn)
                log(f"    G60c g54_sim penuh: {ver['mk']:.3f}, dinilai penuh {ver['n_full']}, terhalang {ver['n_blocked']}")
            wrel = search_b(inst, E, forb, wide=True, dps=rel['dps'])
            wver = search_b(iu, E, SX.forbids(iu), wide=True, dps=ver['dps'])
            return iu, ver, rel, wrel, wver, it
        if ut:
            F.compute(S.neighbours(inst, unver, sorted(ut)), pool)
        if us:
            SX.compute(us, pool)


def run(rset, seeds):
    Z.patch()
    out = Z.setup60(rset, f'g60{"c" if FULL else "b"}_{rset}')
    F, SX = W.Feas49(G41.pose_ok_b()), K.Stops()
    E = M.EdgeMatrix()
    K.solve_gantry_f = M.solve_gantry_e1
    G41.STOP_TORQ = True
    g42f = {r['seed']: r for r in json.load(open(M.G42F))}
    rows = json.load(open(out)) if os.path.exists(out) else []
    done = {r['seed'] for r in rows}
    log = lambda s: print(s, flush=True)                                 # noqa: E731
    with Pool(Z.NPROC) as pool:
        for seed in seeds:
            if seed in done:
                continue
            rb = g42f[seed]
            rec = {k: rb[k] for k in ('seed', 'drawn', 'nodes', 'kind', 'xyz', 'd_env', 'p0', 'p0_rot_deg', 'rot_cmd')}
            log(f'seed {seed} {rset}  [{time.strftime("%T")}]')
            t0 = time.time()
            try:
                inst, best, rel, wrel, wver, it = solve60b(
                    rb['nodes'], F, E, SX, pool, log,
                    fullfn=make_fullfn(F, rec['p0'], seed) if FULL else None)
            except ValueError as e:
                rec[rset] = dict(ok_i=False, why=str(e), wall_s=time.time() - t0)
                log(f'  seed {seed}: GAGAL -- {e}')
                rows.append(rec)
                json.dump(rows, open(out, 'w'), indent=1, default=float)
                continue
            sol = best['sol']
            dec = S.describe(inst, sol)
            conf = S.sched_coll.schedule_conflict(inst, sol.stops)
            sj = {str(g): v for g, v in MI.sched_json(inst, sol).items()}
            for g, gl in G41.goals(inst, sol, F).items():
                for st, gg in zip(sj[str(g)], gl):
                    st['goal'] = gg
            sjn = json.loads(json.dumps(sj, default=float))
            mx, dmx = M.rot_stats(sjn, rec['p0'])
            mk2 = SIMF.fastsim(SIMF.durations(sjn, {g: rec['p0'][str(g)] for g in (1, 2)}))[0]
            wsj = None
            if wver is not None and wver['sol'] is not None:
                wsj = {str(g): [dict(rail_m=float(inst.poses[g][x['pose'], 0]),
                                     rot_deg=round(float(np.degrees(inst.poses[g][x['pose'], 1])), 1),
                                     ntask=bin(x['tasks']).count('1')) for x in wver['sol'].stops[g]] for g in (1, 2)}
            rec[rset] = dict(ok_i=True, ok_ii=all(v['moves'] >= 1 for v in dec.values()), ok_iii=conf is None,
                             makespan=best['mk'], makespan_json=mk2, lb_star=rel['lb_star'], lb_star_ver=best['lb_star'],
                             gap=best['mk'] / rel['lb_star'] - 1, key=json.loads(json.dumps(best['key'], default=str)),
                             n_eval=best['n_eval'], n_cand=best['n_cand'], iii_refused=best['conflicts'], iters=it,
                             n_full=best.get('n_full'), n_blocked=best.get('n_blocked'),
                             lb_wide=wrel['lb_star'] if wrel else None, wide_best=wver['mk'] if wver else None,
                             wide_sched=wsj, wall_s=time.time() - t0,
                             moves={str(g): v['moves'] for g, v in dec.items()},
                             ntask={str(g): v['ntask'] for g, v in dec.items()}, serial=sum(sol.finish.values()),
                             stops_s42=[dict(key=list(k), ok=SX.c[k]['ok']) for k, _ in K.stop_items(inst, sol, F)],
                             rot_max_deg=mx, drot_max_deg=dmx, conflict=repr(conf), schedule=sj)
            log(f"  seed {seed}: makespan eksklusif {best['mk']:.2f} (json {mk2:.2f}), LB* {rel['lb_star']:.2f}, "
                f"celah {100 * (best['mk'] / rel['lb_star'] - 1):.2f} %, |rot| maks {mx:.0f}; LB_lebar(>{WIDE:.0f}) "
                f"{rec[rset]['lb_wide'] if rec[rset]['lb_wide'] is None else round(rec[rset]['lb_wide'], 2)}, terbaik lebar "
                f"{rec[rset]['wide_best'] if rec[rset]['wide_best'] is None else round(rec[rset]['wide_best'], 2)}; konflik "
                f"{conf!r}, iterasi {it}, {time.time() - t0:.0f} s  [{time.strftime('%T')}]")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)


def kb():
    """K-B on the verified G55 instances (R90 s7 s10): min over sizes with |sizes| - f0 = m of tr == GantryM.tr(A, m, f0)
    for every A, m, f0; and every chain (normal + wide) run alone gives tr + W; wide chains really use |rot| > WIDE."""
    Z.patch()
    E = M.EdgeMatrix()
    Z.setup60('R90', 'g60b_k')
    F, SX = W.Feas49(G41.pose_ok_b()), K.Stops()
    g42f = {r['seed']: r for r in json.load(open(M.G42F))}
    n = bad = nw = badw = 0
    for s in (7, 10):
        inst, unver = G4.build(g42f[s]['nodes'], F)
        iu = replace(inst, reach={g: inst.reach[g] & ~unver[g] for g in inst.reach})
        forb = SX.forbids(iu)
        for g in (1, 2):
            a, b = Z.GantryM(iu, g, E, forb[g]), GantryC(iu, g, E, forb[g])
            for A in range(1, 1 << iu.n):
                c = bin(A).count('1')
                for m in range(1, c + 1):
                    for f0 in (False, True):
                        ref = a.tr(A, m, f0)
                        got = min([b.tr(A, cc, f0) for cc in compositions(c) if len(cc) - int(f0) == m] or [np.inf])
                        n += 1
                        bad += not ((np.isinf(ref) and np.isinf(got)) or abs(ref - got) < 1e-6)
                for cc, f0, m in b.keys(A):
                    for x in (False, True):
                        t = b.tr(A, cc, f0, x)
                        if not np.isfinite(t):
                            continue
                        ch = b.chain(A, cc, f0, x)
                        ev = events_of(b, ch)
                        last = max(i for i, e in enumerate(ev) if e[0] == 'task')
                        nw += 1
                        ok = abs(sum(d for _, d in ev[:last + 1]) - (t + w_of(c, m, f0))) < 1e-6 and \
                            [bin(U).count('1') for _, U in ch] == list(cc) and (not x or any(b.wide[p] for p, _ in ch))
                        badw += not ok
        print(f's{s}: min-ukuran = GantryM {n - bad}/{n}; rantai (biasa + lebar) konsisten {nw - badw}/{nw}', flush=True)


if __name__ == '__main__':
    if sys.argv[1] == 'k':
        kb()
    else:
        run(sys.argv[2], [int(x) for x in sys.argv[3:]])
