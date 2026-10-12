"""G60 (docs/p1_g60_excl_sched.md): scheduler that SEES the parallel runner's exclusive traverses. OFFLINE, no ROS.

Objective = the run_g54 makespan model (g60_sim.fastsim == g54_sim with 0 reservation blocks, K-SIM 19/19), P1'' costs.
Feasibility, predicates and lazy verification = G55 (ok49 tuples, (s42) stops, EdgeMatrix (b), G46 E3, mirror, (ii), (iii)),
caches = COPIES in this directory.

Bound (valid for ANY pair of per-gantry event sequences under the runner's gates): every event of gantry g up to its last
task and every traverse of the other gantry o are pairwise disjoint in time (a traverse starts only when o runs nothing
and nothing of o starts while it runs), and all of them end by the makespan, so
    makespan >= W_g + Tr_g + Tr_o   for g = 1, 2      ->   LB = Tr_1 + Tr_2 + max(W_1, W_2)
with Tr = sum of traverse durations, W = tasks x a_t + retracts before traverses (T_SKIP for a first move with every arm
at REST, else a_r). W depends only on (#tasks, #traverses m, first stop at p0 f0); Tr is minimised per (task set, m, f0)
by a DP = sched.solve_gantry's recursion with one extra index m (exact, same feasibility arrays as K.solve_gantry_f).

Search: every allocation (A_1, A_2) x (m_1, f0_1) x (m_2, f0_2) with (ii) (m >= 1 on both gantries) gives LB and a candidate
= the min-Tr sequences of those keys; candidates are evaluated with fastsim in increasing LB until LB >= incumbent,
(iii) sched_coll on the fastsim timeline (stop start = end of its traverse). Result = best evaluated candidate; the
certificate is LB* = min LB over every allocation and key (a bound on EVERY schedule, not only on the candidates).
Lazy loop = G46 solve_lazy46: relaxed instance (unknown tuples feasible) for LB*, verified-only for the schedule, stop
when the relaxed best is verified or verified <= (1 + EPS) x relaxed best.

    python3 g60_make.py k        -> K-DP (see kdp)
    python3 g60_make.py run R90 S..  /  run R35 S..
"""
import json
import os
import shutil
import sys
import time
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g55'))
sys.path.insert(0, HERE)
import g55_make as G55  # noqa: E402
import g60_sim as SIMF  # noqa: E402

M, K, G41, G4, W = G55.M, G55.K, G55.G41, G55.G4, G55.W
S, sched, MI = G4.S, G4.sched, G4.MI
P2 = G55.P2
A_T, A_R, T_SKIP = P2['a_t'], P2['a_r'], SIMF.T_SKIP
C0 = P2['l_bar'] + P2['g'] * G55.C25['c_rovh']          # traverse = C0 + g * max(r T_lin, ROT_CMD T_rot)
EPS = 0.03
NPROC = int(os.environ.get('G60_NPROC', 15))


def setup60(rset, tag):
    K.FIX_CHOICES = True
    K.VAR = 'G49'
    G4.RSET = M.RSETS[rset]

    def cp(src, name):
        dst = os.path.join(HERE, name)
        if not os.path.exists(dst):
            shutil.copy(src, dst)
        return dst
    K.STOP_CACHE = cp(os.path.join(R, 'p1_g55/g55_stop49_cache.jsonl'), 'g60_stop49_cache.jsonl')
    W.TUPLE_CACHE = G41.TUPLE_CACHE = cp(os.path.join(R, 'p1_g55/g55_tuple49_cache.jsonl'), 'g60_tuple49_cache.jsonl')
    return os.path.join(HERE, f'{tag}_candidates.json')


def patch():
    """G55 cost patch (P1'') -- the instance's traverse model = fastsim's."""
    S.RS.p1prime = G55.p1pp
    S.ROT_CMD = P2['g'] * G55.ROT_CMD0


def w_of(n, m, f0):
    if m == 0:
        return n * A_T
    return n * A_T + (m * A_R if f0 else T_SKIP + (m - 1) * A_R)


# ------------------------------------------------------------------------------------------------ per-gantry DP
class GantryM:
    """h[m][R, q]: min traverse total to finish R from pose q with exactly m traverses, first action a traverse;
    w[m][R, p]: same when a stop at p (no traverse) comes first. Feasibility = K._dur_table_f (reach, (s42) forbids),
    edges = EdgeMatrix (b), no consecutive stops at one pose (= G42 B K-lazy)."""

    def __init__(self, inst, g, E, forb):
        n = inst.n
        dur = K._dur_table_f(inst, g, forb)
        keep = np.flatnonzero(np.isfinite(dur[1:]).any(axis=0))
        p0 = inst.p0[g]
        if p0 not in keep:
            keep = np.sort(np.append(keep, p0))
        self.g, self.n, self.keep, self.r0 = g, n, keep, int(np.searchsorted(keep, p0))
        T = sched.traverse_matrix(inst.poses[g][keep], C0, inst.rail_cmd, inst.rot_cmd)
        gid = np.asarray(inst.meta['cols'])[keep]
        T[~E.ok[g][np.ix_(gid, gid)]] = np.inf
        np.fill_diagonal(T, np.inf)
        self.T = T
        self.feas = np.isfinite(dur[:, keep])
        Kp, NS = len(keep), 1 << n
        self.h = np.full((n + 1, NS, Kp), np.inf)
        self.w = np.full((n + 1, NS, Kp), np.inf)
        self.h[0, 0] = 0.0
        for R_ in sorted(range(1, NS), key=lambda x: bin(x).count('1')):
            c = bin(R_).count('1')
            U = R_
            while U:
                f = self.feas[U]
                if f.any():
                    np.minimum(self.w[:c + 1, R_], np.where(f[None, :], self.h[:c + 1, R_ ^ U], np.inf),
                               out=self.w[:c + 1, R_])
                U = (U - 1) & R_
            for m in range(1, c + 1):
                wr = self.w[m - 1, R_]
                cols = np.flatnonzero(np.isfinite(wr))
                if len(cols):
                    self.h[m, R_] = np.min(T[:, cols] + wr[cols], axis=1)

    def tr(self, A, m, f0):
        return float(self.w[m, A, self.r0] if f0 else self.h[m, A, self.r0])

    def chain(self, A, m, f0):
        """Argmin stop sequence [(keep index, task mask), ...] for tr(A, m, f0)."""
        out, Rm, q, ml = [], A, self.r0, m
        if f0:
            U = self._pick(Rm, self.r0, ml)
            out.append((self.r0, U))
            Rm ^= U
        while Rm:
            wr = self.w[ml - 1, Rm]
            p = int(np.argmin(self.T[q] + wr))
            ml -= 1
            U = self._pick(Rm, p, ml)
            out.append((p, U))
            Rm ^= U
            q = p
        assert ml == 0
        return out

    def _pick(self, Rm, p, m):
        best, bu, U = np.inf, None, Rm
        while U:
            if self.feas[U, p] and self.h[m, Rm ^ U, p] < best:
                best, bu = self.h[m, Rm ^ U, p], U
            U = (U - 1) & Rm
        assert np.isfinite(best) and abs(best - self.w[m, Rm, p]) < 1e-6
        return bu


# ------------------------------------------------------------------------------------------------ search
def events_of(dp, ch):
    ev, q, at_rest = [], dp.r0, True
    for p, U in ch:
        if p != q:
            ev.append(('retract', T_SKIP if at_rest else A_R))
            ev.append(('traverse', float(dp.T[q, p])))
            q = p
        ev += [('task', A_T)] * bin(U).count('1')
        at_rest = False
    ev.append(('retract', A_R))
    return ev


def to_solution(inst, dps, chains, forb, trace):
    """sched.Solution from the chains; stop start = fastsim end of its traverse (0 / first task for a stop at p0)."""
    stops, finish, alloc = {}, {}, {}
    for g in (1, 2):
        dp, ch = dps[g], chains[g]
        tev = [x for x in trace if x[1] == g]
        k, st = 0, []
        q = dp.r0
        for p, U in ch:
            if p != q:
                k += 2                                     # retract, traverse
                start = tev[k - 1][4]
                q = p
            else:
                start = tev[k][0]
            d, assign = K.stop_duration_f(inst, g, U, int(dp.keep[p]), forb[g])
            st.append(dict(pose=int(dp.keep[p]), start=float(start), dur=float(d), tasks=int(U), assign=assign))
            k += bin(U).count('1')
        stops[g] = st
        alloc[g] = int(sum(U for _, U in ch))
        finish[g] = max(x[4] for x in tev if x[3] == 'task')
    return sched.Solution(float(max(finish.values())), alloc, stops, finish, 0.0, 0)


def search(inst, E, forb, log=None):
    """-> dict(sol, mk, lb_star, n_eval, n_cand, conflicts) or None (no allocation with (ii))."""
    n, full = inst.n, (1 << inst.n) - 1
    dps = {g: GantryM(inst, g, E, forb[g]) for g in (1, 2)}
    keys = {}
    for g in (1, 2):
        dp = dps[g]
        for A in range(1, full + 1):
            c = bin(A).count('1')
            keys[g, A] = [(m, f0, dp.tr(A, m, f0)) for m in range(1, c + 1) for f0 in (False, True)
                          if np.isfinite(dp.tr(A, m, f0))]
    cand = []
    for A1 in range(1, full):
        A2 = full ^ A1
        n1, n2 = bin(A1).count('1'), bin(A2).count('1')
        for m1, f1, t1 in keys[1, A1]:
            w1 = w_of(n1, m1, f1)
            for m2, f2, t2 in keys[2, A2]:
                cand.append((t1 + t2 + max(w1, w_of(n2, m2, f2)), A1, m1, f1, A2, m2, f2))
    if not cand:
        return None
    cand.sort()
    lb_star = cand[0][0]
    best, n_eval, conflicts = None, 0, 0
    for lb, A1, m1, f1, A2, m2, f2 in cand:
        if best is not None and lb >= best['mk'] - 1e-9:
            break
        chains = {1: dps[1].chain(A1, m1, f1), 2: dps[2].chain(A2, m2, f2)}
        mk, fin, trace = SIMF.fastsim({g: events_of(dps[g], chains[g]) for g in (1, 2)})
        n_eval += 1
        assert lb <= mk + 1e-6, (lb, mk)                   # the bound must hold on every evaluated candidate
        if best is not None and mk >= best['mk'] - 1e-9:
            continue
        sol = to_solution(inst, dps, chains, forb, trace)
        conf = S.sched_coll.schedule_conflict(inst, sol.stops)
        if conf is not None:
            conflicts += 1
            continue
        best = dict(sol=sol, mk=mk, lb=lb, key=(A1, m1, f1, A2, m2, f2))
    if best is None:
        return None
    best.update(lb_star=lb_star, n_eval=n_eval, n_cand=len(cand), conflicts=conflicts, cand=cand, dps=dps)
    return best


class _Sol:
    def __init__(self, stops):
        self.stops = stops


def chain_stops(inst, dps, forb, A1, m1, f1, A2, m2, f2):
    out = {}
    for g, (A, m, f0) in ((1, (A1, m1, f1)), (2, (A2, m2, f2))):
        out[g] = []
        for p, U in dps[g].chain(A, m, f0):
            _, assign = K.stop_duration_f(inst, g, U, int(dps[g].keep[p]), forb[g])
            out[g].append(dict(pose=int(dps[g].keep[p]), assign=assign, tasks=U))
    return _Sol(out)


def solve60(nodes, F, E, SX, pool, log, cap=300):
    """Verified-only instance -> the schedule. Relaxed instance (unknown tuples / (s42) stops feasible) -> LB*.
    Every relaxed key with LB < verified / (1 + EPS) whose min-Tr chains use an unverified tuple or (s42) stop gets
    them verified (+ G31 neighbours), at most `cap` keys per round; stop when there is none -- then any better
    schedule needs a key whose min-Tr chains are verified, i.e. LB_rel = LB_ver for it."""
    it = 0
    while True:
        it += 1
        inst, unver = G4.build(nodes, F)
        iu = replace(inst, reach={g: inst.reach[g] & ~unver[g] for g in inst.reach})
        while True:
            ver = search(iu, E, SX.forbids(iu))
            if ver is None:
                break
            new = [x for x in K.stop_items(iu, ver['sol'], F) if x[0] not in SX.c]
            if not new:
                break
            SX.compute(new, pool)
        forb = SX.forbids(inst)
        rel = search(inst, E, forb)
        if rel is None or ver is None:
            raise ValueError(f'no allocation with (ii)+(iii): relaxed {rel is not None}, verified {ver is not None}')
        target = ver['mk'] / (1 + EPS)
        ut, us, nk = set(), [], 0
        for c in rel['cand']:
            if c[0] >= target or nk >= cap:
                break
            nk += 1
            sol = chain_stops(inst, rel['dps'], forb, *c[1:])
            u = S.used_unverified(inst, sol, unver)
            ut |= set(u)
            if not u:
                us += [x for x in K.stop_items(inst, sol, F) if x[0] not in SX.c and x not in us]
        log(f"    iterasi {it}: terverifikasi {ver['mk']:.3f} (LB*ver {ver['lb_star']:.3f}), LB*rel {rel['lb_star']:.3f}, "
            f"kunci < target {nk}, tuple belum {len(ut)}, stop belum {len(us)}  [{time.strftime('%T')}]")
        if not ut and not us:
            return iu, ver, rel, it
        if ut:
            F.compute(S.neighbours(inst, unver, sorted(ut)), pool)
        if us:
            SX.compute(us, pool)


def run(rset, seeds):
    patch()
    out = setup60(rset, f'g60_{rset}')
    F, SX = W.Feas49(G41.pose_ok_b()), K.Stops()
    E = M.EdgeMatrix()
    K.solve_gantry_f = M.solve_gantry_e1
    G41.STOP_TORQ = True
    g42f = {r['seed']: r for r in json.load(open(M.G42F))}
    rows = json.load(open(out)) if os.path.exists(out) else []
    done = {r['seed'] for r in rows}
    log = lambda s: print(s, flush=True)                                 # noqa: E731
    with Pool(NPROC) as pool:
        for seed in seeds:
            if seed in done:
                continue
            rb = g42f[seed]
            rec = {k: rb[k] for k in ('seed', 'drawn', 'nodes', 'kind', 'xyz', 'd_env', 'p0', 'p0_rot_deg', 'rot_cmd')}
            log(f'seed {seed} {rset}  [{time.strftime("%T")}]')
            t0 = time.time()
            try:
                inst, best, rel, it = solve60(rb['nodes'], F, E, SX, pool, log)
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
            # the fastsim of the SAVED schedule (through g31_screen.events) must equal the search value
            mk2 = SIMF.fastsim(SIMF.durations(sjn, {g: rec['p0'][str(g)] for g in (1, 2)}))[0]
            rec[rset] = dict(ok_i=True, ok_ii=all(v['moves'] >= 1 for v in dec.values()), ok_iii=conf is None,
                             makespan=best['mk'], makespan_json=mk2, lb_star=rel['lb_star'], lb_best=best['lb'],
                             gap=best['mk'] / rel['lb_star'] - 1, lb_star_ver=best['lb_star'], relaxed_best=rel['mk'], key=list(best['key']),
                             n_eval=best['n_eval'], n_cand=best['n_cand'], iii_refused=best['conflicts'], iters=it,
                             wall_s=time.time() - t0, moves={str(g): v['moves'] for g, v in dec.items()},
                             ntask={str(g): v['ntask'] for g, v in dec.items()}, serial=sum(sol.finish.values()),
                             stops_s42=[dict(key=list(k), ok=SX.c[k]['ok']) for k, _ in K.stop_items(inst, sol, F)],
                             rot_max_deg=mx, drot_max_deg=dmx, conflict=repr(conf), schedule=sj)
            log(f"  seed {seed}: makespan eksklusif {best['mk']:.2f} (json {mk2:.2f}), LB* {rel['lb_star']:.2f}, "
                f"celah {100 * (best['mk'] / rel['lb_star'] - 1):.2f} %, |rot| maks {mx:.0f}, drot maks {dmx:.0f}, "
                f"konflik {conf!r}, (ii) {rec[rset]['ok_ii']}, iterasi {it}, {time.time() - t0:.0f} s  [{time.strftime('%T')}]")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)


def kdp():
    """K-DP on the verified G55 instances (no tuple computed): (a) every min-Tr chain run ALONE through events_of
    gives exactly tr + W up to its last task; (b) for every schedule G55 evaluated (g55_eval.json), per gantry the DP
    minimum tr(A, m, f0) <= that schedule's own traverse total (fastsim durations), and LB <= its g54_sim makespan."""
    patch()
    E = M.EdgeMatrix()
    g42f = {r['seed']: r for r in json.load(open(M.G42F))}
    ev55 = json.load(open(os.path.join(R, 'p1_g55/g55_eval.json')))
    na = ba = nb = bb = 0
    for rset in ('R90', 'R35'):
        setup60(rset, 'g60_kdp')
        F, SX = W.Feas49(G41.pose_ok_b()), K.Stops()
        seeds = sorted({x['seed'] for x in ev55 if x['cls'] == rset})
        for s in seeds:
            inst, unver = G4.build(g42f[s]['nodes'], F)
            iu = replace(inst, reach={g: inst.reach[g] & ~unver[g] for g in inst.reach})
            forb = SX.forbids(iu)
            dps = {g: GantryM(iu, g, E, forb[g]) for g in (1, 2)}
            for g in (1, 2):
                dp = dps[g]
                for A in range(1, 1 << iu.n):
                    for m in range(1, bin(A).count('1') + 1):
                        for f0 in (False, True):
                            tr = dp.tr(A, m, f0)
                            if not np.isfinite(tr):
                                continue
                            ev = events_of(dp, dp.chain(A, m, f0))
                            last = max(i for i, e in enumerate(ev) if e[0] == 'task')
                            na += 1
                            ba += abs(sum(d for _, d in ev[:last + 1]) - (tr + w_of(bin(A).count('1'), m, f0))) > 1e-6
            for x in ev55:
                if x['seed'] != s or x['cls'] != rset:
                    continue
                row = {q['seed']: q for q in json.load(open(os.path.join(R, x['plan'])))}[s]
                sj = row[x['variant']]['schedule']
                if any(abs(st['rot_deg']) > max(abs(M.RDEG[i]) for i in M.RSETS[rset]) + 1e-6 for g in sj for st in sj[g]):
                    continue
                Ed = SIMF.durations(sj, {g: row['p0'][str(g)] for g in (1, 2)})
                lb_parts = {}
                for g in (1, 2):
                    A = sum(1 << int(t) for st in sj[str(g)] for t in st['tasks'])
                    m = sum(k == 'traverse' for k, _ in Ed[g])
                    f0 = Ed[g][0][0] == 'task'
                    tr_own = sum(d for k, d in Ed[g] if k == 'traverse')
                    nb += 1
                    ok = dps[g].tr(A, m, f0) <= tr_own + 1e-6
                    bb += not ok
                    if not ok:
                        print(f'  s{s} {rset} {x["plan"]}:{x["variant"]} g{g}: DP {dps[g].tr(A, m, f0):.3f} > own {tr_own:.3f}')
                    lb_parts[g] = (tr_own, w_of(bin(A).count('1'), m, f0))
                lb = lb_parts[1][0] + lb_parts[2][0] + max(lb_parts[1][1], lb_parts[2][1])
                print(f'  s{s} {rset} {x["plan"]}:{x["variant"]}: LB {lb:.2f} <= sim {x["sim"]:.2f} {lb <= x["sim"] + 1e-6}',
                      flush=True)
                bb += lb > x['sim'] + 1e-6
    print(f'K-DP (a) rantai sendiri = Tr + W: {na - ba}/{na}; (b) DP <= jadwal G55 + LB <= sim: {bb} pelanggaran '
          f'({nb} gantry)', flush=True)


if __name__ == '__main__':
    if sys.argv[1] == 'k':
        kdp()
    else:
        run(sys.argv[2], [int(x) for x in sys.argv[3:]])
