"""Layer 3 -- the coupled solver G9 failed to establish. docs/p1_g10_sched4.md.

p1_g9 B4 is the reason this file exists, and it is worth restating exactly: the
MODEL of gantry-gantry collision was built and verified (W0/W0b/W0c predicate,
W1a-d trajectory + continuous-time certificate), and the SOLVER on top of it was
not. Two causes were measured, not guessed, and this file treats them as two
separate problems because that is what they are:

  p1_g9 B6   the start-time candidate set was too coarse. On the binding
             instances both gantries sweep through rot ~ -90 deg at the same
             lin at the same second; the repair is to delay one of them by a
             few seconds, and that time is not a boundary of anything. G9's set
             was {own readiness} U {other's leg boundaries} -- raw boundaries,
             no way to reach the instant a blocked interval actually ends.
             -> A2.2, `feasible_starts`, below. Derived, with a Lipschitz
                constant proved in A2.2 rather than a probe count chosen by
                hand.

  p1_g9 B4   the search was incomplete. With collisions switched OFF, 1 of 12
             W2b instances came back with exactly the incumbent it was seeded
             with: nothing in the machine could produce a complete schedule
             except expanding all the way to R = 0.
             -> A2.3, `_dive`, below. Every popped node is handed a feasible
                completion, so an incumbent exists from the first pop.

WHAT THIS FILE DOES NOT DO, and why -- docs/p1_g10_sched4.md A2.1 in full:
p1_g9 C recommends, in bold, a DP over (R1, p1, R2, p2) with a Pareto frontier
over (t1, t2). That is rejected here, before the code was written, for two
reasons that are stated as claims so they can be attacked:

  * (p_h, t_h) is NOT a sufficient statistic for A2.4. It says where the other
    gantry ENDS and when it is free, not where it was during its leg. Two
    states identical in (R, p1, p2, t1, t2) -- one whose last leg swept from
    +90 deg, one from -90 deg -- differ in whether a concurrent sweep collides.
    The DP would be unsound unless it forbade concurrent legs, and forbidding
    concurrency throws away the parallelism that having two gantries is for.
  * Componentwise time dominance is unsound (A2.1 Lemma 6): arriving EARLIER at
    a pose that blocks the other gantry occupies it LONGER and can cost
    makespan. So a Pareto frontier on (t1, t2) may discard the optimum.

The state carried here is the minimal sufficient one: each gantry's committed
Traj, which answers pose_at(t) for every t including past its end.

Everything imported from sched_coll is used UNMODIFIED -- that file is frozen
(A0), because W0/W1 verified that file rather than the idea behind it.
"""

from __future__ import annotations

import heapq
import itertools
import time
from dataclasses import dataclass, field

import numpy as np

from reachability_gng import sched
from reachability_gng.sched_coll import (C_CLEAR, EPS_CERT, EVADE_CANDIDATES,
                                         Traj, V_POINT, V_REL, _repair_ub,
                                         _serial_ub, dp_pose_index,
                                         leg_duration, leg_pose, may_block,
                                         pair_distance, safe_poses,
                                         schedule_conflict)

# --- A2.2, all locked in docs/p1_g10_sched4.md before this file existed -----
EPS_S = 0.01           # s, refinement tolerance on a start time
K_START = 8            # start times kept per action
N_PROBE = 24           # anchors across the interference horizon
N_MULTI = 24           # actions per node that get the multi-start treatment
# --- forced by contradiction 1 (see feasible_starts), NOT locked in A ------
DS_COARSE = 0.20       # s, start-grid quantum before refinement
NU_MAX = 4000          # own-clock samples per action window

# --- search budgets, carried over from p1_g9 B3 ----------------------------
ACTION_BUDGET = 400
NODE_BUDGET = 200_000
EVADE_ATTEMPTS = 60        # evade targets probed per solve, total
EVADE_PUSH = 4             # evade successors pushed per node


@dataclass
class Budget:
    """Every place the search is allowed to give up, counted. K5.8.

    A sparsification that is not reported is a sparsification that is hidden,
    so each of these is printed with the results whether or not it fired.
    """
    k_start_hit: int = 0        # an action had more feasible starts than K_START
    advance_hit: int = 0        # an anchor ran out of MAX_ADVANCE steps
    eps_s_floor: int = 0        # the Lipschitz step degenerated to the floor
    action_hit: int = 0         # a node hit ACTION_BUDGET
    multi_skipped: int = 0      # actions given only their earliest start
    walks: int = 0              # certificate calls -- the thing that is slow
    evade_tried: int = 0        # evade targets probed (cap EVADE_ATTEMPTS)
    dives: int = 0
    dive_proved: int = 0

    def as_dict(self):
        return dict(self.__dict__)


# ===========================================================================
# A2.2 -- the start-time candidate set
# ===========================================================================
def _blocked_over(s_vals, u, own_lin, own_rot, g_own, other, c_clear, eps,
                  bud=None, cell_budget=1_000_000):
    """Is the action blocked, for EVERY start time in `s_vals` at once?

    One vectorised sweep over the (start time x own-clock offset) grid. Sound in
    the same sense as A2.4's certificate and for the same reason: the own-clock
    samples are spaced so that no point of either body can move more than eps/2
    between neighbours, so a true violation always shows up in a sample.

    Prefiltered by `may_block` -- the (N1)/(N2) necessary conditions, which
    p1_g9 W0 checked for soundness on 3552 pairs. Almost every sample in a real
    sweep fails (N1) outright (both gantries must be > 31.7 deg off
    rail-parallel), so the exact test runs on a small minority of the grid.
    """
    S, U = len(s_vals), len(u)
    out = np.zeros(S, bool)
    rows = max(1, int(cell_budget // max(U, 1)))
    for a in range(0, S, rows):
        ss = np.asarray(s_vals[a:a + rows], float)
        t = ss[:, None] + u[None, :]
        ol, orot = other.pose_at(t)
        wl = np.broadcast_to(own_lin, t.shape)
        wr = np.broadcast_to(own_rot, t.shape)
        l1, r1, l2, r2 = ((wl, wr, ol, orot) if g_own == 1
                          else (ol, orot, wl, wr))
        m = may_block(l1, r1, l2, r2, c_clear + eps)
        if not m.any():
            continue
        ii, jj = np.nonzero(m)
        if bud:
            bud.walks += 1
        d = pair_distance(l1[ii, jj], r1[ii, jj], l2[ii, jj], r2[ii, jj])
        hit = d <= c_clear + eps
        if hit.any():
            out[a + np.unique(ii[hit])] = True
    return out


def feasible_starts(tr, q, dur, t_min, other, c_clear=C_CLEAR, eps=EPS_CERT,
                    k=K_START, eps_s=EPS_S, n_probe=N_PROBE, bud=None,
                    earliest_only=False, ds=DS_COARSE):
    """Start times at which `tr` may traverse to `q` and then hold `dur`.

    Returns [] if the action is infeasible at every start time.

    🔺 CONTRADICTION 1 with A2.2, and it is a contradiction about MECHANISM, not
    about the guarantee. A2.2 locked a walk in `s` that advances by the Lemma 7
    step (c_clear - d*) / V_POINT. Lemma 7 is correct and is not withdrawn --
    but the step DEGENERATES, because `pair_distance` clamps overlap to exactly
    0.0 and reports no penetration depth. Where the bodies actually intersect,
    d* = 0 = c_clear and the derived step is 0; where the walk stopped inside
    the eps band, it is negative. Both fall through to the EPS_S floor, and the
    walk then crawls 0.01 s at a time through blocked intervals seconds long.
    MEASURED before this was rewritten: 1547 floor steps and 8.0 s per dive on
    n4_s4_mr0, i.e. 2 nodes expanded in 74 s -- p1_g9 B3's lesson ("the slow
    thing is the CHECKER") reproduced one session later in the code written to
    avoid it.

    What replaces it keeps the same guarantee and drops the walk: evaluate the
    whole start grid at once (`_blocked_over`), then bisect the first feasible
    interval down to EPS_S. Lemma 7 survives with a different job -- it is what
    says the EPS_S refinement is below the resolution of the collision test
    itself, since V_POINT * EPS_S = 0.0011 m, a fifth of eps = 0.005 m.

    The conservatism is now the grid step `ds`: a feasible window narrower than
    `ds` can be stepped over, so a returned start is never EARLIER than the
    truth. Same direction as every other approximation in this model stack --
    the reported makespan is an upper bound.
    """
    p_from = tr.end_pose()
    T = leg_duration(p_from, q)
    W = T + dur
    horizon = max(t_min, other.end_time())

    nu = int(np.clip(W * V_REL / eps + 2.0, 2, NU_MAX))
    u = np.linspace(0.0, W, nu)
    own_lin, own_rot = leg_pose(u, p_from, q)

    grid = {round(t_min, 9), round(horizon, 9)}
    if not earliest_only:
        for ts, _, _, Tl in other.legs:
            for t in (ts, ts + Tl):
                if t_min < t <= horizon:
                    grid.add(round(float(t), 9))
        if horizon > t_min:
            n = int(min(np.ceil((horizon - t_min) / ds), n_probe * 40))
            grid |= {round(t_min + (horizon - t_min) * j / n, 9)
                     for j in range(1, n)}
    s_vals = np.array(sorted(grid), float)

    blk = _blocked_over(s_vals, u, own_lin, own_rot, tr.g, other, c_clear, eps,
                        bud)
    free = np.flatnonzero(~blk)
    if not len(free):
        return []

    # first index of each maximal free run: those are the interesting starts.
    starts = [int(free[0])] + [int(j) for a, j in zip(free, free[1:])
                               if j != a + 1]
    out = []
    for i in starts:
        s = float(s_vals[i])
        if i > 0:                       # refine leftwards to EPS_S (Lemma 7)
            lo, hi = float(s_vals[i - 1]), s
            while hi - lo > eps_s:
                mid = 0.5 * (lo + hi)
                if _blocked_over(np.array([mid]), u, own_lin, own_rot, tr.g,
                                 other, c_clear, eps, bud)[0]:
                    lo = mid
                else:
                    hi = mid
            s = hi
        s = max(s, t_min)
        cand = tr.copy()
        cand.append(s, q)
        out.append((round(s, 9), cand))
        if earliest_only:
            break

    if len(out) > k:
        if bud:
            bud.k_start_hit += 1
        # earliest, plus an even spread of the rest: Lemma 6 says a LATER start
        # can be the good one, so "keep the k earliest" would trim exactly the
        # interesting tail.
        idx = sorted({0} | {1 + round(j * (len(out) - 2) / (k - 1))
                            for j in range(k - 1)})
        out = [out[i] for i in idx]
    return out


# ===========================================================================
# A2.3 -- the dive: a feasible completion from ANY node
# ===========================================================================
def _continuation(inst, pc, g, A, p_idx):
    """The gantry's own DP-optimal remaining stops from (task set A, grid pose).

    Same walk as sched._reconstruct, but starting from an arbitrary GRID pose
    rather than from p0 and rather than from a `keep` index -- sched.py is
    frozen (A0), so it is written here instead of parameterised there.
    Uncoupled: it ignores the other gantry entirely, which is what makes it a
    starting point for the repair rather than an answer.
    """
    dp = pc.dp
    stops, R, trow = [], A, pc.trow(p_idx)
    while R:
        wr = dp.w[R]
        cand = trow + wr
        r2 = int(np.argmin(cand))
        if not np.isfinite(cand[r2]):
            return None
        bestU, bestc = None, np.inf
        U = R
        while U:
            c = dp.dur[U, r2] + dp.h[R ^ U, r2]
            if c < bestc:
                bestU, bestc = U, c
            U = (U - 1) & R
        if bestU is None:
            return None
        p = int(dp.keep[r2])
        d, assign = sched.stop_duration(inst, g, bestU, p)
        stops.append(dict(pose=p, dur=d, tasks=bestU, assign=assign))
        R ^= bestU
        trow = dp.T[r2]
    return stops


class PoseCost:
    """Cost-to-go and traverse rows for a gantry standing at ANY grid pose.

    🔺 CONTRADICTION 2 with p1_g9's implementation of its own contradiction 2.
    G9 wrote that the action space must become `keep_g U safe_g`; the code
    intersected instead (`safe = [p for p in safe_poses(...) if p in keep_idx]`)
    because the DP lower bound is indexed by `keep`. The consequence is not a
    performance detail: `solve_gantry` keeps only poses where some task subset
    is feasible, so a gantry with NO tasks keeps exactly its start pose and can
    never move at all. It then blocks the other gantry forever and the instance
    is reported INFEASIBLE. Measured on Q3, the pathological case p1_g9 A3-K1b
    specified and p1_g9 never built: coupled makespan came back `inf`.

    The fix is to stop indexing by `keep`. For a gantry standing at an
    arbitrary grid pose, its exact uncoupled cost-to-go for every task subset is

        h[A] = min over kept poses k of ( T(pose, k) + w[A][k] )

    which is the same formula `solve_gantry` uses internally, evaluated one row
    at a time. Cached per pose, it is one vector op over |keep|.
    """

    def __init__(self, inst, dp, g):
        self.inst, self.dp, self.g = inst, dp, g
        self.P = inst.poses[g]
        self.kp = self.P[dp.keep]
        self._trow, self._hcol = {}, {}
        self.keep_idx = {int(p): i for i, p in enumerate(dp.keep)}

    def trow(self, p_idx):
        r = self._trow.get(p_idx)
        if r is None:
            j = self.keep_idx.get(p_idx)
            r = (self.dp.T[j] if j is not None else
                 sched.traverse_time(self.P[p_idx, 0] - self.kp[:, 0],
                                     self.P[p_idx, 1] - self.kp[:, 1],
                                     self.inst.t_fold))
            self._trow[p_idx] = r
        return r

    def hcol(self, p_idx):
        h = self._hcol.get(p_idx)
        if h is None:
            j = self.keep_idx.get(p_idx)
            if j is not None:
                h = self.dp.h[:, j]
            else:
                h = np.min(self.trow(p_idx)[None, :] + self.dp.w, axis=1)
                # `solve_gantry` leaves w[empty set] at inf -- it only ever
                # fills w for non-empty task sets, and reads the empty case off
                # h[0] = 0 instead. Reconstructing h from w therefore has to
                # restore it, and forgetting to made EVERY evasive successor
                # score inf: measured on Q3, where the evade branch probed all
                # three targets, found all three feasible, and pushed none.
                h = h.copy()
                h[0] = 0.0
            self._hcol[p_idx] = h
        return h


def _evade_order(inst, g, cur, safe_g):
    """Poses to try when a gantry has to get out of the way, best first.

    🔺 CONTRADICTION 3, and W2 is what found it -- which is the whole reason
    A3-K1 demanded W2 exist. G9 and the first G10 draft both restricted evasive
    targets to the UNIVERSALLY SAFE set (Lemma 1: |rot| < 31.7 deg blocks
    nothing, whatever the other gantry does). Sufficient as a DESTINATION,
    and that is the trap: the ROUTE to it can be blocked even when a route to
    an unsafe pose is not.

    Measured on Q3. Gantry 2 sits at (0.8, +90 deg) in the way. Rotating to the
    only safe pose, rot = 0, sweeps DOWN through +73...+45 deg and collides with
    gantry 1 at t = 1.553 s. Rotating to rot = -135 deg instead goes the other
    way round, up through +180 deg, and never collides -- so the pair can move
    CONCURRENTLY and the coupled optimum equals the uncoupled one. Restricted to
    safe targets the solver returned 20.5200; the brute-force enumerator found
    11.2600, a schedule the wait-aware gate accepts.

    Order: safe poses first (cheapest traverse first), because they are the ones
    that provably end the deadlock; then every other pose by traverse cost.
    """
    P = inst.poses[g]
    cost = sched.traverse_time(P[:, 0] - cur[0], P[:, 1] - cur[1], inst.t_fold)
    safe_set = set(int(p) for p in safe_g)
    order = np.argsort(cost, kind='stable')
    first = [int(p) for p in order if int(p) in safe_set][:EVADE_CANDIDATES]
    rest = [int(p) for p in order if int(p) not in safe_set]
    return first + rest


def _split_best(pcs, gs, R, r, t):
    """(value, masks) of the split achieving _split_lb -- the admissible bound.

    h_g[R] with R still shared is NOT a lower bound (p1_g9 B3, the first real
    bug of that session): h_g is monotone, so pricing gantry g for the whole
    remainder over-estimates. The min over splits IS the uncoupled optimum from
    this state, which Lemma 3 makes valid.
    """
    a, b = gs
    ha, hb = pcs[a].hcol(r[a]), pcs[b].hcol(r[b])
    best, best_masks = np.inf, None
    sub = R
    while True:
        v1, v2 = ha[sub], hb[R ^ sub]
        if np.isfinite(v1) and np.isfinite(v2):
            v = max(t[a] + float(v1), t[b] + float(v2))
            if v < best:
                best, best_masks = v, {a: sub, b: R ^ sub}
        if sub == 0:
            break
        sub = (sub - 1) & R
    return best, best_masks


def _dive(inst, pcs, gs, safe, t, fin, r, trajs, stops, R, c_clear, eps,
          bud=None):
    """A genuinely feasible completion from this node, or (inf, None).

    Structure copied from sched_coll._repair_ub -- deliberately, because that
    constructor was measured (p1_g9 B6: bracket ratio 1.600 -> 1.126) and A0
    says use it, do not rebuild it. The generalisation is that it starts from an
    arbitrary committed state rather than from t = 0, which _repair_ub cannot
    do; the root call still goes through _repair_ub itself so the measured code
    is on the path.

    Everything it emits is a real traverse or a real dwell. Being feasible, its
    value is a valid UPPER bound however it was found (A2.3 Lemma 8), and when
    it equals the popped node's lower bound the search is over.
    """
    if bud:
        bud.dives += 1
    _, masks = _split_best(pcs, gs, R, r, t)
    if masks is None:
        return np.inf, None
    todo = {}
    for g in gs:
        cont = _continuation(inst, pcs[g], g, masks[g], r[g])
        if cont is None:
            return np.inf, None
        todo[g] = cont
    tt = dict(t)
    ff = dict(fin)
    trj = {g: trajs[g].copy() for g in gs}
    out = {g: list(stops[g]) for g in gs}
    aside = {g: False for g in gs}

    def _try(g, h):
        st = todo[g][0]
        q = tuple(inst.poses[g][st['pose']])
        cands = feasible_starts(trj[g], q, st['dur'], tt[g], trj[h], c_clear,
                                eps, bud=bud, earliest_only=True)
        if not cands:
            return False
        s, newtr = cands[0]
        todo[g].pop(0)
        arrive = s + leg_duration(trj[g].end_pose(), q)
        trj[g] = newtr
        tt[g] = ff[g] = arrive + st['dur']
        out[g].append(dict(st, depart=s, start=arrive))
        return True

    while any(todo[g] for g in gs):
        placed = False
        for g in sorted((k for k in gs if todo[k]), key=lambda k: (tt[k], k)):
            h = [k for k in gs if k != g][0]
            if _try(g, h):
                placed = True
                break
        if placed:
            continue
        for g in gs:                     # both blocked: step aside, once each
            h = [k for k in gs if k != g][0]
            if aside[g] or not safe.get(g):
                continue
            aside[g] = True
            cur = trj[g].end_pose()
            order = sorted(safe[g], key=lambda p: leg_duration(
                cur, tuple(inst.poses[g][p])))[:EVADE_CANDIDATES]
            moved = False
            for p_safe in order:
                qg = tuple(inst.poses[g][p_safe])
                if qg == cur:
                    break
                cands = feasible_starts(trj[g], qg, 0.0, tt[g], trj[h],
                                        c_clear, eps, bud=bud,
                                        earliest_only=True)
                if cands:
                    s_g, tr_g = cands[0]
                    trj[g] = tr_g
                    arrive = s_g + leg_duration(cur, qg)
                    tt[g] = arrive
                    out[g].append(dict(pose=p_safe, depart=s_g, start=arrive,
                                       dur=0.0, tasks=0, assign={}))
                    moved = True
                    break
            if moved:
                placed = True
                break
        if not placed:
            return np.inf, None
    return max(ff.values()), out


# ===========================================================================
# The solver
# ===========================================================================
@dataclass
class CoupledSolution:
    makespan: float
    lb: float
    proved: bool
    stops: dict
    finish: dict
    wall_s: float = 0.0
    n_nodes: int = 0
    route: str = ''
    n_evade: int = 0
    budget: dict = field(default_factory=dict)

    @property
    def bracket(self):
        return self.makespan / self.lb if self.lb > 0 else float('nan')

    @property
    def delta(self):
        return self.makespan - self.lb


def finish_of(stops, gs):
    """Per-gantry completion = end of the LAST DWELL, not of the last motion.

    p1_g7 A2-K2 defines the makespan as the end of the last task's dwell
    window. Evasive moves (p1_g9 contradiction 2) carry no task and no dwell, so
    a gantry that steps aside after finishing must not have that traverse
    counted as work -- reading the last stop blindly would inflate the makespan
    by a whole leg on exactly the instances where collision binds.
    """
    return {g: max((s['start'] + s['dur'] for s in stops.get(g, [])
                    if s['tasks']), default=0.0) for g in gs}


def _stops_with_depart(inst, g, stops):
    """G7/G8 stops carry only `start`; read the departure as "as late as
    possible", which is what those schedules meant."""
    out, cur = [], inst.p0[g]
    for st in stops:
        T = leg_duration(tuple(inst.poses[g][cur]),
                         tuple(inst.poses[g][st['pose']]))
        out.append(dict(st, depart=st['start'] - T))
        cur = st['pose']
    return out


def solve_coupled2(inst, c_clear=C_CLEAR, eps=EPS_CERT, budget=NODE_BUDGET,
                   time_budget=120.0, force_bnb=False, ub_seed=None,
                   action_budget=ACTION_BUDGET, no_dive=False,
                   n_multi=N_MULTI):
    """Minimum makespan under A2.4, exact w.r.t. the A2.2 candidate start set.

    Route 1 (Lemma 4): the uncoupled optimum's own schedule, if it is already
    collision-free, IS the coupled optimum -- it touches a valid lower bound.
    Reported as a route so K5.3 can say how often the search was never run.

    Route 2: best-first branch & bound over joint states. LB from _split_best
    (admissible by Lemma 3); UB from a dive at every popped node (Lemma 8).
    `no_dive=True` turns the dive off and is the ONLY configuration that tests
    search completeness -- with the dive on and collisions off, the root dive
    answers immediately and the expansion machinery never runs, which would be
    p1_g8 B1's never-firing gate wearing a disguise.
    """
    t0 = time.time()
    bud = Budget()
    gs = inst.gantries
    dps = {g: sched.solve_gantry(inst, g) for g in gs}
    sol0 = sched.solve_exact(inst)
    lb = sol0.makespan

    if len(gs) < 2:
        st = {g: _stops_with_depart(inst, g, sol0.stops[g]) for g in gs}
        return CoupledSolution(sol0.makespan, lb, True, st, sol0.finish,
                               time.time() - t0, 0, 'single',
                               budget=bud.as_dict())

    if not force_bnb and schedule_conflict(inst, sol0.stops, c_clear,
                                           eps) is None:
        st = {g: _stops_with_depart(inst, g, sol0.stops[g]) for g in gs}
        return CoupledSolution(sol0.makespan, lb, True, st, sol0.finish,
                               time.time() - t0, 0, 'lemma4',
                               budget=bud.as_dict())

    a, b = gs
    pcs = {g: PoseCost(inst, dps[g], g) for g in gs}
    # every universally safe pose, not just the ones the DP kept -- see the
    # PoseCost docstring for what the intersection cost G9 on Q3.
    safe = {g: [int(p) for p in safe_poses(inst.poses[g], c_clear)]
            for g in gs}
    full = (1 << inst.n) - 1
    t_root = {g: 0.0 for g in gs}
    r_root = {g: int(inst.p0[g]) for g in gs}
    tr_root = {g: Traj(g, tuple(inst.poses[g][inst.p0[g]])) for g in gs}

    # incumbent: the measured G9 constructor first (A0 says use it), then the
    # general dive, then the always-feasible serialisation of Lemma 1.
    best_m, best_stops = _repair_ub(inst, sol0, c_clear, eps, safe)
    if best_stops is not None:
        best_stops = {g: _stops_with_depart(inst, g, best_stops[g])
                      for g in gs}
    if not no_dive:
        m2, s2 = _dive(inst, pcs, gs, safe, t_root, {g: 0.0 for g in gs},
                       r_root, tr_root, {g: [] for g in gs}, full, c_clear,
                       eps, bud)
        if m2 < best_m:
            best_m, best_stops = m2, s2
    if not np.isfinite(best_m):
        m3, s3 = _serial_ub(inst, sol0, safe, c_clear, eps)
        if np.isfinite(m3):
            best_m = m3
            best_stops = {g: _stops_with_depart(inst, g, s3[g]) for g in gs}
    if ub_seed is not None:                 # W2b only: force the search to work
        best_m, best_stops = float(ub_seed), None

    route = 'bnb'
    if best_stops is not None and best_m <= lb + 1e-9:
        return CoupledSolution(best_m, lb, True, best_stops,
                               finish_of(best_stops, gs), time.time() - t0, 0,
                               'dive-lb', budget=bud.as_dict())

    n_nodes, n_evade, exhausted, dive_proof = 0, 0, False, False
    counter = itertools.count()
    heap = [(lb, next(counter), t_root, dict(t_root), r_root, tr_root,
             {g: [] for g in gs}, full)]

    while heap:
        if n_nodes > budget or time.time() - t0 > time_budget:
            exhausted = True
            break
        node_lb, _, t, fin, r, trajs, stops, R = heapq.heappop(heap)
        if node_lb >= best_m - 1e-9:
            continue
        n_nodes += 1
        if R == 0:
            m = max(fin.values())
            if m < best_m - 1e-9:
                best_m, best_stops = m, {g: list(stops[g]) for g in gs}
            continue

        # A2.3 Lemma 8: a feasible completion from HERE. Popped in LB order, so
        # node_lb is the global lower bound; a dive that touches it is a proof.
        if not no_dive:
            dm, ds = _dive(inst, pcs, gs, safe, t, fin, r, trajs, stops, R,
                           c_clear, eps, bud)
            if dm < best_m - 1e-9:
                best_m, best_stops = dm, ds
            if best_m <= node_lb + 1e-9:
                bud.dive_proved += 1
                dive_proof = True
                break

        g = min(gs, key=lambda k: (t[k], k))
        h = b if g == a else a
        dp, other = dps[g], trajs[h]

        # Successor bound, vectorised over every candidate pose, computed
        # BEFORE any start-time search: the certificate is the expensive thing
        # (p1_g9 B3, and p1_g8 B4 before it), so it may only run on actions
        # that already survived pruning.
        Tg = pcs[g].trow(r[g])
        dp_h_g, hcol_h = dps[g].h, pcs[h].hcol(r[h])
        actions = []
        U = R
        while U:
            d = dp.dur[U, :]
            end = t[g] + Tg + d
            R2 = R ^ U
            split = np.full(end.shape, np.inf)
            A = R2
            while True:
                v2 = hcol_h[R2 ^ A]
                if np.isfinite(v2):
                    np.minimum(split, np.maximum(end + dp_h_g[A, :],
                                                 t[h] + float(v2)), out=split)
                if A == 0:
                    break
                A = (A - 1) & R2
            nlb_vec = np.maximum(np.maximum(split, end), fin[h])
            okk = np.flatnonzero(np.isfinite(d) & (nlb_vec < best_m - 1e-9))
            for r2 in okk:
                actions.append((float(nlb_vec[r2]), int(r2), U, float(d[r2])))
            U = (U - 1) & R
        actions.sort(key=lambda x: x[0])

        produced = False
        for k, (_, r2, U, d) in enumerate(actions):
            if k >= action_budget or time.time() - t0 > time_budget:
                exhausted = True
                bud.action_hit += 1
                break
            p2 = int(dp.keep[r2])
            q = tuple(inst.poses[g][p2])
            multi = k < n_multi
            if not multi:
                bud.multi_skipped += 1
            for s, newtr in feasible_starts(trajs[g], q, d, t[g], other,
                                            c_clear, eps, bud=bud,
                                            earliest_only=not multi):
                arrive = s + leg_duration(trajs[g].end_pose(), q)
                end = arrive + d
                nt, nf, nr = dict(t), dict(fin), dict(r)
                nt[g] = nf[g] = end
                nr[g] = p2
                ntr, nst = dict(trajs), dict(stops)
                ntr[g] = newtr
                nst[g] = stops[g] + [dict(
                    pose=p2, depart=s, start=arrive, dur=d, tasks=U,
                    assign=sched.stop_duration(inst, g, U, p2)[1])]
                R2 = R ^ U
                nlb = max(max(nf.values()),
                          _split_best(pcs, gs, R2, nr, nt)[0])
                if nlb < best_m - 1e-9:
                    heapq.heappush(heap, (nlb, next(counter), nt, nf, nr, ntr,
                                          nst, R2))
                    produced = True

        # Every action blocked and the other gantry is parked in the way: let
        # it stand aside (p1_g9 contradiction 2 -- evasive moves are part of
        # the model, not an implementation detail). Skipped when h is already
        # universally safe, since then h is not what blocks g.
        h_safe = int(dp_pose_index(inst, h, trajs[h].end_pose())) in safe[h]
        if not produced and not h_safe:
            pushed = 0
            for p_safe in _evade_order(inst, h, trajs[h].end_pose(), safe[h]):
                if pushed >= EVADE_PUSH or bud.evade_tried >= EVADE_ATTEMPTS:
                    break
                q = tuple(inst.poses[h][p_safe])
                if q == trajs[h].end_pose():
                    continue
                bud.evade_tried += 1
                cands = feasible_starts(trajs[h], q, 0.0, t[h], trajs[g],
                                        c_clear, eps, bud=bud,
                                        earliest_only=True)
                if not cands:
                    continue
                pushed += 1
                s, newtr = cands[0]
                nt, nr = dict(t), dict(r)
                nt[h] = s + leg_duration(trajs[h].end_pose(), q)
                nr[h] = int(p_safe)
                ntr = dict(trajs)
                ntr[h] = newtr
                nst = dict(stops)
                nst[h] = stops[h] + [dict(pose=p_safe, depart=s, start=nt[h],
                                          dur=0.0, tasks=0, assign={})]
                nlb = max(max(fin.values()),
                          _split_best(pcs, gs, R, nr, nt)[0])
                if nlb < best_m - 1e-9:
                    n_evade += 1
                    heapq.heappush(heap, (nlb, next(counter), nt, dict(fin),
                                          nr, ntr, nst, R))

    # `exhausted` only means a budget was touched; it does NOT mean the answer
    # is unproved. Lemma 8 (the dive touched the popped node's LB) and Lemma 3
    # (the incumbent touched the uncoupled optimum) are both proofs of
    # optimality in their own right, and p1_g9 B6 already had to make that
    # argument by hand in the eval script because the solver's flag was
    # stricter than the mathematics. It is fixed here instead.
    proved = dive_proof or (not exhausted) or best_m <= lb + 1e-9
    return CoupledSolution(best_m, lb, proved, best_stops or {},
                           finish_of(best_stops or {}, gs), time.time() - t0,
                           n_nodes, route, n_evade, bud.as_dict())
