"""G12: the ARM-AWARE solver. docs/p1_g12_armsolve.md A2.

WHAT A2.1 LOCKED, AND WHY IT COULD NOT BE DONE
==============================================
A2.1 locked "no second solver": ARM_BLOCK was to enter by substituting four
names in sched_coupled's namespace, leaving the B&B loop frozen and inheriting
its whole bar (W2, W2b(i), P0-P4, the 17-mutation gate).

🔺 CONTRADICTION 1, and it is about a SEAM THAT DOES NOT EXIST, not about
taste. `feasible_starts(tr, q, dur, t_min, other, ...)` is the only place the
loop tests an action, and it is passed the target POSE and the DURATION -- it
is never passed the action's TASK SET `U`. ARM_BLOCK is a function of
(pose, task set) on BOTH sides (A2.4 of p1_g11, the one structural difference
from BLOCK), so the substitution A2.1 locked cannot express it. Three further
seams were read before giving up on it, and each fails for a reason worth
recording so G13 does not re-derive them:

  sched.stop_duration(inst, g, U, p2)  is called with U -- but AFTER
      feasible_starts has already returned, so it cannot veto a start.
  _replay / _dive / repair_schedule    DO carry st['tasks'] and st['assign'],
      so those paths ARE patchable, and they are patched below.
  the `R == 0` incumbent update (sched_coupled line 671) takes a completed
      schedule with no predicate call at all, so no substitution can reach it.

WHAT REPLACES THE GUARANTEE
---------------------------
The loop is forked, and M0 is what carries G10's bar across the fork: with the
arm test disabled (c_arm = -inf) `solve_armfull` must reproduce `solve_coupled2`
EXACTLY -- makespan, route, proved, node count -- on all 40 S1 instances and all
31 W2 fuel instances. Equivalence is checked, not asserted. Every other piece
(`_split_best`, `PoseCost`, `_continuation`, `_evade_order`, `feasible_starts`,
`CoupledSolution`) is IMPORTED from the frozen module, not copied, so the fork
is the ~120 lines of the loop and nothing else.

WHERE ARM_BLOCK IS MOUNTED (A2.3)
=================================
Not as a whole-schedule gate (45 ms, p1_g11 B4 -- thousands of those per node
does not fit 120 s), and not as a precomputed pair table (it has no analogue).
It is mounted on the ACTION, incrementally, decomposed into three cases of
which Lemma B deletes one outright:

  (i)   both gantries hanging      -> Lemma B: ZERO evaluations.
        L2 measured HANG_BLOCK(c) subset BLOCK(c + delta), delta <= 0.004 m,
        over all 2376^2 pose pairs. The solver therefore runs the STRUCTURE
        predicate at c_clear = c_arm + delta and the arm code never looks at
        traverse x traverse at all.
  (ii)  both dwelling, windows overlapping -> ONE arm_block() per window pair,
        both bodies static, no certificate walk.
  (iii) one dwelling (static), one hanging (moving or parked) -> a Lipschitz
        certificate walk over that piece only, at V_ARM rather than V_REL_ARM
        because only one side moves.

Contradiction 5 of p1_g10 applies unchanged and is why every check runs to the
end of the OPPONENT'S committed future, not just the action's own window.
"""

from __future__ import annotations

import heapq
import itertools
import time

import numpy as np

from reachability_gng import sched
from reachability_gng import sched_arm as sa
from reachability_gng import sched_coll as sc
from reachability_gng import sched_coupled as sk
from reachability_gng.irm_sweep import polyline_min_dist

# Lemma B's delta. NOT a constant of the world -- it is L2's measured envelope,
# and A2.2 forbids using Lemma B at any c_arm whose delta was not re-measured
# this session (verify_sched_armfull.py m2a does exactly that).
DELTA_LEMMA_B = 0.004

# Only one body moves in case (iii), so the closing speed is V_ARM, not
# V_REL_ARM: the certificate step is twice as long as p1_g11's full gate takes.
V_ARM = sa.V_ARM

# Floor on the certificate step, so the eps-aware walk terminates instead of
# converging geometrically onto the band edge. STEP_MIN * V_REL_ARM = 2.2 mm,
# the same order as the frozen stack's own EPS_S floor (p1_g10 A2.2:
# V_POINT * EPS_S = 1.1 mm, "a fifth of eps"). Conservative in one direction
# only: it may declare a block up to 2.2 mm early, never late.
STEP_MIN = 0.01


class ArmView:
    """A gantry's arms over time, built from a Traj that already exists.

    sched_arm.ArmTraj rebuilds the trajectory by replaying stop legs; inside the
    search the Traj is already committed and replaying it per action is the kind
    of quiet quadratic that p1_g9 B3 warned about. Same semantics as ArmTraj --
    A2.1 exactly: canonical inside one's own task-carrying dwell, hanging
    everywhere else -- with the trajectory handed in.
    """

    __slots__ = ('geom', 'g', 'tr', 'win', 'hb', '_cache')

    def __init__(self, geom, g, tr, win):
        self.geom, self.g, self.tr, self.win = geom, g, tr, tuple(win)
        self.hb = geom.hang_base
        self._cache = {}

    def end_time(self):
        return max([self.tr.end_time()] + [w[1] for w in self.win] + [0.0])

    def dwell_at(self, t):
        # Half-open on the right, and that is load-bearing rather than tidy:
        # see the fix note in sched_arm.ArmTraj.polys. A closed right edge makes
        # the certificate walk read the just-ended dwell's EXTENDED arm at the
        # exact instant it restarts, and step over the violation.
        for w in self.win:
            if w[0] - 1e-12 <= t < w[1]:
                return w
        return None

    def polys(self, t):
        w = self.dwell_at(t)
        if w is not None:
            return self.geom.configs(self.g, w[2], w[3], w[4])
        lin, rot = self.tr.pose_at(np.asarray(t, float))
        out = []
        for k in (0, 1):
            R, o = sa.base_pose(self.geom.arms[self.g][k], float(lin),
                                float(rot))
            out.append(np.concatenate([o[None], (R @ self.hb.T).T + o])[None])
        return out

    def pts(self, t):
        """(m, 4, 3) -- BOTH arms' configurations stacked into one array.

        ARM_BLOCK is `min over a in arms(g1), b in arms(g2)` (A2.3), so which
        arm a configuration belongs to never affects the value and the four arm
        pairs may be evaluated as one broadcast. Measured: this is the
        difference between 1170 us and 190 us per evaluation, because
        polyline_min_dist costs 129.71 us at N = 1 and 1.86 us at N = 10 000
        (p1_g11 B4) -- the cost is numpy CALL overhead, not arithmetic.
        """
        w = self.dwell_at(t)
        if w is not None:
            key = (w[2], w[3])
            got = self._cache.get(key)
            if got is None:
                got = self._cache[key] = np.concatenate(
                    self.geom.configs(self.g, w[2], w[3], w[4]))
            return got
        return np.concatenate(self.polys(t))

    def moving(self, t):
        for ts, _, _, T in self.tr.legs:
            if ts - 1e-12 <= t <= ts + T + 1e-12:
                return True
        return False


def _wins_of(stops):
    """(t0, t1, pose, tasks, assign) for every stop that actually holds tasks.

    A task-free stop is an evasive move (p1_g9 contradiction 2): the gantry
    stands there with both arms hanging, so it is NOT a window -- it is covered
    by Lemma B like any other hanging interval.
    """
    return [(s['start'], s['start'] + s['dur'], s['pose'], s['tasks'],
             s.get('assign')) for s in stops if s['tasks']]


def _pair_dist(A, B, t):
    """Min distance between the two gantries' four arm pairs at time t.

    ONE broadcast call, not four -- see ArmView.pts. NaN is `task unreachable
    by that arm at that pose`, which the scheduling model already forbids, so
    it is mapped to +inf rather than propagated (same convention as
    sched_arm.pair_min).
    """
    X, Y = A.pts(t), B.pts(t)
    d = polyline_min_dist(X[:, None], Y[None, :])
    return float(np.where(np.isnan(d), np.inf, d).min())


def arm_conflict(A, B, t_lo, t_hi, c_arm, eps=sc.EPS_CERT, memo=None,
                 lemma_b=True):
    """Earliest ARM_BLOCK violation in [t_lo, t_hi], or None. A2.3.

    The decomposition is in the loop over pieces, and its correctness rests on
    one fact about A2.1: between two consecutive breakpoints, EACH gantry's arm
    STATE (hanging vs which canonical set) is constant -- only the gantry POSE
    varies, and only while traversing. So per piece:

      both hanging          -> Lemma B, skipped entirely
      neither moving        -> one evaluation at the piece start
      otherwise             -> Lipschitz walk at V_ARM (one mover) or
                               V_REL_ARM (both), which is the p1_g9 B2
                               certificate with the arm radius in it

    Breakpoints include every dwell edge of both gantries, because a
    configuration CHANGE is instantaneous in this model (T_fold = 0,
    p1_g7 A4.3) and a Lipschitz step must never stride across one.
    """
    marks = sorted({t_lo, t_hi}
                   | {x for w in A.win + B.win for x in w[:2]
                      if t_lo < x < t_hi}
                   | {x for tr in (A.tr, B.tr) for ts, _, _, T in tr.legs
                      for x in (ts, ts + T) if t_lo < x < t_hi})
    for lo, hi in zip(marks, marks[1:] + [t_hi]):
        if hi < lo - 1e-12:
            continue
        mid = 0.5 * (lo + hi)
        wa, wb = A.dwell_at(mid), B.dwell_at(mid)
        if wa is None and wb is None and lemma_b:
            continue                      # Lemma B -- both hanging, delegated
        ma, mb = A.moving(mid), B.moving(mid)
        if not (ma or mb):                # both static: one evaluation
            if wa is not None and wb is not None and memo is not None:
                key = (wa[2], wa[3], wb[2], wb[3])
                d = memo.get(key)
                if d is None:
                    d = memo[key] = _pair_dist(A, B, lo)
            else:
                d = _pair_dist(A, B, lo)
            if d <= c_arm + eps:
                return lo
            continue
        # 🔺 The V_ARM refinement (case (iii) has one mover, so the closing
        # speed is HALF V_REL_ARM and the step could be twice as long) is
        # CORRECT -- measured empirically at 0.111125 m/s against V_ARM =
        # 0.111200 on 200 random legs -- and it is GIVEN UP anyway. Two walks
        # with different step sizes sample different points of the (c, c + eps]
        # guard band and therefore return different verdicts on schedules where
        # the true clearance sits inside it. Both are sound about true blocks
        # and neither can serve as the other's oracle, which costs more than
        # the factor of two is worth: M2 exists to adjudicate the fast path
        # against an independent slow one, and a gate that cannot fire cleanly
        # is p1_g8 B1's never-firing gate in yet another costume.
        v = sa.V_REL_ARM
        t = lo
        while t <= hi:
            d = _pair_dist(A, B, t)
            if d <= c_arm + eps:
                return t
            # 🔴 The step is (d - c - EPS), not (d - c). Stepping by (d - c)
            # only certifies that no TRUE block (d <= c) is skipped; it can
            # still stride past a point inside the eps band, so WHICH points
            # of the band get seen depends on the step size -- and the step
            # size depends on how many bodies move. Measured on n6_s0_mr0:
            # this walk (V_ARM, one mover) stepped to d = 0.055194 and stopped,
            # while sched_arm's (V_REL_ARM, half the step) landed on
            # d = 0.054960 and called it a block. Both are sound about true
            # blocks; they simply disagreed about the guard band, which made
            # an oracle comparison impossible. Subtracting eps makes detection
            # STEP-INDEPENDENT: the walk now sees every point with
            # d <= c + eps, so any two step sizes give the same verdict.
            step = (d - c_arm - eps) / v
            if step < STEP_MIN:
                return t          # within STEP_MIN * v = 1.1 mm of the band
            t += step
        if _pair_dist(A, B, max(lo, hi - 1e-9)) <= c_arm + eps:
            # hi belongs to the NEXT piece's state (windows are half-open), so
            # the endpoint is evaluated as the limit from the LEFT. Reading it
            # at hi itself is the same stale-configuration error the dwell
            # lookup had -- measured 0.0552 in-piece vs 0.4267 at hi.
            return hi
    return None


def schedule_arm_conflict(inst, geom, stops, c_arm, eps=sc.EPS_CERT,
                          lemma_b=True):
    """Whole-schedule arm check through the FAST path. M2 adues it with the
    slow independent walk in sched_arm.arm_schedule_conflict."""
    gs = sorted(stops)
    if len(gs) < 2:
        return None
    views = {}
    for g in gs:
        tr = sc.Traj(g, tuple(inst.poses[g][inst.p0[g]]))
        cur = inst.p0[g]
        for st in stops[g]:
            q = tuple(inst.poses[g][st['pose']])
            dep = st.get('depart', st['start'] - sc.leg_duration(
                tuple(inst.poses[g][cur]), q))
            tr.append(dep, q)
            cur = st['pose']
        views[g] = ArmView(geom, g, tr, _wins_of(stops[g]))
    A, B = views[gs[0]], views[gs[1]]
    return arm_conflict(A, B, 0.0, max(A.end_time(), B.end_time()), c_arm, eps,
                        lemma_b=lemma_b)


class ArmCtx:
    """Everything the arm-aware search needs, and the memo A2.3 locked.

    `c_arm = -inf` is the TRANSPARENCY setting M0 uses: every check returns
    "clear" without evaluating anything, so the solver must then be
    indistinguishable from solve_coupled2.
    """

    def __init__(self, inst, geom, c_arm, eps=sc.EPS_CERT, c_clear=0.0,
                 delta=DELTA_LEMMA_B):
        self.inst, self.geom, self.c_arm, self.eps = inst, geom, c_arm, eps
        # 🔺 CONTRADICTION 2 with A2.2, and it is MEASURED, not preferred.
        # Lemma B is TRUE -- L2 proves HANG_BLOCK(c) subset BLOCK(c + 0.004) on
        # all 2376^2 -- but USING it costs more than not using it, because it
        # only discharges hanging x hanging by raising the STRUCTURE clearance
        # to c_arm + delta, and the structural checker has a cliff there:
        #
        #   c_clear   schedule_conflict   feasible_starts   may_block pass
        #    0.000          0.14 ms           14.33 ms          14.51 %
        #    0.004          0.13 ms           14.49 ms          14.63 %
        #    0.020         70.55 ms          116.62 ms          16.80 %
        #    0.054         95.94 ms          235.61 ms          22.02 %
        #
        # 540x between 0.004 and 0.020 while the prefilter barely widens: the
        # early-out in the frozen certificate walk stops firing. Per action:
        #    (A) Lemma B, struct @ 0.054 : 235.6 + 4.9  = 240.5 ms
        #    (B) no Lemma B, struct @ 0.0:  14.3 + 25.9 =  40.2 ms   <- 6x
        # So Lemma B is kept as a THEOREM and dropped as a MECHANISM: it is
        # applied only when the caller independently wants that clearance.
        # Bonus, and it is the reason this is not merely faster: Delta_struct is
        # then measured at c_clear = 0, the exact p1_g10/p1_g11 basis, so
        # Delta_arm subtracts two numbers that differ ONLY by the arms.
        self.lemma_b = bool(c_clear >= c_arm + delta) if np.isfinite(c_arm) \
            else True
        self.memo = {}
        self.calls = 0
        self.t_arm = 0.0

    @property
    def off(self):
        return not np.isfinite(self.c_arm)

    def view(self, g, tr, stops):
        return ArmView(self.geom, g, tr, _wins_of(stops))

    def ok(self, g, tr_g, stops_g, h, tr_h, stops_h, t_lo):
        """Is the pair arm-clear from t_lo to the end of BOTH futures?

        t_lo is the departure of the action being tested: everything before it
        was checked when the earlier actions were committed, and the opponent's
        whole committed tail is covered here -- p1_g10 contradiction 5, which is
        the same reason the structural check was widened.
        """
        if self.off:
            return True
        self.calls += 1
        t0 = time.perf_counter()
        A = self.view(g, tr_g, stops_g)
        B = self.view(h, tr_h, stops_h)
        r = arm_conflict(A, B, t_lo, max(A.end_time(), B.end_time()),
                         self.c_arm, self.eps, self.memo, self.lemma_b)
        self.t_arm += time.perf_counter() - t0
        return r is None


# ===========================================================================
# A2.4 -- the upper-bound constructors, with the 60-pose cap CUT
# ===========================================================================
def arm_serial_ub(inst, ctx, sol0, c_clear):
    """Serialise: one gantry parks HANGING out of the way, the other works.

    Lemma 1 has no arm analogue (p1_g11 B5 contradiction 4: 0 of 2376 poses are
    universally safe), so the park pose cannot be read off a rule and must be
    SEARCHED. G11's version searched the 60 cheapest and gave up, which made
    "no upper bound exists" indistinguishable from "the cap was hit" -- and it
    reported 12 instances NOT MEASURED for exactly that reason. A2.4 cut the cap
    after costing the exhaustive search at 2376 poses x <= 6 windows x 4 arm
    pairs x 1.86 us ~ 0.11 s.

    Returns (makespan, stops) or (inf, None). Everything emitted is replayed
    through the wait-aware gate by the caller, so p1_g10 B1.2 -- a constructor
    returning a schedule other than the one it checked -- cannot hide here.
    """
    gs = inst.gantries
    best = (np.inf, None)
    for lead, park in (gs, gs[::-1]):
        lead_stops = sk._stops_with_depart(inst, lead, sol0.stops[lead])
        t_lead = max((s['start'] + s['dur'] for s in lead_stops), default=0.0)
        P = inst.poses[park]
        cur = P[inst.p0[park]]
        order = np.argsort(sched.traverse_time(P[:, 0] - cur[0],
                                               P[:, 1] - cur[1]), kind='stable')
        for p in order:                        # A2.4: ALL of them, not 60
            p = int(p)
            T0 = sc.leg_duration(tuple(cur), tuple(P[p]))
            st = ([] if p == int(inst.p0[park]) else
                  [dict(pose=p, depart=0.0, start=T0, dur=0.0, tasks=0,
                        assign={})])
            trial = {lead: lead_stops, park: st}
            if sc.schedule_conflict(inst, trial, c_clear) is not None:
                continue
            if (not ctx.off and schedule_arm_conflict(
                    inst, ctx.geom, trial, ctx.c_arm, ctx.eps,
                    ctx.lemma_b) is not None):
                continue
            rest = sk._stops_with_depart(inst, park, sol0.stops[park])
            t, cur2, moved = max(t_lead, T0), p, []
            for s in rest:
                T = sc.leg_duration(tuple(P[cur2]), tuple(P[s['pose']]))
                moved.append(dict(s, depart=t, start=t + T))
                t += T + s['dur']
                cur2 = s['pose']
            full = {lead: lead_stops, park: st + moved}
            if sc.schedule_conflict(inst, full, c_clear) is not None:
                continue
            if (not ctx.off and schedule_arm_conflict(
                    inst, ctx.geom, full, ctx.c_arm, ctx.eps,
                    ctx.lemma_b) is not None):
                continue
            m = max(t_lead, t)
            if m < best[0]:
                best = (m, full)
            break
    return best


def lemma_c(inst, ctx, stops):
    """A2.4 Lemma C, MEASURED not assumed: does a hanging park pose exist?

    For every task-carrying stop of `stops`, how many of the |P| poses let the
    OTHER gantry hang without ARM_BLOCK. Returns a list of (gantry, stop index,
    surviving pose count, |P|). A zero anywhere is a statement about the CELL:
    a stop with nowhere for the opponent to be.
    """
    out = []
    gs = inst.gantries
    for g in gs:
        h = [k for k in gs if k != g][0]
        P = inst.poses[h]
        H = np.stack(ctx.geom.hang[h])                    # (2, P, 4, 3)
        for j, st in enumerate(stops.get(g, [])):
            if not st['tasks']:
                continue
            cfg = ctx.geom.configs(g, st['pose'], st['tasks'], st.get('assign'))
            d = np.full(len(P), np.inf)
            for X in cfg:
                for k in (0, 1):
                    dd = polyline_min_dist(X[:, None], H[k][None, :])
                    dd = np.where(np.isnan(dd), np.inf, dd)
                    d = np.minimum(d, dd.min(axis=0))
            out.append((g, j, int((d > ctx.c_arm).sum()), len(P)))
    return out


# ===========================================================================
# The solver. Forked loop; everything else imported from the frozen module.
# ===========================================================================
def solve_armfull(inst, c_arm=0.05, delta=DELTA_LEMMA_B, eps=sc.EPS_CERT,
                  budget=sk.NODE_BUDGET, time_budget=120.0,
                  action_budget=sk.ACTION_BUDGET, n_multi=sk.N_MULTI,
                  geom=None, c_clear=None):
    """Minimum makespan under A2.4 AND ARM_BLOCK, exact w.r.t. the A2.2 set.

    Structure is solve_coupled2's, line for line, with exactly four additions,
    each marked ARM below. c_clear defaults to c_arm + delta so that Lemma B
    discharges the hanging x hanging regime through the structural predicate.
    """
    t0 = time.time()
    if c_clear is None:
        c_clear = 0.0            # CONTRADICTION 2, ArmCtx: NOT c_arm + delta
    bud = sk.Budget()
    gs = inst.gantries
    # With the arms off (M0 transparency) no polyline is ever evaluated, so the
    # 0.4 s / 5.5 MB ArmGeom build is skipped -- and skipping it is what lets M0
    # run on the W2 fuel, whose synthetic instances have no real nodes to build
    # arm geometry from at all.
    if geom is None and np.isfinite(c_arm):
        geom = sa.ArmGeom(inst)
    ctx = ArmCtx(inst, geom, c_arm, eps, c_clear, delta)

    dps = {g: sched.solve_gantry(inst, g) for g in gs}
    sol0 = sched.solve_exact(inst)
    lb = sol0.makespan

    if len(gs) < 2:
        st = {g: sk._stops_with_depart(inst, g, sol0.stops[g]) for g in gs}
        return sk.CoupledSolution(sol0.makespan, lb, True, st, sol0.finish,
                                  time.time() - t0, 0, 'single',
                                  budget=bud.as_dict()), ctx

    # ARM 1: the Lemma 4 route must clear BOTH predicates, or it is not a
    # feasible full-model schedule and may not be returned as the optimum.
    st0 = {g: sk._stops_with_depart(inst, g, sol0.stops[g]) for g in gs}
    if (sc.schedule_conflict(inst, sol0.stops, c_clear, eps) is None
            and (ctx.off or schedule_arm_conflict(inst, geom, st0, c_arm, eps,
                                                  ctx.lemma_b) is None)):
        return sk.CoupledSolution(sol0.makespan, lb, True, st0, sol0.finish,
                                  time.time() - t0, 0, 'lemma4',
                                  budget=bud.as_dict()), ctx

    a, b = gs
    pcs = {g: sk.PoseCost(inst, dps[g], g) for g in gs}
    safe = {g: [int(p) for p in sc.safe_poses(inst.poses[g], c_clear)]
            for g in gs}
    full = (1 << inst.n) - 1
    t_root = {g: 0.0 for g in gs}
    r_root = {g: int(inst.p0[g]) for g in gs}
    tr_root = {g: sc.Traj(g, tuple(inst.poses[g][inst.p0[g]])) for g in gs}

    # ARM 2: the incumbent. _repair_ub / _dive are structure-only constructors,
    # so anything they return is re-checked before it is allowed to become
    # best_m -- an arm-infeasible incumbent would make every prune below it
    # UNSOUND, which is the one failure mode of this mount that has no symptom.
    def _accept(m, stops):
        if stops is None or not np.isfinite(m):
            return np.inf, None
        s2 = {g: sk._stops_with_depart(inst, g, stops[g]) for g in gs}
        if not ctx.off and schedule_arm_conflict(inst, geom, s2, c_arm, eps,
                                                 ctx.lemma_b) is not None:
            return np.inf, None
        return m, s2

    best_m, best_stops = _accept(*sc._repair_ub(inst, sol0, c_clear, eps, safe))
    m2, s2 = sk._dive(inst, pcs, gs, safe, t_root, {g: 0.0 for g in gs},
                      r_root, tr_root, {g: [] for g in gs}, full, c_clear,
                      eps, bud)
    m2, s2 = _accept(m2, s2)
    if m2 < best_m:
        best_m, best_stops = m2, s2
    if not np.isfinite(best_m):
        m3, s3 = arm_serial_ub(inst, ctx, sol0, c_clear)
        if np.isfinite(m3):
            best_m, best_stops = m3, {
                g: sk._stops_with_depart(inst, g, s3[g]) for g in gs}

    if best_stops is not None and best_m <= lb + 1e-9:
        return sk.CoupledSolution(best_m, lb, True, best_stops,
                                  sk.finish_of(best_stops, gs),
                                  time.time() - t0, 0, 'dive-lb',
                                  budget=bud.as_dict()), ctx

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
            # ARM 3: the completion path solve_coupled2 takes with no predicate
            # call at all -- the seam that made the A2.1 substitution
            # impossible. Here it is explicit. Nothing arm-infeasible was ever
            # pushed (ARM 4 below), so this is belt and braces, and M1 is what
            # says so out loud.
            m = max(fin.values())
            if m < best_m - 1e-9:
                mm, ss = _accept(m, {g: list(stops[g]) for g in gs})
                if np.isfinite(mm):
                    best_m, best_stops = mm, ss
            continue

        if True:
            dm, ds = sk._dive(inst, pcs, gs, safe, t, fin, r, trajs, stops, R,
                              c_clear, eps, bud)
            dm, ds = _accept(dm, ds)
            if dm < best_m - 1e-9:
                best_m, best_stops = dm, ds
            if best_m <= node_lb + 1e-9:
                bud.dive_proved += 1
                dive_proof = True
                break

        actions = []
        for g in gs:
            h = b if g == a else a
            dp = dps[g]
            Tg = pcs[g].trow(r[g])
            dp_h_g, hcol_h = dps[g].h, pcs[h].hcol(r[h])
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
                                                     t[h] + float(v2)),
                                   out=split)
                    if A == 0:
                        break
                    A = (A - 1) & R2
                nlb_vec = np.maximum(np.maximum(split, end), fin[h])
                okk = np.flatnonzero(np.isfinite(d) & (nlb_vec < best_m - 1e-9))
                for r2 in okk:
                    actions.append((float(nlb_vec[r2]), g, int(r2), U,
                                    float(d[r2])))
                U = (U - 1) & R
        actions.sort(key=lambda x: x[0])

        produced = False
        for k, (_, g, r2, U, d) in enumerate(actions):
            if k >= action_budget or time.time() - t0 > time_budget:
                exhausted = True
                bud.action_hit += 1
                break
            h = b if g == a else a
            dp, other = dps[g], trajs[h]
            p2 = int(dp.keep[r2])
            q = tuple(inst.poses[g][p2])
            multi = k < n_multi
            if not multi:
                bud.multi_skipped += 1
            assign = sched.stop_duration(inst, g, U, p2)[1]
            for s, newtr in sk.feasible_starts(trajs[g], q, d, t[g], other,
                                               c_clear, eps, bud=bud,
                                               earliest_only=not multi):
                arrive = s + sc.leg_duration(trajs[g].end_pose(), q)
                end = arrive + d
                nst_g = stops[g] + [dict(pose=p2, depart=s, start=arrive,
                                         dur=d, tasks=U, assign=assign)]
                # ARM 4: the action test, and the reason the loop had to be
                # forked -- U is known HERE and nowhere the frozen code exposes.
                if not ctx.ok(g, newtr, nst_g, h, other, stops[h], s):
                    continue
                nt, nf, nr = dict(t), dict(fin), dict(r)
                nt[g] = nf[g] = end
                nr[g] = p2
                ntr, nst = dict(trajs), dict(stops)
                ntr[g] = newtr
                nst[g] = nst_g
                R2 = R ^ U
                nlb = max(max(nf.values()),
                          sk._split_best(pcs, gs, R2, nr, nt)[0])
                if nlb < best_m - 1e-9:
                    heapq.heappush(heap, (nlb, next(counter), nt, nf, nr, ntr,
                                          nst, R2))
                    produced = True

        g = min(gs, key=lambda k: (t[k], k))
        h = b if g == a else a
        h_safe = int(sc.dp_pose_index(inst, h, trajs[h].end_pose())) in safe[h]
        if not produced and not h_safe:
            pushed = 0
            for p_safe in sk._evade_order(inst, h, trajs[h].end_pose(),
                                          safe[h]):
                if pushed >= sk.EVADE_PUSH or bud.evade_tried >= sk.EVADE_ATTEMPTS:
                    break
                q = tuple(inst.poses[h][p_safe])
                if q == trajs[h].end_pose():
                    continue
                bud.evade_tried += 1
                cands = sk.feasible_starts(trajs[h], q, 0.0, t[h], trajs[g],
                                           c_clear, eps, bud=bud,
                                           earliest_only=True)
                if not cands:
                    continue
                pushed += 1
                s, newtr = cands[0]
                nst_h = stops[h] + [dict(pose=p_safe, depart=s,
                                         start=s + sc.leg_duration(
                                             trajs[h].end_pose(), q),
                                         dur=0.0, tasks=0, assign={})]
                # ARM 4 again. An evasive stop carries no task, so the mover
                # hangs -- but the OTHER gantry may be mid-dwell, which is
                # case (iii) and not covered by Lemma B.
                if not ctx.ok(h, newtr, nst_h, g, trajs[g], stops[g], s):
                    continue
                nt, nr = dict(t), dict(r)
                nt[h] = nst_h[-1]['start']
                nr[h] = int(p_safe)
                ntr = dict(trajs)
                ntr[h] = newtr
                nst = dict(stops)
                nst[h] = nst_h
                nlb = max(max(fin.values()),
                          sk._split_best(pcs, gs, R, nr, nt)[0])
                if nlb < best_m - 1e-9:
                    n_evade += 1
                    heapq.heappush(heap, (nlb, next(counter), nt, dict(fin),
                                          nr, ntr, nst, R))

    proved = dive_proof or (not exhausted) or best_m <= lb + 1e-9
    return sk.CoupledSolution(best_m, lb, proved, best_stops or {},
                              sk.finish_of(best_stops or {}, gs),
                              time.time() - t0, n_nodes, 'bnb',
                              n_evade, bud.as_dict()), ctx
