"""Layer 3 -- allocation & scheduling, XD [ST-MR-TA]. HEURISTIC half (G8).

The exact half lives in sched.py and is GROUND TRUTH (p1_g7 B1, V0-V4). This
module imports it and never modifies it: measuring a heuristic against a
solver you edited in the same session is the tidiest possible way to fool
yourself (docs/p1_g8_sched2.md A6).

Contents, all locked in p1_g8_sched2.md A2 before this file existed:

  pose_tour     the proposed heuristic. Four stages, derived from p1_g7 B5
                (74-92% of makespan is gantry traverse) and B4 (MR is a POSE
                constraint, not an arm constraint):
                  1 handover poses first  -- scarcest set
                  2 traverse-weighted set cover over gantry poses
                  3 tour: nearest-neighbour from p0, then 2-opt + or-opt
                  4 local repair of task->stop, empty-stop removal, grid-
                    neighbour pose swap
  pose_tour_wide    POST-HOC variant written AFTER the G8 gap table was read.
                    Reported separately and never as `pose-tour`; see B8.
  dur_vec           the stop duration at every pose at once, O(|U| x |P|)
  sched_fixed       baseline: round-robin gantry/arm, own best pose, index order
  sched_greedy      baseline: nearest pose serving >=1 remaining task, do all
  sched_sequential  baseline: one task per stop, nearest-neighbour order
  lb_analytic       cheap valid lower bound (A3-K3)
  lb_subset         exact optimum of a task SUBSET -- a valid lower bound, and
                    the main one above n = 10 (proof in A3-K3)

Every scheduler returns a sched.Solution, so test/verify_sched_exact.py's
validate_schedule() replays it against the model with an independent oracle.
A3-K4: a schedule that fails that replay does not get its number reported.

HARD CONSTRAINT (A2): nothing here may build a 2**n table. `solve_exact` stops
at n = 10 because of `_dur_table`, and the entire point of this module is to run
far above that.

Subcommands
-----------
  run     one instance, one or all schedulers, printed schedule
  check   stop_cost vs sched.stop_duration -- the one shared closed form
  lb      lower bounds for one instance
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from reachability_gng.sched import (DWELL, GANTRY_ARMS, Instance,  # noqa: F401
                                    Solution, gen_real, solve_exact,
                                    stop_duration, traverse_time)

EPS = 1e-3          # traverse floor in the set-cover ratio; see A2 stage 2
TOL = 1e-9


# ---------------------------------------------------------------------------
# Per-gantry view: the oracles plus a cached traverse row
# ---------------------------------------------------------------------------
class GantryView:
    """Read-only view of one gantry of an Instance, with a traverse row cache.

    The full (P, P) traverse matrix is 2376**2 = 5.6 M doubles per gantry. The
    heuristic only ever needs rows out of the poses it actually visits, so rows
    are built on demand and cached. This is the reason nothing here scales with
    |P|**2 either.
    """

    def __init__(self, inst, g):
        self.inst = inst
        self.g = g
        self.poses = inst.poses[g]
        self.reach = inst.reach[g]
        self.zone = inst.zone[g]
        self.hand = inst.hand[g]
        self.p0 = int(inst.p0[g])
        self.dwell = inst.dwell
        self.t_fold = inst.t_fold
        self.is_mr = inst.kind == 'MR'
        self.feas = np.where(self.is_mr[:, None], self.hand,
                             self.reach[:, :, 0] | self.reach[:, :, 1])
        self.n_poses = len(self.poses)
        self._rows = {}
        self._dur = {}

    def trow(self, p):
        """T(p, .) over every candidate pose. Identical arithmetic to sched."""
        r = self._rows.get(p)
        if r is None:
            lin, rot = self.poses[:, 0], self.poses[:, 1]
            r = traverse_time(lin - lin[p], rot - rot[p], self.t_fold)
            self._rows[p] = r
        return r

    def t(self, a, b):
        return 0.0 if a == b else float(self.trow(a)[b])

    def dur(self, tasks, p):
        """Cached stop_cost duration only (assignment discarded)."""
        key = (tuple(sorted(tasks)), p)
        d = self._dur.get(key)
        if d is None:
            d = stop_cost(self, tasks, p)[0]
            self._dur[key] = d
        return d


# ---------------------------------------------------------------------------
# Stop duration -- same closed form as sched.stop_duration, minimised in O(|U|)
# ---------------------------------------------------------------------------
def stop_cost(view, tasks, p):
    """(duration_s, assign) for doing `tasks` at pose `p`, or (inf, None).

    sched.stop_duration evaluates the SAME closed form
        slots = m + max(s_A, s_B, z)
    but by enumerating all 2**|SR| arm assignments. That is fine at n <= 10 and
    impossible at n = 50, so the minimisation is done directly here.

    Since m (the MR count) is common to all three terms, minimising
    max(m+s_A, m+s_B, m+z) is minimising max(s_A, s_B, z). Tasks reachable by
    exactly one arm are forced. For the rest, fix x = how many go to arm A: then
    s_A and s_B are fixed, and z is minimised by loading arm A first with tasks
    that occupy the zone only on arm B (free there), then with tasks whose zone
    flag does not depend on the arm, and only then with tasks that occupy the
    zone on arm A. Sweeping x over 0..t is therefore exact.

    Equivalence to sched.stop_duration -- which V1 checked against exhaustive
    slot enumeration -- is re-checked exhaustively by `check` below, and every
    schedule built on it still has to survive validate_schedule() (A3-K4).
    """
    mr, sr = [], []
    for i in tasks:
        (mr if view.is_mr[i] else sr).append(i)
    for i in mr:
        if not view.hand[i, p]:
            return np.inf, None

    forced = ([], [])
    both = []
    for i in sr:
        r0, r1 = bool(view.reach[i, p, 0]), bool(view.reach[i, p, 1])
        if r0 and r1:
            both.append(i)
        elif r0:
            forced[0].append(i)
        elif r1:
            forced[1].append(i)
        else:
            return np.inf, None

    zf = (sum(int(view.zone[i, p, 0]) for i in forced[0]) +
          sum(int(view.zone[i, p, 1]) for i in forced[1]))
    n0, n1 = len(forced[0]), len(forced[1])

    # both-arm tasks by (zone on arm A, zone on arm B)
    only_b, free, only_a = [], [], []        # cheap on A / same either way / cheap on B
    n11 = 0
    for i in both:
        z0, z1 = int(view.zone[i, p, 0]), int(view.zone[i, p, 1])
        if z0 == 0 and z1 == 1:
            only_b.append(i)                 # free on A, costs on B -> prefer A
        elif z0 == 1 and z1 == 0:
            only_a.append(i)                 # costs on A, free on B -> prefer B
        else:
            free.append(i)
            n11 += z0                        # (1,1) costs one wherever it goes
    t_both = len(both)

    best = None
    for x in range(t_both + 1):
        k01 = min(x, len(only_b))
        rem = x - k01
        kfree = min(rem, len(free))
        k10 = rem - kfree
        if k10 > len(only_a):
            continue
        z = zf + n11 + (len(only_b) - k01) + k10
        val = max(n0 + x, n1 + t_both - x, z)
        if best is None or val < best[0]:
            best = (val, k01, kfree, k10)

    if best is None:
        return np.inf, None
    val, k01, kfree, k10 = best
    arm_a = forced[0] + only_b[:k01] + free[:kfree] + only_a[:k10]
    arm_b = [i for i in both if i not in set(arm_a)] + forced[1]
    assign = {i: 'both' for i in mr}
    assign.update({i: GANTRY_ARMS[view.g][0] for i in arm_a})
    assign.update({i: GANTRY_ARMS[view.g][1] for i in arm_b})
    return (len(mr) + val) * view.dwell, assign


def dur_vec(view, tasks, greedy=False):
    """Stop duration for `tasks` at EVERY pose at once. O(|tasks| x |P|).

    The same closed form as stop_cost, with the x-sweep vectorised over poses.
    `greedy=False` requires every task to be feasible at the pose (inf
    otherwise) -- that is the stop-relocation query. `greedy=True` instead
    scores "everything in `tasks` that is feasible HERE", which is what a
    set-cover step actually buys at a candidate pose.

    Still no 2**n anywhere: this is |tasks|+1 vector passes over |P|.
    """
    P = view.n_poses
    mr = [i for i in tasks if view.is_mr[i]]
    sr = [i for i in tasks if not view.is_mr[i]]
    ok = np.ones(P, bool)
    m = np.zeros(P, np.int32)
    for i in mr:
        h = view.hand[i]
        if greedy:
            m += h
        else:
            ok &= h
            m += 1
    z = np.zeros(P, np.int32)
    n0, n1, zf = z.copy(), z.copy(), z.copy()
    c_ob, c_oa, c_fr, c_11 = z.copy(), z.copy(), z.copy(), z.copy()
    for i in sr:
        r0, r1 = view.reach[i, :, 0], view.reach[i, :, 1]
        z0, z1 = view.zone[i, :, 0], view.zone[i, :, 1]
        if not greedy:
            ok &= r0 | r1
        b = r0 & r1
        f0, f1 = r0 & ~r1, ~r0 & r1
        n0 += f0
        n1 += f1
        zf += (f0 & z0) + (f1 & z1)
        c_ob += b & ~z0 & z1
        c_oa += b & z0 & ~z1
        c_fr += b & (z0 == z1)
        c_11 += b & z0 & z1
    t_both = c_ob + c_oa + c_fr
    best = np.full(P, np.inf)
    for x in range(len(sr) + 1):
        k01 = np.minimum(x, c_ob)
        rem = x - k01
        kfree = np.minimum(rem, c_fr)
        k10 = rem - kfree
        good = (x <= t_both) & (k10 <= c_oa)
        zz = zf + c_11 + (c_ob - k01) + k10
        val = np.maximum(np.maximum(n0 + x, n1 + t_both - x), zz)
        np.minimum(best, np.where(good, val.astype(float), np.inf), out=best)
    return np.where(ok, (m + best) * view.dwell, np.inf)


# ---------------------------------------------------------------------------
# Schedule assembly -- start times accumulated exactly as validate_schedule
# replays them, so the two agree to float noise and not to a tolerance fudge
# ---------------------------------------------------------------------------
def build_solution(inst, views, plan, wall_s=0.0):
    """plan: gantry -> ordered [(pose, [task, ...])]. Returns a sched.Solution."""
    stops, finish, alloc = {}, {}, {}
    for g in inst.gantries:
        v = views[g]
        cur, t, out, mask = v.p0, 0.0, [], 0
        for p, tasks in plan.get(g, []):
            if not tasks:
                continue
            if p != cur:
                t += float(v.trow(cur)[p])
            d, assign = stop_cost(v, tasks, p)
            if assign is None:
                raise ValueError(f'gantry {g}: infeasible stop at pose {p}')
            u = 0
            for i in tasks:
                u |= 1 << i
            out.append(dict(pose=p, start=t, dur=d, tasks=u, assign=assign))
            t += d
            cur = p
            mask |= u
        stops[g] = out
        finish[g] = t
        alloc[g] = mask
    return Solution(max(finish.values()), alloc, stops, finish, wall_s, 0)


# ---------------------------------------------------------------------------
# Tour primitives (stage 3)
# ---------------------------------------------------------------------------
def tour_len(view, p0, order):
    t, cur = 0.0, p0
    for p in order:
        if p != cur:
            t += float(view.trow(cur)[p])
        cur = p
    return t


def nn_order(view, p0, poses):
    rest, order, cur = list(poses), [], p0
    while rest:
        row = view.trow(cur)
        k = int(np.argmin([row[p] for p in rest]))
        cur = rest.pop(k)
        order.append(cur)
    return order


def two_opt_or_opt(view, p0, order):
    """2-opt + or-opt on an OPEN path with a fixed start. A2 stage 3."""
    best, bl = list(order), tour_len(view, p0, order)
    improved = True
    while improved:
        improved = False
        m = len(best)
        for i in range(m):
            for j in range(i + 1, m):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                ln = tour_len(view, p0, cand)
                if ln < bl - TOL:
                    best, bl, improved = cand, ln, True
        for seg_len in (1, 2, 3):
            m = len(best)
            if seg_len >= m:
                break
            for i in range(m - seg_len + 1):
                seg = best[i:i + seg_len]
                rest = best[:i] + best[i + seg_len:]
                for k in range(len(rest) + 1):
                    for s in (seg, seg[::-1]):
                        cand = rest[:k] + list(s) + rest[k:]
                        if cand == best:
                            continue
                        ln = tour_len(view, p0, cand)
                        if ln < bl - TOL:
                            best, bl, improved = cand, ln, True
    return best


def _insert_deltas(view, seq):
    """Cheapest insertion cost of EVERY pose into the open path `seq`."""
    rows = [view.trow(q) for q in seq]
    best = rows[-1].copy()                       # append after the last node
    for k in range(1, len(seq)):
        base = float(rows[k - 1][seq[k]])
        np.minimum(best, rows[k - 1] + rows[k] - base, out=best)
    return np.maximum(best, 0.0)


def _best_insert(view, seq, p):
    k_best, d_best = len(seq), view.t(seq[-1], p)
    for k in range(1, len(seq)):
        d = view.t(seq[k - 1], p) + view.t(p, seq[k]) - view.t(seq[k - 1], seq[k])
        if d < d_best - 1e-12:
            k_best, d_best = k, d
    return k_best


# ---------------------------------------------------------------------------
# pose-tour, one gantry
# ---------------------------------------------------------------------------
def plan_gantry(view, tasks, tour='opt', do_refine=True, grid_radius=2,
                split=False, wide=False):
    """The four A2 stages for one gantry. Returns (stops, cost) or (None, inf).

    stops = ordered [(pose, [task, ...])]; cost = traverse + stop durations.
    `tour` selects how much of stage 3 runs -- 'opt' = nearest-neighbour then
    2-opt + or-opt (the locked design), 'nn' = nearest-neighbour only, 'cover' =
    keep the order the set cover inserted them in. The weaker settings exist
    only to measure what stage 3 is worth (A5-D7); they are not alternatives.
    """
    tasks = list(tasks)
    if not tasks:
        return [], 0.0
    feas = view.feas
    for i in tasks:
        if not feas[i].any():
            return None, np.inf

    # -- stage 1: handover poses first (B4: scarcest set, choose it first) ----
    covered, chosen, seq = set(), [], [view.p0]
    mr = sorted((i for i in tasks if view.is_mr[i]),
                key=lambda i: int(view.hand[i].sum()))
    for i in mr:
        if i in covered:
            continue
        cand = np.flatnonzero(view.hand[i])
        unc = [j for j in tasks if j not in covered]
        cov = feas[unc][:, cand].sum(axis=0)
        delta = _insert_deltas(view, seq)[cand]
        cost = delta + (dur_vec(view, unc, greedy=True)[cand] if wide else EPS)
        p = int(cand[int(np.argmax(cov / (cost + EPS)))])
        if p not in chosen:
            seq.insert(_best_insert(view, seq, p), p)
            chosen.append(p)
        covered |= {j for j in unc if feas[j, p]}

    # -- stage 2: traverse-weighted greedy set cover --------------------------
    while len(covered) < len(tasks):
        unc = [j for j in tasks if j not in covered]
        cov = feas[unc].sum(axis=0)
        delta = _insert_deltas(view, seq)
        cost = delta + (dur_vec(view, unc, greedy=True) if wide else EPS)
        score = np.where(cov > 0, cov / (cost + EPS), -1.0)
        p = int(np.argmax(score))
        if cov[p] == 0:
            return None, np.inf
        seq.insert(_best_insert(view, seq, p), p)
        chosen.append(p)
        covered |= {j for j in unc if feas[j, p]}

    # -- stage 3: tour --------------------------------------------------------
    order = [p for p in seq[1:]] if tour == 'cover' else \
        nn_order(view, view.p0, chosen)
    if tour == 'opt':
        order = two_opt_or_opt(view, view.p0, order)

    # -- stage 4: tasks -> stops, then local repair ---------------------------
    bucket = {p: [] for p in order}
    for i in sorted(tasks, key=lambda i: sum(1 for p in order if feas[i, p])):
        best = None
        for p in order:
            if not feas[i, p]:
                continue
            inc = view.dur(bucket[p] + [i], p) - view.dur(bucket[p], p)
            if best is None or inc < best[0] - 1e-12:
                best = (inc, p)
        bucket[best[1]].append(i)
    order = [p for p in order if bucket[p]]

    if do_refine:
        order, bucket = _refine(view, order, bucket, grid_radius, split, wide)

    stops = [(p, sorted(bucket[p])) for p in order if bucket[p]]
    return stops, _plan_cost(view, stops)


def _plan_cost(view, stops):
    t, cur = 0.0, view.p0
    for p, u in stops:
        if p != cur:
            t += float(view.trow(cur)[p])
        t += view.dur(u, p)
        cur = p
    return t


def _grid_neighbours(view, p, radius):
    """Poses within `radius` grid steps. lin index is clamped, rot is cyclic."""
    # the grid is meshgrid(lin, rot, 'ij'): index = i_lin * n_rot + i_rot
    n_rot = view.meta_n_rot
    i_lin, i_rot = divmod(p, n_rot)
    n_lin = view.n_poses // n_rot
    out = []
    for dl in range(-radius, radius + 1):
        for dr in range(-radius, radius + 1):
            if dl == 0 and dr == 0:
                continue
            jl, jr = i_lin + dl, (i_rot + dr) % n_rot
            if 0 <= jl < n_lin:
                out.append(jl * n_rot + jr)
    return out


def _refine(view, order, bucket, grid_radius, split=False, wide=False):
    """A2 stage 4: move tasks, drop stops, swap a pose for a cheaper neighbour.

    `split` adds a move that is NOT in the locked A2 design and was added after
    reading the B1 gap table: relocate one task out of an over-packed stop into
    a NEW stop of its own. See docs/p1_g8_sched2.md B2 -- it is reported as a
    separate scheduler, never folded into `pose-tour`.
    """
    feas = view.feas
    for _ in range(50):
        moved = False

        # (a) move one task to another stop
        for p in list(order):
            for i in list(bucket[p]):
                if len(order) == 1:
                    break
                base = view.dur(bucket[p], p)
                without = view.dur([j for j in bucket[p] if j != i], p)
                for q in order:
                    if q == p or not feas[i, q]:
                        continue
                    gain = (base - without) - (view.dur(bucket[q] + [i], q) -
                                               view.dur(bucket[q], q))
                    if gain > TOL:
                        bucket[p].remove(i)
                        bucket[q].append(i)
                        moved = True
                        break

        # (b) try to empty a stop entirely and drop it from the tour
        for p in list(order):
            if not bucket[p] or len(order) == 1:
                continue
            others = [q for q in order if q != p]
            tmp = {q: list(bucket[q]) for q in others}
            ok = True
            add = 0.0
            for i in bucket[p]:
                best = None
                for q in others:
                    if not feas[i, q]:
                        continue
                    inc = view.dur(tmp[q] + [i], q) - view.dur(tmp[q], q)
                    if best is None or inc < best[0] - 1e-12:
                        best = (inc, q)
                if best is None:
                    ok = False
                    break
                add += best[0]
                tmp[best[1]].append(i)
            if not ok:
                continue
            gone = view.dur(bucket[p], p)
            before = tour_len(view, view.p0, order)
            after = tour_len(view, view.p0, others)
            if (after + add) < (before + gone) - TOL:
                order = others
                bucket = {q: tmp[q] for q in others}
                moved = True

        order = [p for p in order if bucket[p]]

        # (c') POST-HOC: the same swap over the WHOLE pose set, not just the
        # grid neighbourhood, using the vectorised duration oracle.
        if wide:
            for idx in range(len(order)):
                p = order[idx]
                u = bucket[p]
                if not u:
                    continue
                prev = view.p0 if idx == 0 else order[idx - 1]
                c = dur_vec(view, u) + view.trow(prev)
                if idx + 1 < len(order):
                    c = c + view.trow(order[idx + 1])
                for q in bucket:
                    if q != p:
                        c[q] = np.inf
                q = int(np.argmin(c))
                if q != p and c[q] < c[p] - TOL:
                    order[idx] = q
                    bucket[q] = bucket.pop(p)
                    moved = True

        # (c) swap a stop's pose for a cheaper grid neighbour
        for idx, p in enumerate(list(order)):
            u = bucket[p]
            if not u:
                continue
            cur_cost = view.dur(u, p) + _edge_cost(view, order, idx, p)
            best = None
            for q in _grid_neighbours(view, p, grid_radius):
                if q in bucket:
                    continue
                if not all(feas[i, q] for i in u):
                    continue
                d = view.dur(u, q)
                if not np.isfinite(d):
                    continue
                c = d + _edge_cost(view, order, idx, q)
                if c < cur_cost - TOL and (best is None or c < best[0]):
                    best = (c, q)
            if best is not None:
                q = best[1]
                order[idx] = q
                bucket[q] = bucket.pop(p)
                moved = True

        # (d) POST-HOC, not in the locked A2 stage 4: split an over-packed stop
        # by giving one of its tasks a new stop of its own. The candidate pose
        # is chosen over the WHOLE pose set at once (feasibility mask times
        # cheapest-insertion vector), so this is still O(|P| x stops).
        if split:
            for p in list(order):
                for i in list(bucket[p]):
                    u = bucket[p]
                    if len(u) < 2:
                        break
                    saved = view.dur(u, p) - view.dur([j for j in u if j != i], p)
                    if saved <= view.dwell + TOL:
                        continue
                    seq = [view.p0] + order
                    delta = _insert_deltas(view, seq)
                    mask = feas[i].copy()
                    for q in bucket:
                        mask[q] = False
                    if not mask.any():
                        continue
                    d = np.where(mask, delta, np.inf)
                    q = int(np.argmin(d))
                    if saved - view.dwell - float(d[q]) > TOL:
                        order.insert(_best_insert(view, seq, q) - 1, q)
                        bucket[q] = [i]
                        bucket[p].remove(i)
                        moved = True

        if not moved:
            break
    return order, {p: bucket[p] for p in order}


def _edge_cost(view, order, idx, p):
    """Traverse charged for putting pose `p` at position idx of the tour."""
    prev = view.p0 if idx == 0 else order[idx - 1]
    c = view.t(prev, p)
    if idx + 1 < len(order):
        c += view.t(p, order[idx + 1])
    return c


# ---------------------------------------------------------------------------
# pose-tour, whole instance
# ---------------------------------------------------------------------------
def _allocate(inst, views, tour, wide=False):
    """Task -> gantry. A2: the gantries couple ONLY through allocation.

    No time synchronisation between gantries -- p1_g7 B7.2 says that coupling
    does not exist in the model, so a heuristic that used it would be
    optimising something the objective cannot see. Tasks are placed onto the
    gantry whose own completion time rises least, MR first (scarcest poses),
    then a one-pass move improvement.
    """
    gs = inst.gantries
    # The allocation rule (A2) is "the gantry whose own completion time rises
    # least". How that rise is ESTIMATED is an implementation choice: scoring
    # runs skip 2-opt and stage 4, because re-running an O(m^4) tour search
    # 2n times per instance costs minutes at n = 50. The final plan for each
    # gantry is always the full heuristic.
    tour = 'nn' if tour == 'opt' else tour
    feas_g = {g: views[g].feas.any(axis=1) for g in gs}
    order = sorted(range(inst.n),
                   key=lambda i: (not views[gs[0]].is_mr[i],
                                  int(sum(views[g].feas[i].sum() for g in gs))))
    cur = {g: [] for g in gs}
    cost = {g: 0.0 for g in gs}
    for i in order:
        best = None
        for g in gs:
            if not feas_g[g][i]:
                continue
            _, c = plan_gantry(views[g], cur[g] + [i], tour, False, wide=wide)
            if not np.isfinite(c):
                continue
            mk = max([c] + [cost[h] for h in gs if h != g])
            if best is None or mk < best[0] - 1e-12:
                best = (mk, g, c)
        if best is None:
            raise ValueError(f'task {i} is infeasible on every gantry')
        cur[best[1]].append(i)
        cost[best[1]] = best[2]

    if len(gs) > 1:
        for i in range(inst.n):
            src = next(g for g in gs if i in cur[g])
            mk = max(cost.values())
            for dst in gs:
                if dst == src or not feas_g[dst][i]:
                    continue
                _, cs = plan_gantry(views[src], [j for j in cur[src] if j != i],
                                    tour, False, wide=wide)
                _, cd = plan_gantry(views[dst], cur[dst] + [i], tour, False,
                                    wide=wide)
                if not (np.isfinite(cs) and np.isfinite(cd)):
                    continue
                new = max([cs, cd] + [cost[h] for h in gs
                                      if h not in (src, dst)])
                if new < mk - TOL:
                    cur[src].remove(i)
                    cur[dst].append(i)
                    cost[src], cost[dst] = cs, cd
                    break
    return cur


def pose_tour(inst, tour='opt', do_refine=True, grid_radius=2, split=False,
              wide=False):
    """The proposed heuristic. Returns a sched.Solution."""
    t0 = time.time()
    views = _views(inst)
    alloc = _allocate(inst, views, tour, wide)
    plan = {}
    for g in inst.gantries:
        stops, _ = plan_gantry(views[g], alloc[g], tour, do_refine,
                               grid_radius, split, wide)
        if stops is None:
            raise ValueError(f'gantry {g}: no feasible plan')
        plan[g] = stops
    return build_solution(inst, views, plan, time.time() - t0)


# ---------------------------------------------------------------------------
# Baselines (A2, deliberately weak in three DIFFERENT ways)
# ---------------------------------------------------------------------------
def sched_fixed(inst):
    """Round-robin gantry/arm decided up front; each task at its own best pose.

    Weakness by construction: no tour, no batching. The fallback below is not a
    strength -- gen_real only guarantees a task is feasible on SOME gantry, so
    without it the baseline would emit schedules that fail K4 rather than
    schedules that are merely bad.
    """
    t0 = time.time()
    views = _views(inst)
    gs = inst.gantries
    plan = {g: [] for g in gs}
    for i in range(inst.n):
        pg = gs[i % len(gs)]
        pa = (i // len(gs)) % 2
        cands = [(pg, pa), (pg, 1 - pa)]
        cands += [(g, a) for g in gs if g != pg for a in (pa, 1 - pa)]
        for g, a in cands:
            v = views[g]
            ok = v.hand[i] if v.is_mr[i] else v.reach[i, :, a]
            if not ok.any():
                continue
            row = v.trow(v.p0)
            p = int(np.argmin(np.where(ok, row, np.inf)))
            plan[g].append((p, [i]))
            break
        else:
            raise ValueError(f'task {i} is infeasible on every gantry/arm')
    return build_solution(inst, views, plan, time.time() - t0)


def _dispatch(inst, views, pick):
    """Shared loop for greedy/sequential: least-loaded gantry acts next.

    This is a dispatch rule, not a model change: the objective is a max over
    gantries, so a list scheduler that feeds the currently-idlest gantry is the
    textbook greedy. It adds no coupling that the model does not already have.
    """
    gs = inst.gantries
    remaining = set(range(inst.n))
    cur = {g: views[g].p0 for g in gs}
    now = {g: 0.0 for g in gs}
    plan = {g: [] for g in gs}
    active = set(gs)
    while remaining:
        if not active:
            raise ValueError('no gantry can serve the remaining tasks')
        g = min(active, key=lambda g: (now[g], g))
        v = views[g]
        rem = sorted(remaining)
        got = pick(v, cur[g], rem)
        if got is None:
            active.discard(g)
            continue
        p, tasks = got
        if p != cur[g]:
            now[g] += float(v.trow(cur[g])[p])
        now[g] += stop_cost(v, tasks, p)[0]
        plan[g].append((p, tasks))
        cur[g] = p
        remaining -= set(tasks)
    return plan


def sched_greedy(inst):
    """Nearest pose serving >= 1 remaining task; do as many as possible there."""
    t0 = time.time()
    views = _views(inst)

    def pick(v, cur, rem):
        cov = v.feas[rem].any(axis=0)
        if not cov.any():
            return None
        row = v.trow(cur)
        p = int(np.argmin(np.where(cov, row, np.inf)))
        return p, [i for i in rem if v.feas[i, p]]

    return build_solution(inst, views, _dispatch(inst, views, pick),
                          time.time() - t0)


def sched_sequential(inst):
    """One task per stop, nearest-neighbour order. Pays traverse per task."""
    t0 = time.time()
    views = _views(inst)

    def pick(v, cur, rem):
        m = v.feas[rem]
        if not m.any():
            return None
        row = v.trow(cur)
        cost = np.where(m, row[None, :], np.inf)
        k = int(np.argmin(cost))
        r, p = divmod(k, v.n_poses)
        return int(p), [rem[r]]

    return build_solution(inst, views, _dispatch(inst, views, pick),
                          time.time() - t0)


def pose_tour_wide(inst):
    """POST-HOC variant, designed AFTER reading the B1 gap table (see B2).

    Two changes, both aimed at the one mechanism B1 measured: the locked stage 2
    scores a candidate pose by coverage per unit of TRAVERSE, so it is blind to
    the dwell its own choice induces, and the locked stage 4 can only nudge a
    pose by two grid steps. Here the cover score charges the induced stop
    duration too, and the stage 4 pose swap searches the whole 2376-pose grid.
    NOT the heuristic locked in A2, and not evidence for K1 on the K2 seeds.
    """
    return pose_tour(inst, wide=True)


SCHEDULERS = {'pose-tour': pose_tour, 'fixed': sched_fixed,
              'greedy': sched_greedy, 'sequential': sched_sequential}
SCHEDULERS_PLUS = dict(SCHEDULERS)
SCHEDULERS_PLUS['pose-tour+wide'] = pose_tour_wide


# ---------------------------------------------------------------------------
# Lower bounds (A3-K3)
# ---------------------------------------------------------------------------
def lb_analytic(inst):
    """(LB, LB_reach, LB_work). Both parts are proved valid in A3-K3."""
    views = _views(inst)
    gs = inst.gantries
    worst = 0.0
    for i in range(inst.n):
        best = np.inf
        for g in gs:
            v = views[g]
            m = v.feas[i]
            if m.any():
                best = min(best, float(v.trow(v.p0)[m].min()))
        worst = max(worst, best)
    lb_reach = worst + inst.dwell
    n_mr = int((inst.kind == 'MR').sum())
    n_sr = inst.n - n_mr
    lb_work = inst.dwell * math.ceil((n_mr + math.ceil(n_sr / 2)) / len(gs))
    return max(lb_reach, lb_work), lb_reach, float(lb_work)


def sub_instance(inst, idx):
    idx = np.asarray(sorted(idx))
    return Instance(inst.kind[idx], inst.poses,
                    {g: inst.reach[g][idx] for g in inst.gantries},
                    {g: inst.zone[g][idx] for g in inst.gantries},
                    {g: inst.hand[g][idx] for g in inst.gantries},
                    inst.p0, None if inst.xyz is None else inst.xyz[idx],
                    inst.dwell, inst.t_fold, inst.label + f'+sub{len(idx)}',
                    dict(inst.meta, sub=idx.tolist()))


def lb_subset(inst, k=8, n_draw=3, seed=0):
    """max over `n_draw` seeded random subsets of size k of OPT(subset).

    Valid because OPT(A) <= OPT(A u B) -- proof in A3-K3: deleting tasks never
    lengthens a stop (a, b, z are monotone in the task set) and never lengthens
    the tour (T obeys the triangle inequality).
    """
    if inst.n <= k:
        return float(solve_exact(inst).makespan), [list(range(inst.n))]
    rng = np.random.default_rng(10_000 + seed)
    best, used = 0.0, []
    for _ in range(n_draw):
        idx = sorted(rng.choice(inst.n, k, replace=False).tolist())
        m = float(solve_exact(sub_instance(inst, idx)).makespan)
        used.append(idx)
        best = max(best, m)
    return best, used


# ---------------------------------------------------------------------------
# helpers / CLI
# ---------------------------------------------------------------------------
def _views(inst):
    vs = {}
    for g in inst.gantries:
        v = GantryView(inst, g)
        v.meta_n_rot = _n_rot(v)
        vs[g] = v
    return vs


def _n_rot(view):
    """Number of rotation columns in the pose grid, read off the pose array."""
    lin = view.poses[:, 0]
    change = np.flatnonzero(np.abs(np.diff(lin)) > 1e-12)
    return int(change[0]) + 1 if len(change) else view.n_poses


def traverse_share(inst, sol):
    """Fraction of the CRITICAL gantry's timeline spent traversing (K5.6)."""
    g = max(sol.finish, key=lambda k: sol.finish[k])
    work = sum(s['dur'] for s in sol.stops[g])
    return 1.0 - work / sol.finish[g] if sol.finish[g] > 0 else float('nan')


def cmd_run(a):
    inst = gen_real(a.n_tasks, a.seed, a.n_mr, tuple(a.gantries),
                    maps=(a.map1, a.map2))
    print(inst.describe(), '\n')
    names = list(SCHEDULERS) if a.sched == 'all' else [a.sched]
    for name in names:
        sol = SCHEDULERS_PLUS[name](inst)
        print(f'--- {name}: makespan {sol.makespan:.4f} s  '
              f'({sol.wall_s:.3f} s wall, traverse share '
              f'{traverse_share(inst, sol)*100:.1f}%)')
        if a.verbose:
            print(sol.report(inst))
    if a.exact:
        sol = solve_exact(inst)
        print(f'--- exact: makespan {sol.makespan:.4f} s '
              f'({sol.wall_s:.2f} s wall)')
    return 0


def cmd_lb(a):
    inst = gen_real(a.n_tasks, a.seed, a.n_mr, tuple(a.gantries),
                    maps=(a.map1, a.map2))
    lb, r, w = lb_analytic(inst)
    print(f'LB_analytic {lb:.4f} s   (reach {r:.4f}, work {w:.4f})')
    s, used = lb_subset(inst, a.sub_size, a.sub_draws, a.seed)
    print(f'LB_subset   {s:.4f} s   over {used}')
    print(f'LB          {max(lb, s):.4f} s')
    return 0


def cmd_check(a):
    """stop_cost vs sched.stop_duration, exhaustively. The one shared formula."""
    from reachability_gng.sched import gen_random_small
    bad = checked = 0
    worst = 0.0
    cases = [(4, 3, 1), (5, 3, 0), (5, 4, 2), (6, 3, 1)]
    for n, P, mr in cases:
        for seed in range(a.seeds):
            inst = gen_random_small(n, P, 7000 + seed, n_mr=mr)
            v = GantryView(inst, 1)
            v.meta_n_rot = _n_rot(v)
            for U in range(1, 1 << n):
                tasks = [i for i in range(n) if U >> i & 1]
                for p in range(P):
                    x = stop_cost(v, tasks, p)[0]
                    y = stop_duration(inst, 1, U, p)[0]
                    checked += 1
                    if np.isinf(x) and np.isinf(y):
                        continue
                    if np.isinf(x) != np.isinf(y):
                        bad += 1
                        continue
                    worst = max(worst, abs(x - y))
                    if abs(x - y) > TOL:
                        bad += 1
    print(f'stop_cost vs sched.stop_duration: {checked} (subset, pose) cases, '
          f'{bad} mismatches, max |diff| {worst:.3e}')

    # and on the real map, where the masks are not random
    for seed in range(3):
        inst = gen_real(6, seed, 1, (1,), maps=(a.map1, a.map2))
        v = GantryView(inst, 1)
        v.meta_n_rot = _n_rot(v)
        rng = np.random.default_rng(seed)
        ps = rng.choice(len(inst.poses[1]), 200, replace=False)
        for U in range(1, 1 << inst.n):
            tasks = [i for i in range(inst.n) if U >> i & 1]
            for p in ps:
                x = stop_cost(v, tasks, int(p))[0]
                y = stop_duration(inst, 1, U, int(p))[0]
                checked += 1
                if np.isinf(x) and np.isinf(y):
                    continue
                if np.isinf(x) != np.isinf(y) or abs(x - y) > TOL:
                    bad += 1
    print(f'including the real map: {checked} cases, {bad} mismatches')
    return 0 if bad == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(q):
        q.add_argument('--seed', type=int, default=0)
        q.add_argument('--n-tasks', type=int, default=6)
        q.add_argument('--n-mr', type=int, default=0)
        q.add_argument('--gantries', type=int, nargs='+', default=[1, 2])
        q.add_argument('--map1', default='/tmp/cap_g1_rail160.npz')
        q.add_argument('--map2', default='/tmp/cap_g2_rail160.npz')

    r = sub.add_parser('run')
    common(r)
    r.add_argument('--sched', default='all',
                   choices=['all'] + list(SCHEDULERS_PLUS))
    r.add_argument('--exact', action='store_true')
    r.add_argument('--verbose', action='store_true')
    r.set_defaults(fn=cmd_run)

    lo = sub.add_parser('lb')
    common(lo)
    lo.add_argument('--sub-size', type=int, default=8)
    lo.add_argument('--sub-draws', type=int, default=3)
    lo.set_defaults(fn=cmd_lb)

    c = sub.add_parser('check')
    common(c)
    c.add_argument('--seeds', type=int, default=12)
    c.set_defaults(fn=cmd_check)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    raise SystemExit(main())
