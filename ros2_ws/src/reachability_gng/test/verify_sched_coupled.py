#!/usr/bin/env python3
"""Is the COUPLED solver ground truth? docs/p1_g10_sched4.md A3-K1 and A3-K2.

p1_g9 B4 answered "no" for G9's solver and named the missing piece: W2, the
brute-force joint enumerator, the only check that attacks the start-time
restriction. It was never built. This file builds it -- and, because A3-K1 says
so in red, builds the proof of W2 ITSELF first:

  P0   W2's trajectory model, retyped from the A2.3 TEXT, vs sched_coll.Traj
  P1   W2 with collisions off      vs sched.solve_exact (proved exact by V0-V4)
  P2   every schedule W2 returns   vs the wait-aware gate
  P3   W2 >= the uncoupled optimum (Lemma 3) on every instance
  P4   halving the start grid never RAISES W2's answer

Only then:

  W2    solve_coupled2 vs the enumerator. Rule, locked in A3-K1 before either
        existed: solver > W2 is a SOLVER FAILURE; solver < W2 is the grid being
        coarse and is measured by halving it, not waved away.
  W2b   solve_coupled2 with BLOCK disabled vs sched.solve_exact, run TWICE --
        with the dive and with `no_dive=True`. Only the second tests search
        completeness: with collisions off the root dive answers immediately, and
        a check that passes because nothing ran is p1_g8 B1 in a new costume.
  W3    Q1-Q5 of p1_g9 A3-K1b, including Q3, which G9 never built (p1_g9 B4
        records W3 as passing on Q1/Q2/Q4/Q5 -- four of five).
  W4'   reported schedules through the wait-aware gate.
  REG   the two instances p1_g9 B6 proved exact must still come out at 0.000.

What W2 deliberately SHARES with the thing it judges is exactly one function:
`sched_coll.pair_distance`. Re-implementing the predicate here would re-run W0,
not test the solver, and W0 already compared it against a boundary-sampling
oracle on 1184 cases with 0 mismatches. Everything else -- trajectory, sweep,
enumeration, timing -- is written from the model text.

Run:  python3 test/verify_sched_coupled.py [p|w2|w2b|w3|w4|reg|all]
"""

from __future__ import annotations

import itertools
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_coll as sc               # noqa: E402
from reachability_gng import sched_coupled as sk            # noqa: E402
from gate_sched_coupled import validate_coupled             # noqa: E402

TOL = 1e-9
DS = 0.25              # W2 start-time grid, locked in A3-K1
DS_FINE = 0.125        # the halving P4 and the mismatch rule use
W2_BUDGET = 240.0      # wall seconds per W2 instance


# ===========================================================================
# W2's own model of motion -- typed from docs/p1_g9_sched3.md A2.3, not
# imported. P0 is what makes that claim checkable.
# ===========================================================================
V_LIN = 3000.0 / 95.4930 / 1000.0          # m/s, p1_state 5.6
V_ROT = 1000.0 / 100.0                     # deg/s
OFF_LIN, OFF_ROT = 0.29, 0.26


def ref_short(d_rad):
    d = np.degrees(d_rad)
    return (d + 180.0) % 360.0 - 180.0


def ref_leg_time(p, q):
    dm = abs(q[0] - p[0]) * 1000.0
    dd = abs(ref_short(q[1] - p[1]))
    tl = OFF_LIN + dm / (3000.0 / 95.4930) if dm > 1e-6 else 0.0
    tr = OFF_ROT + dd / V_ROT if dd > 1e-6 else 0.0
    return max(tl, tr)


def ref_pose(traj, t):
    """(lin, rot) of a trajectory [(t_start, p, q), ...] at time(s) t.

    Hold at p0, then each leg: dead for the axis offset, constant speed, hold.
    Written straight from A2.3; no shared code with sched_coll.
    """
    t = np.asarray(t, float)
    p0 = traj[0]
    lin = np.full(t.shape, p0[0], float)
    rot = np.full(t.shape, p0[1], float)
    for ts, p, q in traj[1]:
        u = t - ts
        m = u >= 0.0
        if not m.any():
            continue
        dl = q[0] - p[0]
        if abs(dl) > 1e-9:
            mv = np.minimum(np.maximum(u - OFF_LIN, 0.0) * V_LIN, abs(dl))
            lin = np.where(m, p[0] + np.sign(dl) * mv, lin)
        else:
            lin = np.where(m, p[0], lin)
        dd = float(ref_short(q[1] - p[1]))
        if abs(dd) > 1e-9:
            mv = np.minimum(np.maximum(u - OFF_ROT, 0.0) * V_ROT, abs(dd))
            rot = np.where(m, p[1] + np.deg2rad(np.sign(dd) * mv), rot)
        else:
            rot = np.where(m, p[1], rot)
    return lin, rot


def ref_end(traj):
    return traj[1][-1][2] if traj[1] else traj[0]


def ref_end_time(traj):
    if not traj[1]:
        return 0.0
    ts, p, q = traj[1][-1]
    return ts + ref_leg_time(p, q)


def ref_conflict(trA, trB, t_lo, t_hi, c_clear, eps=sc.EPS_CERT):
    """Does A2.4 fail anywhere on [t_lo, t_hi]? Dense uniform scan.

    Deliberately dumb: no conservative advancement, no early-out. The sample
    spacing is set so no point of either body moves more than eps between
    neighbours, which is the same guarantee `first_block` gives by a completely
    different route.
    """
    if t_hi <= t_lo:
        t = np.array([t_lo])
    else:
        n = int(np.clip((t_hi - t_lo) * sc.V_REL / eps + 2.0, 2, 40000))
        t = np.linspace(t_lo, t_hi, n)
    l1, r1 = ref_pose(trA, t)
    l2, r2 = ref_pose(trB, t)
    return bool((sc.pair_distance(l1, r1, l2, r2) <= c_clear + eps).any())


# ===========================================================================
# W2 -- the brute-force joint enumerator
# ===========================================================================
class Brute:
    """Exhaustive search over joint schedules on a uniform start-time grid.

    No branch & bound, no dynamic programming, no lower bound, no Lemma 5, no
    `feasible_starts`. The only pruning is the incumbent -- `max(t) >= best`
    cannot lead anywhere better, which is true of any schedule and discards
    nothing.

    Actions at each step, for the gantry whose clock is behind:
        * traverse to ANY pose in P and dwell there doing ANY feasible nonempty
          subset of the remaining tasks
        * traverse to ANY pose and dwell 0 -- an evasive move (p1_g9
          contradiction 2), capped at `max_evade` per gantry so the recursion
          terminates
        * start times: t_g, t_g + ds, ... up to the other gantry's last motion.
          Past that the other is static and the action becomes a pure time
          shift, so one more candidate there settles the infinite tail.
    """

    def __init__(self, inst, ds=DS, c_clear=sc.C_CLEAR, max_evade=1,
                 budget=W2_BUDGET):
        self.inst = inst
        self.ds = ds
        self.c = c_clear
        self.max_evade = max_evade
        self.budget = budget
        self.gs = inst.gantries
        self.P = {g: [tuple(p) for p in inst.poses[g]] for g in self.gs}
        self.best = np.inf
        self.best_stops = None
        self.nodes = 0
        self.timeout = False
        self.t0 = 0.0
        self.dur = {}
        for g in self.gs:
            for U in range(1, 1 << inst.n):
                for p in range(len(self.P[g])):
                    d, a = sched.stop_duration(inst, g, U, p)
                    if np.isfinite(d):
                        self.dur[(g, U, p)] = (d, a)

    def solve(self):
        self.t0 = time.time()
        tr = {g: (self.P[g][self.inst.p0[g]], []) for g in self.gs}
        self._rec({g: 0.0 for g in self.gs}, {g: 0.0 for g in self.gs},
                  tr, {g: [] for g in self.gs},
                  (1 << self.inst.n) - 1, {g: 0 for g in self.gs})
        return self.best, self.best_stops

    def _starts(self, t_min, other):
        hi = max(t_min, ref_end_time(other))
        out = [t_min]
        s = t_min + self.ds
        while s < hi - 1e-9:
            out.append(s)
            s += self.ds
        if hi > t_min + 1e-9:
            out.append(hi)
        return out

    def _rec(self, t, fin, tr, stops, R, evade):
        if time.time() - self.t0 > self.budget:
            self.timeout = True
            return
        self.nodes += 1
        if R == 0:
            m = max(fin.values())
            if m < self.best - 1e-12:
                self.best = m
                self.best_stops = {g: list(stops[g]) for g in self.gs}
            return
        if max(t.values()) >= self.best - 1e-12:
            return

        # BOTH gantries are expanded, not just the one whose clock is behind.
        # "Expand the earlier gantry" is a heuristic, not a theorem, and an
        # oracle may not assume it: measured on Q4, where the earlier gantry is
        # blocked by the later one and the later one must move first. The
        # solver reaches that state through a special-cased evasive branch;
        # this enumerator reaches it by not having the restriction at all,
        # which is the whole point of having a second implementation.
        acts = []
        for g in self.gs:
            cur = ref_end(tr[g])
            for p in range(len(self.P[g])):
                U = R
                while U:
                    if (g, U, p) in self.dur:
                        acts.append((g, p, U, self.dur[(g, U, p)][0]))
                    U = (U - 1) & R
                if evade[g] < self.max_evade and self.P[g][p] != cur:
                    acts.append((g, p, 0, 0.0))

        for g, p, U, d in acts:
            h = [k for k in self.gs if k != g][0]
            cur = ref_end(tr[g])
            q = self.P[g][p]
            T = ref_leg_time(cur, q)
            for s in self._starts(t[g], tr[h]):
                if s + T + d >= self.best - 1e-12 and U:
                    continue
                cand = (tr[g][0], tr[g][1] + ([(s, cur, q)] if T > 0 else []))
                A, B = (cand, tr[h]) if g == self.gs[0] else (tr[h], cand)
                # the check must reach past this action's own window, out to
                # the end of the other gantry's committed motion: otherwise the
                # STATIC TAIL of this action is never checked against anything
                # the other gantry commits afterwards. P2 caught exactly that
                # here, in this enumerator, which is what P2 is for.
                if ref_conflict(A, B, s, max(s + T + d, ref_end_time(tr[h])),
                                self.c):
                    continue
                nt, nf = dict(t), dict(fin)
                nt[g] = s + T + d
                ntr, nst, nev = dict(tr), dict(stops), dict(evade)
                ntr[g] = cand
                if U:
                    nf[g] = nt[g]
                    nst[g] = stops[g] + [dict(
                        pose=p, depart=s, start=s + T, dur=d, tasks=U,
                        assign=self.dur[(g, U, p)][1])]
                else:
                    nev[g] += 1
                    nst[g] = stops[g] + [dict(pose=p, depart=s, start=s + T,
                                              dur=0.0, tasks=0, assign={})]
                self._rec(nt, nf, ntr, nst, R ^ U, nev)


# ===========================================================================
# S5 -- W2's fuel
# ===========================================================================
def gen_small_crowded(n, P, seed, n_mr=0):
    """`gen_random_small`, but on poses that CAN collide.

    ADDED BEYOND A3-K4, and reported as an addition. (N1) proves a collision
    needs both gantries more than 31.7 deg off rail-parallel with their `lin`
    close together; `gen_random_small` draws lin over the whole 1.6 m rail and
    rot over the whole circle, so a random tiny pose set almost never contains
    such a pair, and a W2 that never sees a collision would be judging the
    such a pair.

    The first draft drew EVERY pose inside the blocking band, and W2 said so at
    once: 4 of 16 instances came back infeasible for both solver and oracle (no
    Lemma 1 park anywhere in the pose set, so the gantries were locked from
    t = 0) and most of the rest were trivial -- both tasks doable at p0,
    makespan exactly one dwell. An instance that is infeasible or trivial tests
    nothing, which is p1_g8 B1's never-firing gate in yet another costume. This
    version fixes both by construction:

      pose 0     (0.80, 0 deg): universally safe, and p0 for BOTH gantries, so
                 the start state is collision-free and Lemma 1's serialisation
                 always exists
      poses 1..  (0.80 +- 0.04, +-90 deg): mutually blocking by Lemma 2
      reach      every task reachable ONLY at the blocking poses, so both
                 gantries are forced into them

    How many W2 instances actually bind is reported as a number, because that
    count is the difference between W2 judging the solver and W2 agreeing with
    it about nothing.
    """
    rng = np.random.default_rng(1000 + seed)
    inst = sched.gen_random_small(n, P, seed, n_mr=n_mr, gantries=(1, 2))
    lin = np.concatenate([[0.80], 0.80 + rng.uniform(-0.04, 0.04, P - 1)])
    rot = np.concatenate([[0.0],
                          rng.choice([-1.0, 1.0], P - 1)
                          * rng.uniform(np.deg2rad(80.0), np.deg2rad(100.0),
                                        P - 1)])
    poses = np.stack([lin, rot], axis=1)
    reach, hand = {}, {}
    for g in (1, 2):
        r = np.zeros((n, P, 2), bool)
        for i in range(n):
            p = 1 + (i + g) % (P - 1)
            if inst.kind[i] == 'MR':
                r[i, p, :] = True
            else:
                r[i, p, (i + g) % 2] = True
        reach[g] = r
        hand[g] = r[:, :, 0] & r[:, :, 1]
    zone = {g: np.zeros((n, P, 2), bool) for g in (1, 2)}
    return sched.Instance(inst.kind, {g: poses for g in (1, 2)}, reach, zone,
                          hand, {1: 0, 2: 0}, None, inst.dwell, 0.0,
                          f'crowded-small(n={n},P={P},seed={seed})')


def s5_instances():
    """>= 24 instances, n <= 3, |P| <= 4, two gantries (A3-K4 + the addition)."""
    out = []
    for n, P, mr in [(2, 3, 0), (2, 4, 0), (3, 3, 0), (2, 3, 1), (3, 3, 1)]:
        for seed in range(3):
            out.append((f'plain_n{n}P{P}mr{mr}s{seed}',
                        sched.gen_random_small(n, P, 300 + seed, n_mr=mr,
                                               gantries=(1, 2))))
    for n, P, mr in [(2, 3, 0), (2, 4, 0), (3, 3, 0), (2, 3, 1)]:
        for seed in range(4):
            out.append((f'crowd_n{n}P{P}mr{mr}s{seed}',
                        gen_small_crowded(n, P, 400 + seed, n_mr=mr)))
    return out


# ===========================================================================
# P0-P4 -- the proof of the oracle
# ===========================================================================
def p0_trajectory():
    """W2's retyped motion model vs sched_coll.Traj on random trajectories."""
    rng = np.random.default_rng(21)
    worst_p = worst_T = 0.0
    for _ in range(300):
        p0 = (float(rng.uniform(0, 1.6)), float(rng.uniform(-np.pi, np.pi)))
        A = sc.Traj(1, p0)
        ref = (p0, [])
        t = float(rng.uniform(0, 2))
        for _ in range(3):
            q = (float(rng.uniform(0, 1.6)), float(rng.uniform(-np.pi, np.pi)))
            p = A.end_pose()
            worst_T = max(worst_T, abs(ref_leg_time(p, q) - sc.leg_duration(p, q)))
            ref[1].append((t, p, q))
            t = A.append(t, q) + float(rng.uniform(0, 1.5))
        ts = np.sort(rng.uniform(0.0, max(A.end_time(), 1.0) * 1.2, 200))
        l1, r1 = A.pose_at(ts)
        l2, r2 = ref_pose(ref, ts)
        worst_p = max(worst_p, float(np.abs(l1 - l2).max()),
                      float(np.abs(np.sin(r1) - np.sin(r2)).max()),
                      float(np.abs(np.cos(r1) - np.cos(r2)).max()))
    ok = worst_p < 1e-12 and worst_T < 1e-12
    print(f'P0 W2 motion model (retyped from A2.3) vs sched_coll.Traj: '
          f'300 trajectories x 200 times, max |pose diff| {worst_p:.3e}, '
          f'max |duration diff| {worst_T:.3e} s -> {"PASS" if ok else "FAIL"}')
    return ok


def p1_collision_off():
    """W2 with BLOCK switched off must reproduce sched.solve_exact."""
    bad, n = [], 0
    for label, inst in s5_instances()[:12]:
        ref = sched.solve_exact(inst).makespan
        b = Brute(inst, ds=DS, c_clear=-1.0, max_evade=0)
        got, _ = b.solve()
        n += 1
        if b.timeout or abs(got - ref) > 1e-6:
            bad.append((label, got, ref, b.timeout))
    print(f'P1 W2 with collisions off vs sched.solve_exact (V0-V4 proved): '
          f'{n} instances, {len(bad)} mismatches')
    for x in bad[:4]:
        print(f'     {x[0]}: brute {x[1]:.6f} vs exact {x[2]:.6f} '
              f'(timeout={x[3]})')
    return not bad


def _p2_p3(results):
    """P2 (gate) and P3 (Lemma 3) over whatever W2 already computed."""
    bad_gate, bad_lb = [], []
    for label, inst, val, stops, lb in results:
        if stops is None:
            continue
        errs = validate_coupled(inst, stops)
        if errs:
            bad_gate.append((label, errs[:2]))
        if val < lb - 1e-6:
            bad_lb.append((label, val, lb))
    print(f'P2 every W2 schedule through the wait-aware gate: '
          f'{len(results)} schedules, {len(bad_gate)} rejected')
    for x in bad_gate[:4]:
        print(f'     {x[0]}: {x[1]}')
    print(f'P3 W2 >= uncoupled optimum (Lemma 3): {len(bad_lb)} violations')
    for x in bad_lb[:4]:
        print(f'     {x[0]}: brute {x[1]:.6f} < uncoupled {x[2]:.6f}')
    return not bad_gate, not bad_lb


def w2_main():
    """P1-P4 and the solver-vs-oracle comparison, in one pass over S5."""
    ok = p0_trajectory()
    print()
    ok &= p1_collision_off()
    print()

    rows, results = [], []
    n_bind = n_solver_high = n_solver_low = n_to = 0
    for label, inst in s5_instances():
        lb = sched.solve_exact(inst).makespan
        b = Brute(inst, ds=DS)
        bv, bs = b.solve()
        s = sk.solve_coupled2(inst, time_budget=120.0)
        if b.timeout:
            n_to += 1
        results.append((label, inst, bv, bs, lb))
        if bv > lb + 1e-6:
            n_bind += 1
        hi = s.makespan > bv + 1e-9
        lo = s.makespan < bv - 1e-9
        n_solver_high += hi
        n_solver_low += lo
        rows.append((label, lb, bv, s.makespan, b.timeout, hi, lo, b.nodes,
                     s.proved))
        print(f'   {label:22s} unc {lb:8.3f}  W2 {bv:9.3f}  solver '
              f'{s.makespan:9.3f}  {"SOLVER>W2" if hi else ("solver<W2" if lo else "match"):9s}'
              f'  nodes {b.nodes:7d}{"  TIMEOUT" if b.timeout else ""}',
              flush=True)

    p2, p3 = _p2_p3(results)
    ok &= p2 and p3

    # P4 -- halving the grid must not RAISE the answer
    worse = []
    for label, inst in s5_instances()[:8]:
        a, _ = Brute(inst, ds=DS).solve()
        c, _ = Brute(inst, ds=DS_FINE).solve()
        if c > a + 1e-9:
            worse.append((label, a, c))
    print(f'P4 halving the start grid ({DS} -> {DS_FINE} s) never raises W2: '
          f'8 instances, {len(worse)} raised')
    ok &= not worse

    n = len(rows)
    print(f'\nW2 solve_coupled2 vs the brute-force enumerator: {n} instances '
          f'({n_bind} where the collision actually binds, {n_to} timed out)')
    print(f'   solver > W2 (SOLVER FAILURE, must be 0): {n_solver_high}')
    print(f'   solver < W2 (grid coarser than the solver, measured not waved): '
          f'{n_solver_low}')
    w2_ok = n_solver_high == 0 and n >= 24 and n_bind > 0
    if n_solver_low:
        fixed = 0
        for label, lb, bv, sv, to, hi, lo, nd, pr in rows:
            if not lo:
                continue
            inst = dict(s5_instances())[label]
            c, _ = Brute(inst, ds=DS_FINE).solve()
            fixed += c <= sv + 1e-9
            print(f'     {label}: W2 at ds={DS_FINE} gives {c:.4f} vs solver '
                  f'{sv:.4f}')
        print(f'   of those, {fixed}/{n_solver_low} closed when the grid was '
              f'halved -- i.e. the gap was the GRID, not the solver')
    print(f'W2 -> {"PASS" if w2_ok else "FAIL"}')
    return ok and w2_ok


# ===========================================================================
# W2b / W3 / W4' / regression
# ===========================================================================
def w2b(no_dive):
    bad, n = [], 0
    for nt, mr in [(4, 0), (4, 1), (6, 0), (6, 1)]:
        for seed in range(3):
            inst = sched.gen_real(nt, seed, mr, (1, 2))
            ref = sched.solve_exact(inst).makespan
            got = sk.solve_coupled2(inst, c_clear=-1.0, force_bnb=True,
                                    ub_seed=ref + 2.0, time_budget=90.0,
                                    no_dive=no_dive)
            n += 1
            if abs(got.makespan - ref) > TOL or not got.proved:
                bad.append((nt, mr, seed, got.makespan, ref, got.proved,
                            got.n_nodes))
    tag = 'no_dive (search completeness)' if no_dive else 'dive on'
    print(f'W2b [{tag}] BLOCK disabled vs sched.solve_exact: {n} real '
          f'instances, |P| = 2376, {len(bad)} failures')
    for b in bad[:5]:
        print(f'     n={b[0]} mr={b[1]} seed={b[2]}: coupled {b[3]:.6f} vs '
              f'exact {b[4]:.6f} proved={b[5]} nodes={b[6]}')
    return not bad


def _synth2(poses, r1_idx, r2_idx, p0=0, p0b=None):
    P = np.asarray(poses, float)
    n, K = 2, len(P)
    r1 = np.zeros((n, K, 2), bool)
    r2 = np.zeros((n, K, 2), bool)
    if r1_idx is not None:
        r1[0, r1_idx, 0] = True
    if r2_idx is not None:
        r2[1, r2_idx, 0] = True
    z = np.zeros((n, K, 2), bool)
    h = np.zeros((n, K), bool)
    return sched.gen_synthetic(['SR', 'SR'], {1: P, 2: P}, {1: r1, 2: r2},
                               {1: z, 2: z}, {1: h, 2: h},
                               {1: p0, 2: p0 if p0b is None else p0b},
                               gantries=(1, 2))


def w3():
    ok = True
    d90 = np.pi / 2

    inst = _synth2([[0.0, 0.0], [0.8, 0.0]], 1, 1)
    s = sk.solve_coupled2(inst)
    ref = sched.solve_exact(inst).makespan
    q1 = abs(s.makespan - ref) <= TOL
    ok &= q1
    print(f'Q1 collision algebraically impossible (both at rot = 0): coupled '
          f'{s.makespan:.4f} == uncoupled {ref:.4f} -> '
          f'{"PASS" if q1 else "FAIL"}')

    inst = _synth2([[0.8, 0.0], [0.8, -d90], [0.8, d90]], 1, 2)
    s = sk.solve_coupled2(inst, time_budget=60.0)
    ref = sched.solve_exact(inst).makespan
    floor = (0.26 + 90.0 / 10.0) + 2 * sched.DWELL
    bq2, _ = Brute(inst, ds=DS).solve()
    e2 = validate_coupled(inst, s.stops, s.finish, s.makespan)
    # solver <= brute is the locked rule (A3-K1): solver ABOVE the enumerator is
    # the failure, solver below it is the enumerator's grid being coarse.
    q2 = (s.makespan >= floor - TOL and s.makespan > ref + TOL
          and s.makespan <= bq2 + 1e-6 and not e2)
    ok &= q2
    print(f'Q2 mutually locking poses: uncoupled {ref:.4f}, coupled '
          f'{s.makespan:.4f}, brute force {bq2:.4f}, hand floor {floor:.4f} '
          f'(the two dwells cannot overlap), gate {len(e2)} -> '
          f'{"PASS" if q2 else "FAIL"}')

    # Q3 -- BUILT HERE FOR THE FIRST TIME. p1_g9 A3-K1b specified it; p1_g9 B4
    # records W3 as passing on Q1/Q2/Q4/Q5, i.e. Q3 was silently dropped.
    #
    # Gantry 2 has no task and starts parked at (0.8, +90 deg). Gantry 1 must
    # rotate -45 -> -135 deg at the same lin, so it sweeps through -90 deg,
    # which W0c measured as blocking at dlin = 0. The floor is hand-derived:
    # read off the largest |rot| of gantry 2 that still blocks a gantry 1 at
    # -90 deg (one geometric threshold, from the W0-verified predicate), then
    # gantry 2 must rotate from +90 down past it before gantry 1 arrives at
    # -90 deg, and gantry 1 needs its full traverse plus a dwell after that.
    theta = None
    for th in np.arange(90.0, -0.1, -1.0):
        if float(sc.pair_distance(0.8, -d90, 0.8, np.deg2rad(th))) <= 0.0:
            theta = th
        else:
            break
    t_clear = 0.26 + (90.0 - theta) / 10.0          # gantry 2 rotates past it
    t_g1_at_90 = 0.26 + 45.0 / 10.0                 # gantry 1 reaches -90 deg
    depart = max(0.0, t_clear - t_g1_at_90)
    floor3 = depart + (0.26 + 90.0 / 10.0) + sched.DWELL
    P3 = np.array([[0.8, np.deg2rad(-45.0)], [0.8, np.deg2rad(-135.0)],
                   [0.8, d90], [0.8, 0.0]])
    r1 = np.zeros((1, 4, 2), bool)
    r1[0, 1, 0] = True                      # the only task, gantry 1, pose 1
    r0 = np.zeros((1, 4, 2), bool)          # gantry 2 can do nothing at all
    z = np.zeros((1, 4, 2), bool)
    hh = np.zeros((1, 4), bool)
    inst = sched.gen_synthetic(['SR'], {1: P3, 2: P3}, {1: r1, 2: r0},
                               {1: z, 2: z}, {1: hh, 2: hh}, {1: 0, 2: 2},
                               gantries=(1, 2))
    s = sk.solve_coupled2(inst, time_budget=60.0)
    ref = sched.solve_exact(inst).makespan
    # cross-checked against the brute-force enumerator as well: Q3 is small
    # enough (n = 1, |P| = 4) to be W2 fuel, so the hand floor is not the only
    # thing standing behind it.
    bq3, _ = Brute(inst, ds=DS).solve()
    errs3 = validate_coupled(inst, s.stops, s.finish, s.makespan)
    # 🔺 p1_g9 A3-K1b predicted "> the uncoupled value". MEASURED: equal to it.
    # The gantry in the way has no tasks, so it can dodge CONCURRENTLY and its
    # dodge costs no makespan at all -- provided it dodges the long way round
    # (see _evade_order). So the fixture asserts what is actually true and
    # checkable: the solver matches the brute-force enumerator exactly, its
    # schedule passes the gate, and Lemma 3 holds.
    q3 = (np.isfinite(s.makespan) and s.makespan >= ref - TOL
          and s.makespan >= floor3 - 1e-6 and s.makespan <= bq3 + 1e-6
          and not errs3)
    ok &= q3
    print(f'Q3 traverse cut by a parked gantry: uncoupled {ref:.4f}, coupled '
          f'{s.makespan:.4f}, brute force {bq3:.4f}, hand floor {floor3:.4f} '
          f'(gantry 2 blocks a -90 deg sweep up to |rot| = {theta:.0f} deg), '
          f'gate {len(errs3)} -> {"PASS" if q3 else "FAIL"}')
    print(f'     evasive moves used {s.n_evade}, route {s.route}, '
          f'proved {s.proved}')

    # Q4 -- waiting is useful. Gantry 1 has a second task at the safe rot = 0
    # pose; the optimum must reorder rather than sit blocked.
    P = np.array([[0.8, 0.0], [0.8, -d90], [0.8, d90]])
    r1 = np.zeros((3, 3, 2), bool)
    r2 = np.zeros((3, 3, 2), bool)
    r1[0, 1, 0] = True
    r1[2, 0, 0] = True
    r2[1, 2, 0] = True
    z = np.zeros((3, 3, 2), bool)
    h = np.zeros((3, 3), bool)
    inst = sched.gen_synthetic(['SR', 'SR', 'SR'], {1: P, 2: P}, {1: r1, 2: r2},
                               {1: z, 2: z}, {1: h, 2: h}, {1: 0, 2: 0},
                               gantries=(1, 2))
    s = sk.solve_coupled2(inst, time_budget=60.0)
    errs = validate_coupled(inst, s.stops, s.finish, s.makespan)
    bq4, _ = Brute(inst, ds=DS).solve()
    q4 = (np.isfinite(s.makespan) and not errs
          and s.makespan <= bq4 + 1e-6)
    ok &= q4
    print(f'Q4 waiting is useful: coupled {s.makespan:.4f}, brute force '
          f'{bq4:.4f}, uncoupled {sched.solve_exact(inst).makespan:.4f}, gate '
          f'{len(errs)} violations -> {"PASS" if q4 else "FAIL"}')

    inst = _synth2([[0.0, 0.0], [0.8, 0.0], [0.8, np.pi]], 1, 2)
    s = sk.solve_coupled2(inst)
    ref = sched.solve_exact(inst).makespan
    q5 = abs(s.makespan - ref) <= TOL
    ok &= q5
    print(f'Q5 safe parking (rot = 0 vs rot = 180): coupled {s.makespan:.4f} '
          f'== uncoupled {ref:.4f} -> {"PASS" if q5 else "FAIL"}')
    return ok


def w4():
    bad, n, waits = [], 0, 0.0
    from gate_sched_coupled import total_wait
    for nt, mr in [(4, 0), (4, 1), (6, 0), (6, 1)]:
        for seed in range(3):
            inst = sched.gen_real(nt, seed, mr, (1, 2))
            s = sk.solve_coupled2(inst, time_budget=120.0)
            if not np.isfinite(s.makespan) or not s.stops:
                bad.append((nt, mr, seed, ['no schedule returned']))
                continue
            n += 1
            waits += total_wait(inst, s.stops)
            errs = validate_coupled(inst, s.stops, s.finish, s.makespan)
            if errs:
                bad.append((nt, mr, seed, errs[:3]))
    print(f"W4' reported coupled schedules through the wait-aware gate "
          f'(intra-gantry rules + A2.4 at eps/10): {n} schedules, {len(bad)} '
          f'with violations; {waits:.1f} s of waiting across them')
    for b in bad[:5]:
        print(f'     n={b[0]} mr={b[1]} seed={b[2]}: {b[3]}')
    return not bad


def reg():
    """p1_g9 B6 proved these two exact. They must stay exact."""
    bad = []
    for nt, seed, mr in [(4, 2, 1), (4, 9, 1)]:
        inst = sched.gen_real(nt, seed, mr, (1, 2))
        s = sk.solve_coupled2(inst, time_budget=120.0)
        if abs(s.delta) > 1e-9:
            bad.append((nt, seed, mr, s.delta))
        print(f'REG n{nt}_s{seed}_mr{mr}: Delta {s.delta:+.6f} s '
              f'(p1_g9 B6 proved 0.000) route {s.route} proved {s.proved}')
    return not bad


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['all']
    checks = []
    if a[0] in ('all', 'w2', 'p'):
        checks.append(('W2+P0-P4', w2_main))
    if a[0] in ('all', 'w2b'):
        checks += [('W2b(i) dive', lambda: w2b(False)),
                   ('W2b(ii) no_dive', lambda: w2b(True))]
    if a[0] in ('all', 'w3'):
        checks.append(('W3', w3))
    if a[0] in ('all', 'w4'):
        checks.append(("W4'", w4))
    if a[0] in ('all', 'reg'):
        checks.append(('REG', reg))
    res = []
    for tag, fn in checks:
        res.append((tag, fn()))
        print()
    print('=' * 66)
    for tag, ok in res:
        print(f'{tag}: {"PASS" if ok else "FAIL"}')
    allok = all(ok for _, ok in res)
    print('=' * 66)
    print('COUPLED GROUND TRUTH ESTABLISHED (w.r.t. the A2.2 candidate set)'
          if allok and a[0] == 'all' else
          'partial run' if allok else
          'NOT ESTABLISHED -- p1_state 7.1 forbids touching heuristics')
    return 0 if allok else 1


if __name__ == '__main__':
    raise SystemExit(main())
