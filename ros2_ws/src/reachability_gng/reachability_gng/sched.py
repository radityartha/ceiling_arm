"""Layer 3 -- allocation & scheduling, XD [ST-MR-TA]. GROUND TRUTH half.

This module holds the MODEL and an EXACT solver, and nothing else. No
heuristic lives here and none may be added: a schedule that is wrong still
looks reasonable -- there is no exception and no red test -- so the optimum has
to exist before anything is allowed to be compared against it (p1_state 7.1).

The model is written out in full in docs/p1_g7_sched.md A1 and was locked
before this file was executed once. Summary:

  resource   two gantries, two arms each; the two arms of a gantry share the
             gantry pose p = (lin, rot). That shared variable IS the XD.
  timeline   per gantry: stop -> traverse -> stop -> ...  Both arms of a gantry
             are dead during traverse, so work blocks are contiguous by
             construction.
  task       reach-and-dwell, duration 2.0 s (p1_state 5.8).
             SR  one arm, at a pose that arm reaches (L1 tol 0.05 m).
             MR  BOTH arms of one gantry, ONE shared 2.0 s window. Locked in
                 g7 A2-K3, and the reason is p1_state 5.8: N-arm success is one
                 COMMON window, never "each succeeded at some point", because
                 the latter is satisfiable by alternating and alternating is
                 exactly what prior work does.
  mutex      task t occupies the zone for arm a at pose p iff a reaches it at
             0.05 AND it lies within r=0.20 of the partner's reachable set at
             that same pose. At most one arm per gantry in the zone at a time.
             MEASURED on the real map: 51.8% of reachable (node, pose)
             pairs occupy the zone, and the mutex costs extra slots at 10.9%
             of (task set, pose) pairs -- yet it costs ZERO makespan on every
             real instance solved, because the optimum always has another
             feasible pose. Measured, not argued: `diag`, and g7 B3.
  cost       T_traverse = max(T_lin, T_rot), p1_state 5.6, LOCKED, not
             re-derived here.
  objective  makespan = last dwell window end, max over gantries, from t=0 with
             both gantries at their instance-given initial pose.

Exactness is DP over (remaining task set, current pose), not MIP: the setup
cost is continuous and sequence-dependent, so a time-indexed MILP is exact only
after time is discretised, and that discretisation error would be
indistinguishable from model error. The DP discretises nothing.

  h(R, p)  = min_{p'} [ T(p, p') + w(R, p') ]
  w(R, p') = min over nonempty U subset of R, U feasible at p'
                   [ dur(U, p') + h(R \\ U, p') ]

Revisiting a pose is allowed by the DP and never needed: T obeys the triangle
inequality (offsets are charged per MOVING axis only), so it is never strictly
better to leave and come back.

NOT MODELLED, and every loss number out of this file is therefore a LOWER
BOUND, not a value (docs/p1_g7_sched.md A4): gantry-gantry structural collision
(mount plates sweep r=0.4 m at y=+-0.36, overlapping in y in [-0.04, 0.04]);
arm fold/unfold cost (`--t-fold`, default 0.0, because it was never measured
and 7.2 forbids inventing constants); arm motion inside a stop; online
capability change.

Subcommands
-----------
  gen      build one instance and describe it
  solve    build one instance and solve it EXACTLY, print the schedule
  sweep    measure how instance size trades against solver wall time (A2-K1)
  ablate   mutex on/off and MR-vs-SR, the two measurements A5 predicted
  diag     why the mutex costs nothing: local binding, escape, optimal path
"""

from __future__ import annotations

import argparse
import itertools
import time
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Setup cost. LOCKED in p1_state 5.6 (nine runs, R^2 = 1.00000). Copied, not
# re-derived. s is the commanded pulse/s; the defaults are the bridge values.
# ---------------------------------------------------------------------------
PULSES_PER_MM = 95.4930
PULSES_PER_DEG = 100.0
V_LIN_MM_S = 3000.0 / PULSES_PER_MM      # 31.4160 mm/s
V_ROT_DEG_S = 1000.0 / PULSES_PER_DEG    # 10.0 deg/s
T_LIN_OFFSET = 0.29
T_ROT_OFFSET = 0.26

DWELL = 2.0                              # p1_state 5.8 / 8b, continuous hold

GANTRY_ARMS = {1: ('arm1', 'arm2'), 2: ('arm3', 'arm4')}


def traverse_time(dlin_m, drot_rad, t_fold=0.0):
    """Scalar/array T_traverse for a linear and a rotational delta.

    The offset is charged only on an axis that actually moves: T(p, p) = 0.
    Charging 0.29 s to stand still would make the gantry pay for not moving,
    and would also break the triangle inequality the DP leans on.
    """
    dl = np.abs(np.asarray(dlin_m, float)) * 1000.0
    dr = np.degrees(np.abs(np.asarray(drot_rad, float)))
    dr = np.minimum(dr, 360.0 - dr)                  # rotation axis is cyclic
    tl = np.where(dl > 1e-6, T_LIN_OFFSET + dl / V_LIN_MM_S, 0.0)
    tr = np.where(dr > 1e-6, T_ROT_OFFSET + dr / V_ROT_DEG_S, 0.0)
    t = np.maximum(tl, tr)
    return np.where(t > 0, t + t_fold, 0.0) if t_fold else t


def traverse_matrix(poses, t_fold=0.0):
    """(P, P) pairwise setup times for a pose set (P, 2) = (lin m, rot rad)."""
    lin, rot = poses[:, 0], poses[:, 1]
    return traverse_time(lin[:, None] - lin[None, :],
                         rot[:, None] - rot[None, :], t_fold)


# ---------------------------------------------------------------------------
# Instance
# ---------------------------------------------------------------------------
@dataclass
class Instance:
    """One scheduling instance, fully expanded into boolean oracles.

    Everything the solver needs is here as plain arrays, so a synthetic
    instance with hand-built masks and a real one read off
    cap_g{1,2}_rail160.npz are the same object to the solver. Per gantry g:

      poses[g]  (P, 2)      candidate gantry poses, (lin m, rot rad)
      reach[g]  (n, P, 2)   arm slot 0 = right-plate arm, 1 = its half-turn
                            partner; True = reaches the task at L1 = 0.05 m
      zone[g]   (n, P, 2)   arm slot occupies the r=0.20 mutex zone there
      hand[g]   (n, P)      BOTH arms reach strictly -> a handover may happen
      p0[g]     int         initial pose index at t = 0
    """

    kind: np.ndarray                      # (n,) '<U2', 'SR' or 'MR'
    poses: dict
    reach: dict
    zone: dict
    hand: dict
    p0: dict
    xyz: np.ndarray = None
    dwell: float = DWELL
    t_fold: float = 0.0
    label: str = ''
    meta: dict = field(default_factory=dict)

    @property
    def n(self):
        return len(self.kind)

    @property
    def gantries(self):
        return tuple(sorted(self.poses))

    def without_mutex(self):
        """Same instance with the r=0.20 zone removed. Ablation only."""
        z = {g: np.zeros_like(v) for g, v in self.zone.items()}
        return Instance(self.kind, self.poses, self.reach, z, self.hand,
                        self.p0, self.xyz, self.dwell, self.t_fold,
                        self.label + '+nomutex', dict(self.meta))

    def describe(self):
        out = [f'instance {self.label!r}: {self.n} tasks '
               f'({int((self.kind == "MR").sum())} MR), '
               f'gantries {self.gantries}, '
               f'|P| = {len(self.poses[self.gantries[0]])}']
        for g in self.gantries:
            r, z, hd = self.reach[g], self.zone[g], self.hand[g]
            out.append(f'  gantry {g}  p0 = {self.pose_str(g, self.p0[g])}')
            for i in range(self.n):
                arms = GANTRY_ARMS[g]
                bits = [f'{arms[k]} {int(r[i, :, k].sum()):4d}p'
                        f'/{int(z[i, :, k].sum()):4d}z' for k in (0, 1)]
                out.append(f'    t{i} {self.kind[i]}  ' + '  '.join(bits) +
                           f'  handover {int(hd[i].sum()):4d}p')
        return '\n'.join(out)

    def pose_str(self, g, p):
        lin, rot = self.poses[g][p]
        return f'(lin {lin*1000:7.1f} mm, rot {np.degrees(rot):7.1f} deg)'


# ---------------------------------------------------------------------------
# Stop duration -- the closed form, and the arm assignment that achieves it
# ---------------------------------------------------------------------------
def stop_duration(inst, g, U, p):
    """(duration_s, assign) for doing task set U at pose p, or (inf, None).

    All tasks last one dwell, so a stop normalises to slots of `dwell`. With
    m MR tasks and s_A / s_B SR tasks on the two arms, of which z_A / z_B
    occupy the mutex zone:

        slots = max(m + s_A, m + s_B, m + z_A + z_B)

    Lower bound: each arm needs its own count, and the unary mutex must
    serialise m + z_A + z_B slots. Achievable: MR first, then A-zone, then
    B-zone, then backfill the non-zone tasks into whatever slots each arm has
    left -- there are always enough because slots >= s_A + m and >= s_B + m.

    This closed form is the single most suspicious line in the module. It is
    checked against exhaustive slot enumeration in test/verify_sched_exact.py
    (V1), which never touches this function.
    """
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
        s = [v.count(0), v.count(1)]
        z = sum(inst.zone[g][i, p, a] for i, a in zip(sr, v))
        slots = max(m + s[0], m + s[1], m + int(z))
        if slots < best:
            best, best_v = slots, v
    if best_v is None:
        return np.inf, None
    assign = {i: 'both' for i in mr}
    assign.update({i: GANTRY_ARMS[g][a] for i, a in zip(sr, best_v)})
    return best * inst.dwell, assign


def _dur_table(inst, g):
    """(2**n, P) stop duration for every task subset at every pose.

    Vectorised over poses; the arm-assignment search is the inner loop, so the
    total work is sum over U of 2**|U| = 3**n pose-length vector ops.
    """
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
            if not feas.any():
                continue
            load = max(m + v.count(0), m + v.count(1))
            slots = np.maximum(load, m + zc).astype(float)
            np.minimum(best, np.where(feas, slots, np.inf), out=best)
        dur[U] = best * inst.dwell
    return dur


# ---------------------------------------------------------------------------
# Exact solver
# ---------------------------------------------------------------------------
@dataclass
class GantryDP:
    """Solved DP for one gantry over the full task universe.

    `cost[A]` is the exact time for this gantry to complete exactly the task
    subset A starting from p0 -- every subset at once, which is what makes the
    outer task-to-gantry enumeration cheap.
    """
    g: int
    keep: np.ndarray       # candidate pose indices actually retained
    T: np.ndarray          # (K, K) setup times among retained poses
    dur: np.ndarray        # (2**n, K)
    h: np.ndarray          # (2**n, K) cost-to-go
    w: np.ndarray          # (2**n, K) cost-to-go given we are ALREADY at p'
    r0: int                # index of p0 inside `keep`

    @property
    def cost(self):
        return self.h[:, self.r0]


def solve_gantry(inst, g):
    """Exact DP for gantry g over all 2**n task subsets."""
    n = inst.n
    dur_full = _dur_table(inst, g)
    usable = np.isfinite(dur_full[1:]).any(axis=0)
    keep = np.flatnonzero(usable)
    p0 = inst.p0[g]
    if p0 not in keep:
        keep = np.sort(np.append(keep, p0))
    r0 = int(np.searchsorted(keep, p0))
    T = traverse_matrix(inst.poses[g][keep], inst.t_fold)
    dur = dur_full[:, keep]
    K = len(keep)

    h = np.full((1 << n, K), np.inf)
    w = np.full((1 << n, K), np.inf)
    h[0] = 0.0
    order = sorted(range(1, 1 << n), key=lambda R: bin(R).count('1'))
    for R in order:
        wr = w[R]
        U = R
        while U:
            np.minimum(wr, dur[U] + h[R ^ U], out=wr)
            U = (U - 1) & R
        cols = np.flatnonzero(np.isfinite(wr))
        if len(cols):
            np.min(T[:, cols] + wr[cols], axis=1, out=h[R])
    return GantryDP(g, keep, T, dur, h, w, r0)


def _reconstruct(inst, dp, A):
    """Walk the solved DP back into an explicit schedule for one gantry."""
    stops, R, r, t = [], A, dp.r0, 0.0
    while R:
        wr = dp.w[R]
        cand = dp.T[r] + wr
        r2 = int(np.argmin(cand))
        t += dp.T[r, r2]
        bestU, bestc = None, np.inf
        U = R
        while U:
            c = dp.dur[U, r2] + dp.h[R ^ U, r2]
            if c < bestc:
                bestU, bestc = U, c
            U = (U - 1) & R
        p = int(dp.keep[r2])
        d, assign = stop_duration(inst, dp.g, bestU, p)
        stops.append(dict(pose=p, start=t, dur=d, tasks=bestU, assign=assign))
        t += d
        R ^= bestU
        r = r2
    return stops, t


@dataclass
class Solution:
    makespan: float
    assign: dict            # gantry -> task bitmask
    stops: dict             # gantry -> list of stop dicts
    finish: dict            # gantry -> completion time
    wall_s: float = 0.0
    n_alloc: int = 0        # task->gantry allocations enumerated

    def report(self, inst):
        out = [f'EXACT makespan {self.makespan:.4f} s   '
               f'({self.n_alloc} allocations, {self.wall_s:.2f} s wall)']
        for g in sorted(self.stops):
            out.append(f'  gantry {g}  finish {self.finish[g]:.4f} s')
            for s in self.stops[g]:
                who = ', '.join(f't{i}->{s["assign"][i]}'
                                for i in sorted(s['assign']))
                out.append(f'    t={s["start"]:7.3f}s +{s["dur"]:5.2f}s  '
                           f'{inst.pose_str(g, s["pose"])}  {who}')
            if not self.stops[g]:
                out.append('    (idle)')
        return '\n'.join(out)


def solve_exact(inst):
    """Exact minimum makespan over task->gantry allocation and both schedules.

    Every gantry DP is solved once for ALL subsets, so the allocation loop is a
    max over two table lookups. Exact with respect to the candidate pose set
    carried by the instance -- which for `real` instances is the full 33x72
    grid, i.e. no extra discretisation beyond the capability map itself.
    """
    t0 = time.time()
    gs = inst.gantries
    dps = {g: solve_gantry(inst, g) for g in gs}
    costs = {g: dps[g].cost for g in gs}

    # a task may only go to a gantry that can do it at all
    choices = []
    for i in range(inst.n):
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
        stops[g], finish[g] = _reconstruct(inst, dps[g], best_alloc[g])
    return Solution(float(best), best_alloc, stops, finish,
                    time.time() - t0, n_alloc)


# ---------------------------------------------------------------------------
# Generators
# ---------------------------------------------------------------------------
def _gantry_oracles(cap, node_idx):
    """(reach, zone, hand) for one gantry, read straight off a CapabilityMap.

    Arm slot 0 is the stored right-plate arm; slot 1 is its partner, which is
    the same mask rolled by half the rotation axis -- exact, p1_state 3. The
    zone test is deliberately per-arm: "I reach it AND the other arm's 0.20
    dilation covers it here".
    """
    L, R = len(cap.lin), len(cap.rot)
    m0 = cap.masks[0][node_idx].reshape(-1, L, R)          # tol 0.05
    mr = cap.masks[2][node_idx].reshape(-1, L, R)          # tol 0.20
    p0a, p0b = m0, np.roll(m0, R // 2, axis=2)
    pra, prb = mr, np.roll(mr, R // 2, axis=2)
    flat = lambda a: a.reshape(len(node_idx), L * R)
    reach = np.stack([flat(p0a), flat(p0b)], axis=2)
    zone = np.stack([flat(p0a & prb), flat(p0b & pra)], axis=2)
    hand = flat(p0a & p0b)
    return reach, zone, hand


def gen_real(n_tasks, seed, n_mr=0, gantries=(1, 2), p0=None, t_fold=0.0,
             maps=('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz'),
             max_reject=10_000):
    """Draw tasks from the real capability map nodes. Seeded and deterministic.

    Rejects rather than patches: an SR task must be reachable by some arm at
    some pose, an MR task must have at least one strict-intersection pose. The
    reject count is returned in `meta` so it is reported as a number.
    """
    from reachability_gng.capability import CapabilityMap
    caps = {g: CapabilityMap.load(maps[g - 1]) for g in gantries}
    ref = caps[gantries[0]]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    if p0 is None:                       # rail zero, rotation zero
        p0_idx = int(np.argmin(np.hypot(poses[:, 0] - ref.lin[0], poses[:, 1])))
    else:
        p0_idx = int(p0)

    rng = np.random.default_rng(seed)
    kinds = np.array(['MR'] * n_mr + ['SR'] * (n_tasks - n_mr), dtype='<U2')
    chosen, rejects = [], 0
    while len(chosen) < n_tasks:
        cand = int(rng.integers(0, len(ref.nodes)))
        want_mr = kinds[len(chosen)] == 'MR'
        ok = False
        for g in gantries:
            r, _, hd = _gantry_oracles(caps[g], np.array([cand]))
            ok |= bool(hd[0].any()) if want_mr else bool(r[0].any())
        if ok:
            chosen.append(cand)
        else:
            rejects += 1
            if rejects > max_reject:
                raise RuntimeError('generator could not find feasible tasks')
    node_idx = np.array(chosen)

    reach, zone, hand = {}, {}, {}
    for g in gantries:
        reach[g], zone[g], hand[g] = _gantry_oracles(caps[g], node_idx)
    return Instance(kinds, {g: poses for g in gantries}, reach, zone, hand,
                    {g: p0_idx for g in gantries}, ref.nodes[node_idx],
                    DWELL, t_fold, f'real(n={n_tasks},mr={n_mr},seed={seed})',
                    dict(nodes=node_idx.tolist(), rejects=rejects,
                         n_poses=len(poses)))


def gen_synthetic(kinds, poses, reach, zone, hand, p0, label='synthetic',
                  gantries=(1,), t_fold=0.0):
    """Hand-built instance. Only for the pathological cases (g7 A2-K5) and V2."""
    kinds = np.asarray(kinds, dtype='<U2')
    return Instance(kinds, {g: np.asarray(poses[g], float) for g in gantries},
                    {g: np.asarray(reach[g], bool) for g in gantries},
                    {g: np.asarray(zone[g], bool) for g in gantries},
                    {g: np.asarray(hand[g], bool) for g in gantries},
                    dict(p0), None, DWELL, t_fold, label)


def gen_random_small(n_tasks, n_poses, seed, n_mr=0, gantries=(1,),
                     density=0.55, t_fold=0.0):
    """Tiny random instance on a tiny random pose set -- the V2 cross-check fuel.

    Masks are random, not from the oracle, on purpose: V2 tests the SOLVER, and
    random masks reach corners of the constraint structure that the smooth real
    capability map never produces.
    """
    rng = np.random.default_rng(seed)
    kinds = np.array(['MR'] * n_mr + ['SR'] * (n_tasks - n_mr), dtype='<U2')
    lin = np.sort(rng.choice(np.arange(0, 33) * 0.05, n_poses, replace=False))
    rot = np.deg2rad(rng.choice(np.arange(-180, 180, 5), n_poses,
                                replace=False))
    poses = np.stack([lin, rot], axis=1)
    reach, zone, hand = {}, {}, {}
    for g in gantries:
        r = rng.random((n_tasks, n_poses, 2)) < density
        for i in range(n_tasks):                 # keep every task feasible
            if kinds[i] == 'MR':
                p = rng.integers(0, n_poses)
                r[i, p, :] = True
            elif not r[i].any():
                r[i, rng.integers(0, n_poses), rng.integers(0, 2)] = True
        z = r & (rng.random((n_tasks, n_poses, 2)) < 0.5)
        reach[g], zone[g], hand[g] = r, z, r[:, :, 0] & r[:, :, 1]
    return Instance(kinds, {g: poses for g in gantries}, reach, zone, hand,
                    {g: 0 for g in gantries}, None, DWELL, t_fold,
                    f'rand(n={n_tasks},P={n_poses},seed={seed})')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _instance_from_args(a):
    return gen_real(a.n_tasks, a.seed, a.n_mr, tuple(a.gantries),
                    t_fold=a.t_fold, maps=(a.map1, a.map2))


def cmd_gen(a):
    inst = _instance_from_args(a)
    print(inst.describe())
    print(f'  rejected {inst.meta["rejects"]} infeasible draws')
    return 0


def cmd_solve(a):
    inst = _instance_from_args(a)
    print(inst.describe())
    print(f'  rejected {inst.meta["rejects"]} infeasible draws\n')
    sol = solve_exact(inst)
    print(sol.report(inst))
    return 0


def cmd_sweep(a):
    """A2-K1: what actually fits the 120 s budget. Measured, not guessed."""
    print(f'{"n":>2} {"mr":>3} {"gantry":>7} {"|P|":>6} {"|keep|":>7} '
          f'{"makespan":>10} {"wall_s":>8}  budget')
    print('-' * 62)
    for n in a.tasks:
        for gs in ([1], [1, 2]):
            inst = gen_real(n, a.seed, a.n_mr, tuple(gs), t_fold=a.t_fold,
                            maps=(a.map1, a.map2))
            t0 = time.time()
            try:
                sol = solve_exact(inst)
                ms = f'{sol.makespan:10.4f}'
            except MemoryError:
                ms = '       OOM'
            wall = time.time() - t0
            keep = len(solve_gantry(inst, gs[0]).keep)
            print(f'{n:>2} {a.n_mr:>3} {len(gs):>7} '
                  f'{inst.meta["n_poses"]:>6} {keep:>7} {ms} {wall:8.2f}  '
                  f'{"OK" if wall <= 120 else "OVER"}')
    return 0


def dwell_share(sol):
    """Fraction of the CRITICAL gantry's timeline actually spent dwelling.

    The rest is gantry traverse. Measured at 4-26% on real instances, and that
    ratio is the whole reason the mutex costs nothing there (g7 B3): the
    timeline is setup, not work.
    """
    g = max(sol.finish, key=lambda k: sol.finish[k])
    work = sum(s['dur'] for s in sol.stops[g])
    return work / sol.finish[g] if sol.finish[g] > 0 else float('nan')


def cmd_ablate(a):
    """A5 D2/D3: does the mutex bind, and is MR really expensive? Measured."""
    print(f'{"seed":>5} {"n":>2} {"mr":>3} | {"makespan":>9} {"no-mutex":>9} '
          f'{"mutex":>7} | {"all-SR":>9} {"MR cost":>8} | {"dwell%":>6}')
    print('-' * 78)
    rows = []
    for seed in a.seeds:
        inst = gen_real(a.n_tasks, seed, a.n_mr, tuple(a.gantries),
                        t_fold=a.t_fold, maps=(a.map1, a.map2))
        sol = solve_exact(inst)
        m, share = sol.makespan, dwell_share(sol)
        m_nx = solve_exact(inst.without_mutex()).makespan
        sr = Instance(np.array(['SR'] * inst.n, dtype='<U2'), inst.poses,
                      inst.reach, inst.zone, inst.hand, inst.p0, inst.xyz,
                      inst.dwell, inst.t_fold, inst.label + '+allSR')
        m_sr = solve_exact(sr).makespan if a.n_mr else float('nan')
        rows.append((m, m_nx, m_sr, share))
        print(f'{seed:>5} {a.n_tasks:>2} {a.n_mr:>3} | {m:9.3f} {m_nx:9.3f} '
              f'{m - m_nx:7.3f} | {m_sr:9.3f} {m - m_sr:8.3f} | '
              f'{share*100:5.1f}%')
    r = np.array(rows)
    print('-' * 78)
    print(f'mean mutex cost {np.nanmean(r[:, 0] - r[:, 1]):+.3f} s '
          f'({np.mean(r[:, 0] > r[:, 1] + 1e-9)*100:.0f}% of instances bind)'
          + (f'   mean MR cost {np.nanmean(r[:, 0] - r[:, 2]):+.3f} s'
             if a.n_mr else '')
          + f'   mean dwell share {np.nanmean(r[:, 3])*100:.1f}%')
    return 0


def cmd_diag(a):
    """Why the mutex costs nothing. Three questions, asked in order.

    Reported as measurement rather than argument, because the answer decides
    whether the r=0.20 zone belongs in the scheduling model at all (g7 B3).

      Q1 does it bind LOCALLY -- (task set, pose) pairs where the closed form
         exceeds the no-mutex value at all?
      Q2 is there an ESCAPE -- for a task set that binds somewhere, is there
         another feasible pose that is mutex-free at no extra slot?
      Q3 do the stops the OPTIMUM actually uses pay anything?
    """
    gs = tuple(a.gantries)
    tot = bind = sets_bind = sets_escape = 0
    for seed in a.seeds:
        inst = gen_real(a.n_tasks, seed, 0, (gs[0],), t_fold=a.t_fold,
                        maps=(a.map1, a.map2))
        nx = inst.without_mutex()
        P = len(inst.poses[gs[0]])
        for U in range(1, 1 << inst.n):
            du = np.array([stop_duration(inst, gs[0], U, p)[0]
                           for p in range(P)])
            dn = np.array([stop_duration(nx, gs[0], U, p)[0]
                           for p in range(P)])
            feas = np.isfinite(dn)
            if not feas.any():
                continue
            b = (du > dn + 1e-9) & feas
            tot += int(feas.sum())
            bind += int(b.sum())
            if b.any():
                sets_bind += 1
                free = feas & ~b
                if free.any() and du[free].min() <= dn[feas].min() + 1e-9:
                    sets_escape += 1
    print(f'Q1 mutex costs slots at {bind}/{tot} = {bind/max(tot,1)*100:.1f}% '
          f'of (task set, pose) pairs -- so it is NOT vacuous')
    print(f'Q2 {sets_bind} task sets bind at some pose; {sets_escape} '
          f'({sets_escape/max(sets_bind,1)*100:.0f}%) have another feasible '
          f'pose that is mutex-free at no extra slot')

    rows = []
    for seed in a.seeds:
        inst = gen_real(a.n_tasks, seed, 0, gs, t_fold=a.t_fold,
                        maps=(a.map1, a.map2))
        nx = inst.without_mutex()
        sol = solve_exact(inst)
        for g, stops in sol.stops.items():
            for st in stops:
                d0 = stop_duration(nx, g, st['tasks'], st['pose'])[0]
                rows.append((bin(st['tasks']).count('1'), st['dur'], d0))
    r = np.array(rows)
    paid = int((r[:, 1] > r[:, 2] + 1e-9).sum())
    print(f'Q3 {len(r)} stops on the optimal path, '
          f'{r[:, 0].mean():.2f} tasks per stop (max {r[:, 0].max():.0f}); '
          f'{paid} of them pay any mutex cost')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(q):
        q.add_argument('--seed', type=int, default=0)
        q.add_argument('--n-tasks', type=int, default=4)
        q.add_argument('--n-mr', type=int, default=0)
        q.add_argument('--gantries', type=int, nargs='+', default=[1, 2])
        q.add_argument('--t-fold', type=float, default=0.0,
                       help='additive fold+unfold per traverse; NEVER MEASURED '
                            '(p1_g7 A4.3), default 0.0')
        q.add_argument('--map1', default='/tmp/cap_g1_rail160.npz')
        q.add_argument('--map2', default='/tmp/cap_g2_rail160.npz')

    g = sub.add_parser('gen'); common(g); g.set_defaults(fn=cmd_gen)
    s = sub.add_parser('solve'); common(s); s.set_defaults(fn=cmd_solve)
    w = sub.add_parser('sweep'); common(w)
    w.add_argument('--tasks', type=int, nargs='+', default=[2, 3, 4, 5, 6])
    w.set_defaults(fn=cmd_sweep)
    b = sub.add_parser('ablate'); common(b)
    b.add_argument('--seeds', type=int, nargs='+', default=list(range(10)))
    b.set_defaults(fn=cmd_ablate)
    d = sub.add_parser('diag'); common(d)
    d.add_argument('--seeds', type=int, nargs='+', default=list(range(8)))
    d.set_defaults(fn=cmd_diag)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    raise SystemExit(main())
