#!/usr/bin/env python3
"""Is the gantry-gantry collision model right? docs/p1_g9_sched3.md A3-K1.

Same rule as test/verify_sched_exact.py: nothing here reuses the machinery it
is judging. The reference side knows the bodies as SETS OF POINTS and nothing
else -- no distance primitive, no SAT, no half-extent algebra.

  W0   sched_coll.pair_distance vs a dense BOUNDARY-SAMPLING oracle, plus
       soundness of the (N1)/(N2) prefilter
  W0b  sched_coll.plate_centres vs irm_sweep.base_pose -- the URDF chain that
       was already checked against pinocchio to 1e-16 (p1_g2 1). This is what
       ties the predicate to the same transform the capability map was built
       with, instead of to a second transform that merely looks like it.
  W0c  Lemma 1 (safe parking at rot = 0) and Lemma 2 (non-vacuity) of A2.2,
       checked as numbers rather than trusted as algebra.
       ADDED BEYOND A3-K1, the way g7 added V4: A2.2 states both lemmas as
       proofs, and a proof that is never executed is a proof nobody read.

  W1, W2, W2b, W3, W4 -- added in later steps of A7.

Run:  python3 test/verify_sched_coll.py
Exit: 0 all pass, 1 any failure.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reachability_gng import sched                         # noqa: E402
from reachability_gng import sched_coll as sc              # noqa: E402
from reachability_gng.irm_sweep import base_pose           # noqa: E402

# boundary sample spacing, and the tolerance it justifies. The closest pair of
# points lies on the two boundaries; sampling them at spacing h puts a sample
# within h/2 of each, so the sampled distance overshoots by at most h.
H = 0.0015
TOL_DIST = 2 * H
TOL = 1e-9          # time tolerance, same as g7 A2-K4; not negotiable


# ---------------------------------------------------------------------------
# reference body: a SET OF POINTS. No distance code, no SAT, no half-extents.
# ---------------------------------------------------------------------------
def ref_boundary(g, lin, rot, h=H):
    """Dense sample of the footprint BOUNDARY, built from the URDF text."""
    y = 0.36 if g == 1 else -0.36
    c = np.array([lin, y])
    u = np.array([np.cos(rot), np.sin(rot)])
    v = np.array([-np.sin(rot), np.cos(rot)])

    pts = []
    corners = [c + 0.40 * u + 0.04 * v, c + 0.40 * u - 0.04 * v,
               c - 0.40 * u - 0.04 * v, c - 0.40 * u + 0.04 * v]
    for a, b in zip(corners, corners[1:] + corners[:1]):
        n = max(int(np.linalg.norm(b - a) / h) + 1, 2)
        t = np.linspace(0.0, 1.0, n)[:, None]
        pts.append(a + t * (b - a))
    for centre in (c + 0.40 * u, c - 0.40 * u):
        n = max(int(2 * np.pi * 0.055 / h) + 1, 8)
        th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
        pts.append(centre + 0.055 * np.stack([np.cos(th), np.sin(th)], -1))
    return np.concatenate(pts, axis=0)


def ref_inside(g, lin, rot, p):
    """Is point p in the body? Set membership, written from the URDF text."""
    y = 0.36 if g == 1 else -0.36
    c = np.array([lin, y])
    u = np.array([np.cos(rot), np.sin(rot)])
    v = np.array([-np.sin(rot), np.cos(rot)])
    d = p - c
    in_bar = (np.abs(d @ u) <= 0.40) & (np.abs(d @ v) <= 0.04)
    in_a = np.linalg.norm(p - (c + 0.40 * u), axis=-1) <= 0.055
    in_b = np.linalg.norm(p - (c - 0.40 * u), axis=-1) <= 0.055
    return in_bar | in_a | in_b


def ref_distance(lin1, rot1, lin2, rot2):
    """Brute-force distance between the two point sets.

    Containment cannot happen here and that is not assumed, it is structural:
    the two rotation axes are 0.72 m apart and the bodies reach only 0.455 m,
    so neither centre is ever inside the other body. Overlap therefore always
    shows up as a boundary point of one body lying inside the other.
    """
    b1 = ref_boundary(1, lin1, rot1)
    b2 = ref_boundary(2, lin2, rot2)
    if ref_inside(2, lin2, rot2, b1).any() or ref_inside(1, lin1, rot1, b2).any():
        return 0.0
    d = np.linalg.norm(b1[:, None, :] - b2[None, :, :], axis=-1)
    return float(d.min())


# ---------------------------------------------------------------------------
# cases
# ---------------------------------------------------------------------------
def _cases():
    """Structured cases first, then random. Both are reported separately."""
    out = []
    # the regime A2.2 (N1) says is the only one that can touch at all
    for r1 in (-90.0, -80.0, -60.0, 90.0, 75.0, 45.0, 0.0, 180.0):
        for r2 in (90.0, 80.0, 60.0, -90.0, -75.0, -45.0, 0.0, 180.0):
            for dl in (0.0, 0.05, 0.10, 0.15, 0.30, 0.60):
                out.append(('structured', 0.80, np.deg2rad(r1),
                            0.80 + dl, np.deg2rad(r2)))
    rng = np.random.default_rng(9)
    for _ in range(400):
        out.append(('random',
                    float(rng.uniform(0.0, 1.6)),
                    float(rng.uniform(-np.pi, np.pi)),
                    float(rng.uniform(0.0, 1.6)),
                    float(rng.uniform(-np.pi, np.pi))))
    # random cases restricted to the near-contact regime, where the two
    # implementations are most likely to disagree
    for _ in range(400):
        l1 = float(rng.uniform(0.0, 1.6))
        s = float(rng.uniform(-0.25, 0.25))
        r1 = float(rng.choice([-1.0, 1.0]) * rng.uniform(np.pi / 3, np.pi / 2))
        r2 = float(rng.choice([-1.0, 1.0]) * rng.uniform(np.pi / 3, np.pi / 2))
        out.append(('near', l1, r1, float(np.clip(l1 + s, 0.0, 1.6)), r2))
    return out


def w0_distance():
    worst = {'structured': 0.0, 'random': 0.0, 'near': 0.0}
    count = {'structured': 0, 'random': 0, 'near': 0}
    bad, unsound, ambiguous = [], 0, 0
    for tag, l1, r1, l2, r2 in _cases():
        got = float(sc.pair_distance(l1, r1, l2, r2))
        ref = ref_distance(l1, r1, l2, r2)
        e = abs(got - ref)
        count[tag] += 1
        worst[tag] = max(worst[tag], e)
        if e > TOL_DIST:
            bad.append((tag, l1, r1, l2, r2, got, ref))
        for c in (0.0, 0.05, 0.10):
            if ref <= c and not bool(sc.may_block(l1, r1, l2, r2, c)):
                unsound += 1
            if abs(ref - c) <= TOL_DIST:
                ambiguous += 1
    n = sum(count.values())
    print(f'W0 pair_distance vs boundary-sampling oracle: {n} cases '
          f'(structured {count["structured"]}, random {count["random"]}, '
          f'near-contact {count["near"]}), {len(bad)} mismatches > {TOL_DIST}')
    for k in ('structured', 'random', 'near'):
        print(f'     max |diff| {k:<10} {worst[k]:.3e} m')
    print(f'     (N1)/(N2) prefilter unsound on {unsound} of {3 * n} '
          f'(case, c_clear) pairs -- must be 0')
    print(f'     {ambiguous} of {3 * n} sit within the sampling tolerance of a '
          f'threshold and so cannot decide the boolean either way')
    for b in bad[:5]:
        print(f'     {b[0]} lin {b[1]:.3f}/{b[3]:.3f} rot '
              f'{np.degrees(b[2]):7.2f}/{np.degrees(b[4]):7.2f}: '
              f'closed {b[5]:.6f} vs sampled {b[6]:.6f}')
    return not bad and unsound == 0


def w0b_plate_centres():
    """Plate centres must be the arm bases irm_sweep already validated.

    gantry 1 carries arm1 (right plate, y_p = -0.4) and arm2 (left); gantry 2
    carries arm3 and arm4. sched_coll orders its plates as (+0.40 u, -0.40 u),
    which must land on (right, left).
    """
    rng = np.random.default_rng(11)
    lin = rng.uniform(0.0, 1.6, 500)
    rot = rng.uniform(-np.pi, np.pi, 500)
    worst = 0.0
    for g, arms in ((1, ('arm1', 'arm2')), (2, ('arm3', 'arm4'))):
        got = sc.plate_centres(g, lin, rot)
        for k, arm in enumerate(arms):
            _, p = base_pose(arm, lin, rot)
            worst = max(worst, float(np.abs(got[:, k, :] - p[:, :2]).max()))
    print(f'W0b plate centres vs irm_sweep.base_pose (pinocchio-checked URDF '
          f'chain): 2000 poses, max |diff| {worst:.3e} m')
    return worst < 1e-12


def w0c_lemmas():
    """A2.2 Lemma 1 and Lemma 2, executed instead of trusted."""
    lin = np.arange(33) * 0.05
    rot = np.deg2rad(np.arange(-180, 180, 5))
    L1, R1 = np.meshgrid(lin, rot, indexing='ij')
    ok = True

    # Lemma 1: any pose of one gantry is compatible with any rot = 0 pose of
    # the other, with a margin the lemma puts at 0.21 m.
    margin = np.inf
    for l2 in lin:
        d = sc.pair_distance(L1.ravel(), R1.ravel(), l2, 0.0)
        margin = min(margin, float(d.min()))
    lemma1 = margin > 0.2
    ok &= lemma1
    print(f'W0c Lemma 1 (safe parking at rot = 0): min clearance over all '
          f'{L1.size} x 33 pose pairs = {margin:.4f} m  '
          f'(A2.2 says 0.21) -> {"PASS" if lemma1 else "FAIL"}')

    # Lemma 2: rot1 = -90, rot2 = +90.
    #
    # A2.2 predicted the threshold |dlin| <= 0.11 and that is WRONG -- it is
    # the (N2) NECESSARY bound (h_x + h_x = 0.055 + 0.055) used as if it were
    # sufficient. (N2) only says the x-extents must overlap; it says nothing
    # about whether the parts that overlap in x also meet in y.
    #
    # The correct threshold, derived here and checked against the predicate:
    # at rot = -+90 the plates sit at y = -+0.04, so the binding feature pair
    # is one gantry's PLATE against the other's BAR FLANK, giving
    #     |dlin| <= BAR_HALF_WID + PLATE_R = 0.04 + 0.055 = 0.095
    # (plate-vs-plate needs sqrt(dlin^2 + 0.08^2) <= 0.11, i.e. |dlin| <=
    # 0.0756, which is slacker; bar-vs-bar needs |dlin| <= 0.08.)
    # The SUBSTANTIVE claim of Lemma 2 -- the predicate is not vacuous -- is
    # unaffected, and is what is asserted. See p1_g9 B, contradiction 1.
    thr = sc.BAR_HALF_WID + sc.PLATE_R
    d = np.array([float(sc.pair_distance(0.8, -np.pi / 2, 0.8 + dl, np.pi / 2))
                  for dl in lin])
    blk = d <= 0.0
    want = lin <= thr + 1e-12
    lemma2 = bool((blk == want).all()) and blk.any()
    ok &= lemma2
    print(f'W0c Lemma 2 (non-vacuity): at rot = -90/+90, BLOCK holds for '
          f'dlin in {sorted(lin[blk].round(3).tolist())} m; closed-form '
          f'threshold {thr:.3f} m predicts '
          f'{sorted(lin[want].round(3).tolist())} -> '
          f'{"PASS" if lemma2 else "FAIL"}')
    print(f'     A2.2 predicted 0.110 m and that was the (N2) NECESSARY bound '
          f'misused as sufficient; measured boundary is '
          f'{lin[blk].max():.3f} < dlin <= {lin[~blk].min():.3f} on the grid')

    # (N1) as an angle: a pair can only touch if BOTH are > 31.7 deg off
    # rail-parallel. Checked against the predicate, not against the algebra.
    s = np.abs(np.sin(rot))
    lo = s[s >= 0.525].min() if (s >= 0.525).any() else np.nan
    print(f'W0c (N1) says |sin rot| >= 0.525 (31.7 deg) is necessary for both '
          f'gantries; smallest grid rotation meeting it is '
          f'{np.degrees(np.arcsin(lo)):.1f} deg')
    return ok


# ---------------------------------------------------------------------------
# W1 -- the A2.4 certificate
# ---------------------------------------------------------------------------
def ref_scan(A, B, t_lo, t_hi, n, c_clear):
    """Dense uniform scan. Knows nothing about conservative advancement.

    Returns (blocked?, min distance seen). Deliberately dumb: this is the
    thing `first_block` has to survive.
    """
    t = np.linspace(t_lo, t_hi, n)
    l1, r1 = A.pose_at(t)
    l2, r2 = B.pose_at(t)
    d = sc.pair_distance(l1, r1, l2, r2)
    return bool((d <= c_clear).any()), float(d.min())


def _rand_traj(rng, g, n_legs):
    """A trajectory with random legs, starting at a random pose."""
    p0 = (float(rng.uniform(0.0, 1.6)), float(rng.uniform(-np.pi, np.pi)))
    tr = sc.Traj(g, p0)
    t = float(rng.uniform(0.0, 2.0))
    for _ in range(n_legs):
        q = (float(rng.uniform(0.0, 1.6)), float(rng.uniform(-np.pi, np.pi)))
        t = tr.append(t, q) + float(rng.uniform(0.0, 3.0))
    return tr


def _near_traj(rng, g, lin_c):
    """A trajectory confined to the regime (N1) says is the only dangerous one."""
    sgn = 1.0 if g == 1 else -1.0
    p0 = (float(np.clip(lin_c + rng.uniform(-0.2, 0.2), 0, 1.6)),
          float(-sgn * rng.uniform(np.pi / 3, np.pi / 2)))
    tr = sc.Traj(g, p0)
    t = 0.0
    for _ in range(2):
        q = (float(np.clip(lin_c + rng.uniform(-0.2, 0.2), 0, 1.6)),
             float(-sgn * rng.uniform(np.pi / 3, np.pi / 2)))
        t = tr.append(t, q) + float(rng.uniform(0.0, 1.0))
    return tr


def w1_certificate():
    ok = True

    # (a) the trajectory must reproduce the LOCKED duration exactly, and land
    #     on its endpoints. If A2.3 drifted from p1_state 5.6, everything
    #     downstream is measuring a different model than G7/G8 did.
    from reachability_gng.sched import traverse_time
    rng = np.random.default_rng(3)
    worst_T = worst_end = 0.0
    for _ in range(3000):
        p = (float(rng.uniform(0, 1.6)), float(rng.uniform(-np.pi, np.pi)))
        q = (float(rng.uniform(0, 1.6)), float(rng.uniform(-np.pi, np.pi)))
        T = sc.leg_duration(p, q)
        ref = float(traverse_time(q[0] - p[0], q[1] - p[1]))
        worst_T = max(worst_T, abs(T - ref))
        l, r = sc.leg_pose(np.array([0.0, T]), p, q)
        worst_end = max(worst_end, abs(l[0] - p[0]), abs(l[1] - q[0]),
                        abs(float(sc.short_deg(r[0] - p[1]))),
                        abs(float(sc.short_deg(r[1] - q[1]))))
    a_ok = worst_T < 1e-12 and worst_end < 1e-9
    ok &= a_ok
    print(f'W1a traverse duration vs sched.traverse_time (LOCKED p1_state 5.6): '
          f'3000 legs, max |diff| {worst_T:.3e} s; endpoint error '
          f'{worst_end:.3e} -> {"PASS" if a_ok else "FAIL"}')

    # (b) soundness: a dense scan must never find a block that first_block
    #     missed. 10x finer than the certificate's own worst-case step, plus
    #     random times.
    rng = np.random.default_rng(5)
    pairs = []
    for _ in range(100):
        pairs.append((_rand_traj(rng, 1, 2), _rand_traj(rng, 2, 2)))
    for _ in range(150):
        lc = float(rng.uniform(0.2, 1.4))
        pairs.append((_near_traj(rng, 1, lc), _near_traj(rng, 2, lc)))

    missed, conservative, worst_gap, n_block, dense = 0, 0, 0.0, 0, 0
    for A, B in pairs:
        t_hi = max(A.end_time(), B.end_time())
        ca = sc.first_block(A, B, 0.0, t_hi)
        # 10x finer than the certificate's own worst-case step (A2.4), which
        # is what "10x-dense" has to mean for the comparison to bite.
        n_uni = min(max(int(t_hi * sc.V_REL / sc.EPS_CERT * 10) + 2, 200), 20000)
        dense = max(dense, n_uni)
        ref_blk, dmin = ref_scan(A, B, 0.0, t_hi, n_uni, sc.C_CLEAR)
        tr = rng.uniform(0.0, t_hi, 5000) if t_hi > 0 else np.zeros(1)
        l1, r1 = A.pose_at(tr)
        l2, r2 = B.pose_at(tr)
        d2 = sc.pair_distance(l1, r1, l2, r2)
        ref_blk |= bool((d2 <= sc.C_CLEAR).any())
        dmin = min(dmin, float(d2.min()))
        if ref_blk:
            n_block += 1
            if ca is None:
                missed += 1
        elif ca is not None:
            conservative += 1
            worst_gap = max(worst_gap, dmin)
    b_ok = missed == 0 and worst_gap <= sc.EPS_CERT + 1e-9
    ok &= b_ok
    print(f'W1b certificate vs dense scan (up to {dense} pts, 10x the '
          f'certificate step) + 5e3 random times per pair: '
          f'{len(pairs)} trajectory pairs, {n_block} genuinely blocked, '
          f'{missed} MISSED by the certificate -> {"PASS" if b_ok else "FAIL"}')
    print(f'     conservative calls (certificate says blocked, dense scan does '
          f'not): {conservative}; largest true clearance so rejected '
          f'{worst_gap:.5f} m, must be <= eps = {sc.EPS_CERT}')

    # (c) a hand-computed case: gantry 2 parked at (0.8, +90 deg), gantry 1
    #     rotating -45 -> -135 deg at lin = 0.8. Gantry 1 passes through
    #     rot = -90, where W0c measured dlin = 0 as blocked, so a block is
    #     certain, and its time follows from A2.3 alone.
    A = sc.Traj(1, (0.8, np.deg2rad(-45.0)))
    A.append(0.0, (0.8, np.deg2rad(-135.0)))
    B = sc.Traj(2, (0.8, np.deg2rad(90.0)))
    t = sc.first_block(A, B, 0.0, A.end_time())
    # rot reaches -90 after the 0.26 s dead time plus 45 deg at 10 deg/s
    t_reach = 0.26 + 45.0 / 10.0
    c_ok = t is not None and t <= t_reach + 1e-6
    ok &= c_ok
    print(f'W1c hand-computed sweep: gantry 1 rotates through -90 deg past a '
          f'parked gantry 2; certificate first blocks at t = {t:.4f} s, '
          f'A2.3 says it must be <= {t_reach:.4f} s -> '
          f'{"PASS" if c_ok else "FAIL"}')

    # (d) Lemma 1 again, but in TIME: a gantry parked at rot = 0 can never be
    #     hit, no matter what the other one does.
    rng = np.random.default_rng(13)
    hits = 0
    for _ in range(200):
        A = _rand_traj(rng, 1, 3)
        B = sc.Traj(2, (float(rng.uniform(0, 1.6)), 0.0))
        if sc.first_block(A, B, 0.0, A.end_time()) is not None:
            hits += 1
    d_ok = hits == 0
    ok &= d_ok
    print(f'W1d Lemma 1 in time: 200 arbitrary gantry-1 trajectories vs a '
          f'gantry 2 parked at rot = 0, {hits} blocks -> '
          f'{"PASS" if d_ok else "FAIL"}')
    return ok


# ---------------------------------------------------------------------------
# W2b / W3 / W4 -- the coupled solver
# ---------------------------------------------------------------------------
def w2b_collision_off():
    """Turn the collision off; the coupled search must reproduce G7 exactly.

    This is the one check that exercises the ENTIRE new search -- split lower
    bound, action enumeration, goal test, makespan accounting -- at the real
    |P| = 2376, against an oracle already proved exact by V0-V4. The Lemma 4
    shortcut is forced off and the incumbent is seeded 2 s ABOVE the true
    optimum, so the search cannot pass by accepting what it was handed.
    """
    bad, checked = [], 0
    for n, mr in [(4, 0), (4, 1), (6, 0), (6, 1)]:
        for seed in range(3):
            inst = sched.gen_real(n, seed, mr, (1, 2))
            ref = sched.solve_exact(inst).makespan
            got = sc.solve_coupled(inst, c_clear=-1.0, force_bnb=True,
                                   ub_seed=ref + 2.0, time_budget=90.0)
            checked += 1
            if abs(got.makespan - ref) > TOL or not got.proved:
                bad.append((n, mr, seed, got.makespan, ref, got.proved))
    print(f'W2b coupled search with BLOCK disabled vs sched.solve_exact: '
          f'{checked} real instances, |P| = 2376, {len(bad)} mismatches')
    for b in bad[:5]:
        print(f'     n={b[0]} mr={b[1]} seed={b[2]}: coupled {b[3]:.6f} vs '
              f'exact {b[4]:.6f} (proved={b[5]})')
    return not bad


def _synth2(poses, r1_idx, r2_idx, p0=0):
    """Two gantries, one task each, each feasible at exactly one pose."""
    P = np.asarray(poses, float)
    n, K = 2, len(P)
    r1 = np.zeros((n, K, 2), bool)
    r2 = np.zeros((n, K, 2), bool)
    r1[0, r1_idx, 0] = True          # task 0 only doable by gantry 1
    r2[1, r2_idx, 0] = True          # task 1 only doable by gantry 2
    z = np.zeros((n, K, 2), bool)
    h = np.zeros((n, K), bool)
    return sched.gen_synthetic(['SR', 'SR'], {1: P, 2: P}, {1: r1, 2: r2},
                               {1: z, 2: z}, {1: h, 2: h}, {1: p0, 2: p0},
                               gantries=(1, 2))


def w3_pathological():
    """Values fixed in p1_g9 A3-K1b BEFORE the solver existed."""
    ok = True
    d90 = np.pi / 2

    # Q1 -- both gantries pinned to rot = 0, so (N1) makes collision
    # algebraically impossible: the coupled answer must EQUAL the uncoupled one.
    inst = _synth2([[0.0, 0.0], [0.8, 0.0]], 1, 1)
    s = sc.solve_coupled(inst)
    ref = sched.solve_exact(inst).makespan
    q1 = abs(s.makespan - ref) <= TOL
    ok &= q1
    print(f'Q1 collision impossible (both at rot = 0): coupled {s.makespan:.4f} '
          f'== uncoupled {ref:.4f} -> {"PASS" if q1 else "FAIL"}')

    # Q5 -- Lemma 1 again: rot = 0 against rot = 180 can never touch.
    inst = _synth2([[0.0, 0.0], [0.8, 0.0], [0.8, np.pi]], 1, 2)
    s = sc.solve_coupled(inst)
    ref = sched.solve_exact(inst).makespan
    q5 = abs(s.makespan - ref) <= TOL
    ok &= q5
    print(f'Q5 safe parking (rot = 0 vs rot = 180): coupled {s.makespan:.4f} '
          f'== uncoupled {ref:.4f} -> {"PASS" if q5 else "FAIL"}')

    # Q2 -- the P4 of this session. Both tasks live at mutually blocking poses
    # (rot = -90 / +90 at the same lin, which W0c measured as blocking at
    # dlin = 0). The exact optimum needs an evasive move and is not worth
    # hand-deriving, but two closed-form facts are:
    #   Lemma 3   coupled >= uncoupled = T(0 -> 90 deg) + dwell = 11.26
    #   disjoint  the two dwells CANNOT overlap, since the only poses that can
    #             do them block each other, so coupled >= 9.26 + 2 + 2 = 13.26
    inst = _synth2([[0.8, 0.0], [0.8, -d90], [0.8, d90]], 1, 2)
    s = sc.solve_coupled(inst, time_budget=60.0)
    ref = sched.solve_exact(inst).makespan
    floor = (0.26 + 90.0 / 10.0) + 2 * sched.DWELL
    q2 = s.makespan >= floor - TOL and s.makespan > ref + TOL
    ok &= q2
    print(f'Q2 mutually locking poses: uncoupled {ref:.4f}, coupled '
          f'{s.makespan:.4f}, hand-derived floor {floor:.4f} '
          f'(dwells cannot overlap) -> {"PASS" if q2 else "FAIL"}')
    print(f'     evasive moves used: {s.n_evade}; route {s.route}; '
          f'proved={s.proved}')

    # Q4 -- Lemma 3 as an invariant on real instances. Cheap, and it is the
    # one check that runs on EVERY instance the session measures.
    neg = []
    for n, mr, seed in [(4, 0, 0), (4, 1, 1), (4, 0, 2), (6, 0, 0), (6, 1, 3)]:
        i2 = sched.gen_real(n, seed, mr, (1, 2))
        s2 = sc.solve_coupled(i2, time_budget=60.0)
        if s2.makespan < s2.lb - TOL:
            neg.append((n, mr, seed, s2.makespan, s2.lb))
    ok &= not neg
    print(f'Q4 Lemma 3 invariant (coupled >= uncoupled) on 5 real instances: '
          f'{len(neg)} violations -> {"PASS" if not neg else "FAIL"}')
    return ok


def w4_replay():
    """Replay reported coupled schedules against BOTH rule sets.

    Intra-gantry rules come from verify_sched_exact.validate_schedule -- the
    same function, unmodified, that G7 V4 and the whole of G8 K4 used. The
    collision rule is replayed separately at eps/10, ten times tighter than the
    solver's own certificate: a gate as loose as the thing it judges is not a
    gate.
    """
    from verify_sched_exact import validate_schedule
    bad, checked, blocked = [], 0, 0
    for n, mr in [(4, 0), (4, 1), (6, 0), (6, 1)]:
        for seed in range(3):
            inst = sched.gen_real(n, seed, mr, (1, 2))
            s = sc.solve_coupled(inst, time_budget=60.0)
            if not np.isfinite(s.makespan) or not s.stops:
                bad.append((n, mr, seed, ['no schedule returned']))
                continue
            checked += 1
            errs = validate_schedule(inst, s)
            hit = sc.schedule_conflict(inst, s.stops, sc.C_CLEAR,
                                       sc.EPS_CERT / 10.0)
            if hit is not None:
                blocked += 1
                errs = errs + [f'A2.4 violated at t = {hit:.4f}']
            if errs:
                bad.append((n, mr, seed, errs[:3]))
    print(f'W4 reported coupled schedules replayed (intra-gantry rules via the '
          f'UNMODIFIED validate_schedule, collision at eps/10): {checked} '
          f'schedules, {len(bad)} with violations, {blocked} colliding')
    for b in bad[:5]:
        print(f'     n={b[0]} mr={b[1]} seed={b[2]}: {b[3]}')
    return not bad


def main():
    checks = [('W0', w0_distance), ('W0b', w0b_plate_centres),
              ('W0c', w0c_lemmas), ('W1', w1_certificate),
              ('W2b', w2b_collision_off), ('W3', w3_pathological),
              ('W4', w4_replay)]
    results = []
    for tag, fn in checks:
        results.append((tag, fn()))
        print()
    print('=' * 62)
    for tag, ok in results:
        print(f'{tag}: {"PASS" if ok else "FAIL"}')
    allok = all(ok for _, ok in results)
    print('=' * 62)
    print('COUPLED GROUND TRUTH ESTABLISHED (w.r.t. the A3-K1 candidate sets)'
          if allok else
          'NOT ESTABLISHED -- p1_state 7.1 forbids touching heuristics')
    return 0 if allok else 1


if __name__ == '__main__':
    raise SystemExit(main())
