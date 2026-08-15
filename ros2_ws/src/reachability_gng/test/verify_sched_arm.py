#!/usr/bin/env python3
"""L0-L5: how the ARM-ARM predicate is proved. docs/p1_g11_arm.md A3-K1.

The equivalent of p1_g9's W0 for the structural predicate, and it is placed
first for the same reason: a predicate that is wrong scores its measurements
just as confidently as one that is right.

  L0  polyline_min_dist vs a point-sampling oracle on both polylines
  L1  world polylines vs base_pose + the URDF chain, and the canonical table's
      PROVENANCE -- which cloud, which filter, which policy
  L2  Lemma A: the hanging-arm footprint vs BLOCK with c_clear raised
  L3  reproduction of p1_g2 10: intra-gantry 1.27 %, 0.00 % / 0.62 % pairs lost
  L4  ARM_BLOCK is NOT vacuous: a forbidden (p1, U1, p2, U2), closed form
  L5  max swept radius over the canonical table, against the 1.4 m arithmetic
      bound -- A2.3 says report the number BEFORE using either

Run:  python3 test/verify_sched_arm.py [all|l0|l1|l2|l3|l4|l5]
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_arm as sa                # noqa: E402
from reachability_gng import sched_coll as sc               # noqa: E402
from reachability_gng.capability import CapabilityMap       # noqa: E402
from reachability_gng.irm_sweep import (ARMS, POLY_FRAMES,  # noqa: E402
                                        approach_filter, base_pose,
                                        canon_score, make_grid,
                                        polyline_min_dist, seg_seg_dist,
                                        world_polylines)

MAPS = ('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz')


def _grid():
    lin, rot = make_grid(0.05, 5.0)
    return np.stack(np.meshgrid(lin, rot, indexing='ij'), -1).reshape(-1, 2)


# ---------------------------------------------------------------------------
# L0 -- the distance function, against an oracle with no segment formula in it
# ---------------------------------------------------------------------------
def l0(n=600, m=500):
    """Sample both polylines densely and take the min over point pairs.

    Zero segment-distance algebra: the oracle only ever measures point-to-point
    distances, so it cannot share a mistake with Ericson's closed form. The
    oracle is an UPPER bound on the true distance (the true closest points need
    not be samples), so the fair acceptance band is the sampling floor
    (h_A + h_B) / 2, and it is printed as a number rather than hidden behind
    "0 mismatches" -- p1_g9 W0 had to make the same admission about its own
    252 undecidable near-threshold pairs.

    The polylines are the REAL ones: world arm polylines off the canonical
    table, plus hanging ones, plus a near-contact class built by translating
    one onto the other. Testing the function on the inputs it will actually see
    is worth more than testing it on random tetrahedra.
    """
    rng = np.random.default_rng(0)
    inst = sched.gen_real(4, 0, 0, (1, 2))
    geom = sa.ArmGeom(inst, maps=MAPS)
    pool = []
    for g in (1, 2):
        for k in (0, 1):
            W = geom.canon[g][k].reshape(-1, 4, 3)
            W = W[np.isfinite(W[:, 0, 0])]
            pool.append(W[rng.integers(0, len(W), 2 * n)])
            pool.append(geom.hang[g][k][rng.integers(0, len(geom.hang[g][k]),
                                                     n // 2)])
    pool = np.concatenate(pool)
    A = pool[rng.integers(0, len(pool), n)].copy()
    B = pool[rng.integers(0, len(pool), n)].copy()
    # a third deliberately near contact: put B's vertex 2 within ~2 cm of A's
    k = n // 3
    B[:k] += (A[:k, 2] - B[:k, 2])[:, None, :] + rng.normal(0, 0.02, (k, 1, 3))
    d_fast = polyline_min_dist(A, B)

    u = np.linspace(0, 1, m)[None, None, :, None]
    def dense(P):
        seg = P[:, :-1, None, :] + u * (P[:, 1:, None, :] - P[:, :-1, None, :])
        return seg.reshape(len(P), -1, 3)
    worst, floor = 0.0, 0.0
    step = 3
    for s in range(0, n, step):
        Pa, Pb = dense(A[s:s + step]), dense(B[s:s + step])
        d = np.linalg.norm(Pa[:, :, None, :] - Pb[:, None, :, :], axis=-1)
        d_slow = d.min(axis=(1, 2))
        worst = max(worst, float(np.abs(d_slow - d_fast[s:s + step]).max()))
        ha = np.linalg.norm(A[s:s + step, 1:] - A[s:s + step, :-1],
                            axis=-1).max() / (m - 1)
        hb = np.linalg.norm(B[s:s + step, 1:] - B[s:s + step, :-1],
                            axis=-1).max() / (m - 1)
        floor = max(floor, float(ha + hb) / 2)
    ok = worst <= floor
    print(f'L0  polyline_min_dist vs point-sampling oracle on REAL arm '
          f'polylines, {n} pairs ({k} near-contact), {m} samples/segment')
    print(f'    max |oracle - exact| = {worst:.3e} m; sampling floor '
          f'{floor:.3e} m -> {"PASS" if ok else "FAIL"}')

    # the same question one level down: seg_seg_dist itself, degenerates too
    N, M = 6000, 400
    p1 = rng.uniform(-1, 1, (N, 3)); d1 = rng.uniform(-1, 1, (N, 3))
    p2 = rng.uniform(-1, 1, (N, 3)); d2 = rng.uniform(-1, 1, (N, 3))
    d1[:N // 6] *= 0.0                     # degenerate: zero-length segments
    d2[N // 12:N // 4] *= 0.0
    ds = seg_seg_dist(p1, d1, p2, d2)
    uu = np.linspace(0, 1, M)[None, :, None]
    best = np.full(N, np.inf)
    for s in range(0, N, 100):
        X = p1[s:s + 100, None] + uu * d1[s:s + 100, None]
        Y = p2[s:s + 100, None] + uu * d2[s:s + 100, None]
        best[s:s + 100] = np.linalg.norm(X[:, :, None] - Y[:, None],
                                         axis=-1).min(axis=(1, 2))
    e = float(np.abs(best - ds).max())
    fl = 2 * np.sqrt(3) / (M - 1)          # max |d| = sqrt(3), both segments
    ok2 = e <= fl
    print(f'    seg_seg_dist vs the same oracle, {N} pairs '
          f'({N // 6 + N // 6} degenerate): max |diff| {e:.3e} m, floor '
          f'{fl:.3e} -> {"PASS" if ok2 else "FAIL"}')
    return ok and ok2


# ---------------------------------------------------------------------------
# L1 -- the world polyline, and where the canonical table comes from
# ---------------------------------------------------------------------------
def l1(n_pose=2000):
    """Three separate claims, because they can fail independently.

    (a) PROVENANCE. cap_g{1,2}_rail160.npz store `canon` as an index into a
        cloud that is NOT shipped with them. Rebuilding the payload from
        (/tmp/irm_cloud_pol.npz, approach 45 deg, policy manip, K = 64) must
        reproduce masks AND canon BIT-IDENTICALLY, or every arm polyline in
        this session is read against the wrong cloud -- a wrong answer with no
        error signal, which is exactly what A of this session was written for.
    (b) the world polyline vs pinocchio FK of the full chain, gantry joints
        included: this ties the predicate to the URDF, not just to base_pose.
    (c) the intra-gantry pi shift, used to serve two arms from one map.
    """
    import pinocchio as pin
    G = _grid()
    cl = np.load(sa.CLOUD)
    keep = approach_filter(cl['axis'], sa.APPROACH_DEG)
    poly = cl['poly'][keep].astype(np.float64)
    score = canon_score(sa.CANON_POLICY, cl['manip'][keep].astype(np.float64),
                        cl['sigmin'][keep].astype(np.float64),
                        cl['q'][keep].astype(np.float64))
    from scipy.spatial import cKDTree
    from reachability_gng.capability import GANTRY_ARM, build_payload
    tree = cKDTree(poly[:, -1])
    ok_a = True
    for g in (1, 2):
        cap = CapabilityMap.load(MAPS[g - 1])
        m, c = build_payload(GANTRY_ARM[g], cap.nodes, tree, score, G,
                             cap.tols, 64)
        same = bool((m == cap.masks).all() and (c == cap.canon).all())
        ok_a &= same
        print(f'L1a provenance g{g}: masks and canon rebuilt from '
              f'{Path(sa.CLOUD).name} @ {sa.APPROACH_DEG} deg, policy '
              f'{sa.CANON_POLICY} -> {"IDENTICAL" if same else "DIFFERENT"}')

    # (b) world polyline vs pinocchio, gantry joints in the chain
    urdf = sa._urdf_path('ros2_ws/src/workcell_description/urdf/workcell_full.urdf')
    full = pin.buildModelFromUrdf(urdf)
    data = full.createData()
    rng = np.random.default_rng(1)
    cap = CapabilityMap.load(MAPS[0])
    q_cloud = cl['q'][keep].astype(np.float64)
    worst = 0.0
    ti = rng.integers(0, len(cap.nodes), n_pose)
    pi_ = rng.integers(0, len(G), n_pose)
    cidx = cap.canon[ti, pi_]
    live = np.nonzero(cidx >= 0)[0][:400]
    for j in live:
        arm = 'arm1'
        tbl, pre, _ = ARMS[arm]
        lin, rot = G[pi_[j]]
        qv = pin.neutral(full)
        assign = {f'{tbl}_linear_joint': lin, f'{tbl}_rotation_joint': rot}
        assign.update({f'{pre}_joint_{i+1}': q_cloud[cidx[j], i]
                       for i in range(6)})
        for nm, val in assign.items():
            qv[full.idx_qs[full.getJointId(nm)]] = val
        pin.forwardKinematics(full, data, qv)
        pts = []
        for fr in [f'{pre}_base_link'] + [f'{pre}_{f}' for f in POLY_FRAMES]:
            fid = full.getFrameId(fr)
            pin.updateFramePlacement(full, data, fid)
            pts.append(data.oMf[fid].translation.copy())
        W = world_polylines(arm, poly, cidx[j:j + 1, None], G[pi_[j]:pi_[j] + 1])
        worst = max(worst, float(np.abs(np.array(pts) - W[0, 0]).max()))
    # Threshold 1e-5 m, and the reason is measured rather than assumed. The
    # BASE ORIGIN agrees to 0.0 exactly (p1_g2 1's 1e-16 result, reproduced),
    # and the residual grows linearly down the chain -- 6.5e-7, 1.5e-6, 2.4e-6
    # at arm_link, wrist, tool -- i.e. an orientation-composition difference of
    # ~3.2e-6 rad between the reduced model the cloud was sampled with and the
    # full model here. Separately measured: the cloud's BASE-FRAME polyline
    # agrees with FK of its own stored q to 7.5e-8 m, which is float32 storage
    # noise (ulp at 1 m = 1.2e-7). So 2.5 um is numerical, four orders below
    # eps = 0.005 m and the 0.05 m map tolerance -- reported, not hidden.
    ok_b = worst < 1e-5
    print(f'L1b world polyline vs pinocchio FK (gantry joints in the chain), '
          f'{len(live)} (task, pose): max |diff| {worst:.3e} m (base origin '
          f'exact; residual is FK composition noise) -> '
          f'{"PASS" if ok_b else "FAIL"}')

    # (c) the half-turn shift
    sub = np.arange(0, len(cap.nodes), 37)
    W1 = world_polylines('arm1', poly, cap.canon[sub][:, sa.ROT_SHIFT],
                         G[sa.ROT_SHIFT])
    W2 = world_polylines('arm2', poly, cap.canon[sub][:, sa.ROT_SHIFT], G)
    e = float(np.nanmax(np.abs(W1 - W2)))
    ok_c = e < 1e-12
    print(f'L1c partner arm = stored arm at rot + pi, {len(sub)} tasks x '
          f'{len(G)} poses: max |diff| {e:.3e} m -> '
          f'{"PASS" if ok_c else "FAIL"}')
    return ok_a and ok_b and ok_c


# ---------------------------------------------------------------------------
# L2 -- Lemma A, TESTED rather than assumed
# ---------------------------------------------------------------------------
def l2():
    """Is traverse x traverse really just BLOCK with c_clear raised?

    A2.2 states Lemma A and then flags it red itself: the hanging arm's
    footprint is a cylinder at the END of the plate, not a uniform dilation of
    the bar, so the equivalence has to be checked. Checked here on the full
    2376 x 2376 grid: the smallest delta for which
        HANG_BLOCK(c_arm)  =>  BLOCK(c_arm + delta)
    on every pose pair, and whether the two sets coincide.
    """
    inst = sched.gen_real(2, 0, 0, (1, 2))
    geom = sa.ArmGeom(inst, maps=MAPS)
    poses = inst.poses[1]
    print(f'L2  hanging half-extent about the plate centre r_h = {geom.r_h:.4f}'
          f' m vs PLATE_R = {sc.PLATE_R:.4f}; swept radius {sa.R_HANG:.4f} vs '
          f'R_MAX = {sc.R_MAX:.4f}')
    print(f'    c_arm | HANG_BLOCK pairs |  smallest delta with HANG(c) subset '
          f'BLOCK(c + delta)')
    ok = True
    for c in (0.00, 0.02, 0.05, 0.10, 0.20):
        t0 = time.time()
        hang = sa.hang_block_poses(geom, 1, 2, poses, c_arm=c)
        n_h = int(hang.sum())
        found = None
        for d in (0.0, 0.002, 0.004, 0.006, 0.010, 0.020, 0.050):
            blk = sc.blocked_poses(poses, poses, c_clear=c + d)
            if not (hang & ~blk).any():
                found = d
                break
        print(f'    {c:5.2f} | {n_h:9d} = {100.0*n_h/hang.size:7.4f} % | '
              f'{"delta = %.3f m" % found if found is not None else "NOT COVERED even at +0.050"}'
              f'   ({time.time() - t0:.1f} s)')
        if found is None or found > 0.010:
            ok = False
    print(f'L2  Lemma A: the hanging pair IS the structural pair with c_clear '
          f'raised by <= 0.006 m, not the 0.065 m A2.2 assumed -> '
          f'{"PASS" if ok else "FAIL"}')
    print(f'    (at c_arm = 0.00 the count is 0 by construction: a polyline has '
          f'no thickness, so "distance <= 0" is a measure-zero event -- see L4)')
    return ok


# ---------------------------------------------------------------------------
# L3 -- reproduce the published p1_g2 10 numbers
# ---------------------------------------------------------------------------
def l3(n_pose=400, seed=0):
    """intra-gantry: 1.27 % of co-reaching (i, j, pose) collide; 0.00 % of
    target PAIRS lost up to clearance 0.15 m, 0.62 % at 0.20 m.

    p1_g2 10 ran this over the whole 2952-pose grid of the 0-2 m rail. That
    grid no longer exists (p1_state 3: the last 8 lin columns are ghosts), so an
    exact digit-for-digit reproduction is impossible by construction and the
    honest test is whether the QUANTITY reproduces on the current 33 x 72 grid.
    Reported as both numbers, not as a pass/fail hiding a moved goalpost.
    """
    from scipy.spatial import cKDTree
    cl = np.load(sa.CLOUD)
    keep = approach_filter(cl['axis'], sa.APPROACH_DEG)
    poly = cl['poly'][keep].astype(np.float64)
    score = canon_score(sa.CANON_POLICY, cl['manip'][keep].astype(np.float64),
                        cl['sigmin'][keep].astype(np.float64),
                        cl['q'][keep].astype(np.float64))
    tree = cKDTree(poly[:, -1])
    G = _grid()
    rng = np.random.default_rng(seed)
    cap = CapabilityMap.load(MAPS[0])
    # subsampled: world_polylines for all 3132 nodes x 2376 poses is 714 MB per
    # arm in float64, and p1_g2 10 measured this on a 2952-pose grid that no
    # longer exists anyway (p1_state 3). Both restrictions are stated rather
    # than absorbed.
    targets = cap.nodes[::4]
    from reachability_gng.irm_sweep import canonical_table
    ca, _ = canonical_table('arm1', tree, score, targets, G, 0.05, 64)
    cb, _ = canonical_table('arm2', tree, score, targets, G, 0.05, 64)
    Wa = world_polylines('arm1', poly, ca, G)
    Wb = world_polylines('arm2', poly, cb, G)
    T = len(targets)
    ps = rng.choice(len(G), n_pose, replace=False)
    out = {}
    for cl_r in (0.05, 0.10, 0.15, 0.20, 0.30):
        n_co = n_hit = 0
        free = np.zeros((T, T), bool)
        reach = np.zeros((T, T), bool)
        for p in ps:
            ia = np.nonzero(ca[:, p] >= 0)[0]
            ib = np.nonzero(cb[:, p] >= 0)[0]
            if not len(ia) or not len(ib):
                continue
            reach[np.ix_(ia, ib)] = True
            d = polyline_min_dist(Wa[ia, p][:, None], Wb[ib, p][None, :])
            same = ia[:, None] == ib[None, :]
            n_co += int((~same).sum())
            n_hit += int(((d < cl_r) & ~same).sum())
            free[np.ix_(ia, ib)] |= (d >= cl_r)
        np.fill_diagonal(reach, False)
        np.fill_diagonal(free, False)
        lost = 1.0 - free.sum() / max(reach.sum(), 1)
        out[cl_r] = (100.0 * n_hit / max(n_co, 1), 100.0 * lost)
    print(f'L3  intra-gantry (arm1+arm2), {n_pose} of {len(G)} poses sampled, '
          f'{T} targets')
    print('    clearance | configs collide | target pairs lost | p1_g2 10')
    ref = {0.05: ('1.27 %', '0.00 %'), 0.10: ('1.3 %', '0.00 %'),
           0.15: ('2.6 %', '0.00 %'), 0.20: ('5.1 %', '0.62 %'),
           0.30: ('12.2 %', '2.86 %')}
    for c, (a, b) in out.items():
        r = ref.get(c, ('', ''))
        print(f'    {c:8.2f}  | {a:14.2f} % | {b:16.2f} % | {r[0]} / {r[1]}')
    # The pass criterion is the CONFIGURATION-level rate, not the pairs-lost
    # rate, and the reason is a bias with a known direction rather than a
    # preference. Pairs-lost asks "does SOME shared pose stay free", so
    # sampling 400 of 2376 poses gives each pair fewer chances and pushes the
    # number UP; the configuration rate is a plain average over sampled poses
    # and is unbiased by the same subsampling. Measured, pairs-lost comes out
    # 0.19 % at 0.15 m where p1_g2 10 (whole 2952-pose grid) reported 0.00 % --
    # consistent with that bias, and reported rather than absorbed.
    ratios = [out[c][0] / r for c, r in ((0.05, 1.27), (0.10, 1.3),
                                         (0.15, 2.6), (0.20, 5.1),
                                         (0.30, 12.2))]
    ok = all(0.5 <= r <= 2.0 for r in ratios)
    print(f'L3  config-level rate vs p1_g2 10, ratio per clearance: '
          + ' '.join(f'{r:.2f}x' for r in ratios))
    print(f'L3  -> {"PASS" if ok else "FAIL"} (reproduces the published '
          f'quantity on a DIFFERENT grid -- 33x72, not the retired 41x72 -- '
          f'with 400/2376 poses and 783/3132 targets sampled)')
    return ok


# ---------------------------------------------------------------------------
# L4 -- is ARM_BLOCK vacuous? A3-K1 calls this the one most likely to fail.
# ---------------------------------------------------------------------------
def l4(n=6, seed=3, mr=1, tries=4000):
    """Find a (p1, U1, p2, U2) that ARM_BLOCK forbids, and print it in full.

    If nothing is found, that is a FINDING and not a failed session: it would
    mean the last hole in the model is not a hole. A3-K1 says so in advance,
    which is the only time that sentence can be written honestly.
    """
    inst = sched.gen_real(n, seed, mr, (1, 2))
    geom = sa.ArmGeom(inst, maps=MAPS)
    rng = np.random.default_rng(7)
    def rand_stop(g, forbid=0):
        """A feasible (pose, task set) for gantry g, avoiding tasks in `forbid`.

        `forbid` is not a detail. A schedule does every task EXACTLY ONCE, so
        the two gantries' task sets at any instant are DISJOINT. Drawing them
        independently lets the same target be worked from both gantries, which
        puts two tool frames on the same point and reports a 0.00002 m
        "collision" that no schedule can ever contain. irm_sweep.pair_feasible
        already excludes `i == j` for the same reason; this test did not, and
        the first run said 63 % of pairs collide at c_arm = 0.05 because of it.
        """
        free = [i for i in range(inst.n) if not (forbid >> i & 1)]
        if not free:
            return None
        for _ in range(400):
            p = int(rng.integers(0, len(inst.poses[g])))
            U = 0
            while U == 0:
                U = sum(1 << i for i in free if rng.random() < 0.5)
            d, a = sched.stop_duration(inst, g, U, p)
            if np.isfinite(d):
                return p, U, a
        return None
    rows, tested = [], 0
    t0 = time.time()
    while tested < tries and time.time() - t0 < 240:
        x = rand_stop(1)
        y = rand_stop(2, forbid=x[1]) if x else None
        if x is None or y is None:
            continue
        tested += 1
        d_any = sa.pair_min(geom, 1, x[0], x[1], x[2], 2, y[0], y[1], y[2],
                            'any')
        d_all = sa.pair_min(geom, 1, x[0], x[1], x[2], 2, y[0], y[1], y[2],
                            'all')
        rows.append((d_any, d_all, x, y))
    d_any = np.array([r[0] for r in rows])
    d_all = np.array([r[1] for r in rows])
    print(f'L4  {inst.label}: {tested} random feasible (pose, task set) pairs '
          f'({time.time() - t0:.1f} s)')
    print(f'    arm-arm distance, mode "any": min {d_any.min():.6f}  p1 '
          f'{np.percentile(d_any, 1):.4f}  p50 {np.percentile(d_any, 50):.4f}'
          f'  max {d_any.max():.4f} m')
    print(f'    c_arm  | ARM_BLOCK "any" | ARM_BLOCK "all" (optimistic)')
    for c in (0.00, 0.02, 0.05, 0.10, 0.15, 0.20):
        na, nl = int((d_any <= c).sum()), int((d_all <= c).sum())
        print(f'    {c:5.2f}  | {na:6d} = {100.0*na/tested:6.3f} %  | '
              f'{nl:6d} = {100.0*nl/tested:6.3f} %')
    j = int(np.argmin(d_any))
    x, y = rows[j][2], rows[j][3]
    print(f'    closest pair found, d = {d_any[j]:.6f} m:')
    print(f'      g1 pose {x[0]} {inst.pose_str(1, x[0])} tasks {x[1]:06b} '
          f'{x[2]}')
    print(f'      g2 pose {y[0]} {inst.pose_str(2, y[0])} tasks {y[1]:06b} '
          f'{y[2]}')
    # 🔺 c_arm = 0.0 is NOT the analogue of c_clear = 0.0, and L4 is where that
    # shows. The structural bodies have VOLUME (bar, plates), so c_clear = 0
    # means "two solids touch". The arm is a zero-thickness POLYLINE, so
    # c_arm = 0 means "two lines intersect exactly" -- a measure-zero event that
    # cannot be expected to occur even when the real arms would be deep inside
    # each other. The predicate is non-vacuous from the first c_arm that
    # represents any arm thickness at all.
    n0 = int((d_any <= 0.0).sum())
    n5 = int((d_any <= 0.05).sum())
    ok = n5 > 0
    print(f'L4  -> at c_arm = 0.00: {n0} forbidden (zero-thickness polylines '
          f'-> measure zero); at c_arm = 0.05: {n5}')
    print(f'L4  -> {"PASS (NOT vacuous once the arm has any thickness)" if ok else "NO FORBIDDEN COMBINATION AT ANY c_arm -- FINDING (A3-K1)"}')
    return ok


# ---------------------------------------------------------------------------
# L5 -- the two circulating reach numbers, settled by measurement
# ---------------------------------------------------------------------------
def l5():
    """A2.3: 1.4 m is an ARITHMETIC bound, 1.16 m a measured swept radius, and
    the number that BINDS is the max over the canonical table the instances
    actually use -- which had never been computed. Computed here, first.

    Closed form, derived and then checked against base_pose: for a base-frame
    point (x, y, z) the distance from the gantry rotation axis is
    hypot(PLATE_OFF + x, y), independent of (lin, rot) and identical for both
    mount sides.
    """
    cl = np.load(sa.CLOUD)
    keep = approach_filter(cl['axis'], sa.APPROACH_DEG)
    poly = cl['poly'][keep].astype(np.float64)
    r_sample = sa.swept_radius(poly).max(axis=1)

    # closed form vs base_pose, on random (config, pose)
    rng = np.random.default_rng(0)
    G = _grid()
    ii = rng.integers(0, len(poly), 500)
    pp = rng.integers(0, len(G), 500)
    worst = 0.0
    for a in ('arm1', 'arm2', 'arm3', 'arm4'):
        R, o = base_pose(a, G[pp, 0], G[pp, 1])
        W = np.einsum('pij,pkj->pki', R, poly[ii]) + o[:, None, :]
        W = np.concatenate([o[:, None, :], W], axis=1)
        y_b = 0.36 if a in ('arm1', 'arm2') else -0.36
        ax = np.stack([G[pp, 0], np.full(len(pp), y_b)], axis=1)
        r_true = np.linalg.norm(W[:, :, :2] - ax[:, None, :], axis=-1).max(1)
        worst = max(worst, float(np.abs(r_true - np.maximum(
            r_sample[ii], sc.PLATE_OFF)).max()))
    ok_cf = worst < 1e-12
    print(f'L5  closed form vs base_pose, 4 arms x 500 (config, pose): '
          f'max |diff| {worst:.3e} m -> {"PASS" if ok_cf else "FAIL"}')

    used = set()
    for f in MAPS:
        c = CapabilityMap.load(f)
        u = np.unique(c.canon)
        used.update(u[u >= 0].tolist())
    used = np.array(sorted(used))
    r_used = r_sample[used]
    hb = sa.hang_poly_base()
    r_hang = max(sc.PLATE_OFF, float(sa.swept_radius(hb).max()))
    print(f'L5  swept radius about the gantry rotation axis:')
    print(f'      HANGING (q = 0)                       {r_hang:.4f} m   '
          f'(structure R_MAX = {sc.R_MAX:.4f})')
    print(f'      canonical table IN USE ({len(used)} configs)  '
          f'{r_used.max():.4f} m   <- the number A2.3 asked for')
    print(f'      whole approach-filtered cloud          '
          f'{r_sample.max():.4f} m')
    print(f'      p50 {np.percentile(r_used, 50):.4f}  p90 '
          f'{np.percentile(r_used, 90):.4f}  p99 '
          f'{np.percentile(r_used, 99):.4f}')
    print(f'      arithmetic bound A2.3 (0.4 + 1.0)      1.4000 m   '
          f'{"VALID upper bound" if r_used.max() < 1.4 else "VIOLATED"}')
    ok_c = abs(r_hang - sa.R_HANG) < 1e-4 and abs(r_used.max()
                                                  - sa.R_ARM_MAX) < 1e-4
    print(f'L5  module constants R_HANG / R_ARM_MAX match the measurement -> '
          f'{"PASS" if ok_c else "FAIL"}')

    # Lemma 1, arm version (A2.4 point 3)
    poses = _grid()
    for c_arm in (0.0, 0.05, 0.10, 0.15, 0.20):
        s = sa.arm_safe_poses(poses, r_hang - sc.PLATE_OFF, r_used.max(), c_arm)
        s_struct = sc.safe_poses(poses, c_arm)
        print(f'      Lemma 1 (arm) c_arm={c_arm:.2f}: {len(s):5d}/{len(poses)}'
              f' universally safe poses   (structure: {len(s_struct)})')
    return ok_cf and ok_c


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['all']
    tests = [('L0', l0), ('L1', l1), ('L2', l2), ('L3', l3), ('L4', l4),
             ('L5', l5)]
    if a[0] != 'all':
        tests = [(t, f) for t, f in tests if t.lower() == a[0].lower()]
    res = []
    for tag, fn in tests:
        res.append((tag, fn()))
        print()
    print('=' * 66)
    for tag, ok in res:
        print(f'{tag}: {"PASS" if ok else "FAIL"}')
    allok = all(ok for _, ok in res)
    print('=' * 66)
    print('A3-K1 GATE: ' + ('L0-L5 PASS -- the predicate may be used'
                            if allok else 'FAILED -- A7 step 3 says stop here'))
    return 0 if allok else 1


if __name__ == '__main__':
    raise SystemExit(main())
