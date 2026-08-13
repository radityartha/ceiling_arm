"""G2 -- inverse-reachability sweep over gantry pose, for the P1 coordination study.

Inverts the sampling order of data_gen.py. data_gen samples the full 8-DOF
(gantry + arm) jointly and asks "where can the tool go?". Here we ask the
inverse question the coordination-degree decision actually needs:

    for a target t, which gantry poses g = (linear, rotation) put t inside
    arm a's reach?   ->   G_a(t)

The whole sweep rests on one structural fact, checked by the `verify`
subcommand: the arm base pose is a RIGID function of (linear, rotation) alone.
world -> t{k}_base_link -> [linear, prismatic x] -> [rotation, revolute z]
-> mount plate -> arm base link, with nothing but fixed joints in between.
So the tool cloud never has to be recomputed per gantry pose:

    reachable(t | g)  <=>  min_p || p - T_a(g)^-1 . t || < tol,   p in P

with P sampled ONCE in the arm base frame. We transform the target, not the
cloud -- one KD-tree query per (target, pose) instead of one FK sweep.

Subcommands
-----------
  verify    URDF check: base pose rigid in (linear, rotation)? + arm symmetry.
  cloud     Sample the arm-base-frame tool cloud P (+ arm polyline). Cached npz.
  sweep     Reach masks G_a(t) over the gantry grid x target pool. Cached npz.
  analyze   X0 (coordination degree vs task spread), X1 (fixed vs coupling-
            aware assignment), connectivity of G_a(t) (open question 2), and
            obstacle amplification through coupling (open question 1).

Example
-------
  python3 -m reachability_gng.irm_sweep verify
  python3 -m reachability_gng.irm_sweep cloud  --n 500000 --out /tmp/irm_cloud.npz
  python3 -m reachability_gng.irm_sweep sweep  --cloud /tmp/irm_cloud.npz \
      --out /tmp/irm_sweep.npz
  python3 -m reachability_gng.irm_sweep analyze --sweep /tmp/irm_sweep.npz \
      --cloud /tmp/irm_cloud.npz

Position-only, like the reach map the executor already consumes
(gantry_reach_executor.py: "the GNG reach map is POSITION-only"). Orientation
feasibility stays with IK downstream.
"""

from __future__ import annotations

import argparse
import itertools
import time

import numpy as np

URDF_DEFAULT = 'ros2_ws/src/workcell_description/urdf/workcell_full.urdf'

# Gen3 Lite joint limits (URDF t*_a*_joint_1..6).
ARM_LIMITS = np.array([2.68, 2.61, 2.61, 2.60, 2.53, 2.60])

# arm label -> (gantry prefix, urdf arm prefix). Mount side sets the rotation
# offset: the right plate sits at local y=-0.4, the left plate at y=+0.4.
ARMS = {
    'arm1': ('t1', 't1_a1', 'right'),
    'arm2': ('t1', 't1_a2', 'left'),
    'arm3': ('t2', 't2_a1', 'right'),
    'arm4': ('t2', 't2_a2', 'left'),
}
ARM_ORDER = ['arm1', 'arm2', 'arm3', 'arm4']
GANTRY_OF = {'arm1': 0, 'arm2': 0, 'arm3': 1, 'arm4': 1}

# Link origins traced as the arm's collision polyline (base -> ... -> tool).
POLY_FRAMES = ['arm_link', 'lower_wrist_link', 'tool_frame']


# --------------------------------------------------------------------------
# T_a(g): arm base pose as a closed-form rigid function of (linear, rotation)
# --------------------------------------------------------------------------
def base_pose(arm, lin, rot):
    """World pose of <arm>_base_link at gantry pose (lin, rot).

    Derived from the URDF chain and checked against pinocchio to 1e-16 by
    `verify`. lin/rot broadcast; returns R (...,3,3) and p (...,3).

        p = (lin - y_p sin(psi),  y_b + y_p cos(psi),  1.9525),  psi = pi/2 + rot
        R = Rz(rot + yaw_off) . Rx(pi)

    y_b = +-0.36 (gantry rail y), y_p = -+0.4 (mount plate offset from the
    rotation axis), yaw_off = 0 for a right-plate arm and pi for a left-plate
    arm. The two arms on one gantry therefore differ by EXACTLY a pi shift in
    rotation -- see `verify`.
    """
    tbl, _, side = ARMS[arm]
    y_b = 0.36 if tbl == 't1' else -0.36
    y_p = -0.4 if side == 'right' else 0.4
    yaw_off = 0.0 if side == 'right' else np.pi

    lin, rot = np.broadcast_arrays(np.asarray(lin, float), np.asarray(rot, float))
    psi = np.pi / 2 + rot
    p = np.stack([lin - y_p * np.sin(psi),
                  y_b + y_p * np.cos(psi),
                  np.full(lin.shape, 1.9525)], axis=-1)

    th = rot + yaw_off
    c, s, z, o = np.cos(th), np.sin(th), np.zeros(th.shape), np.ones(th.shape)
    # Rz(th) . Rx(pi) -- symmetric, so R^T == R.
    R = np.stack([np.stack([c, s, z], -1),
                  np.stack([s, -c, z], -1),
                  np.stack([z, z, -o], -1)], axis=-2)
    return R, p


def to_base_frame(arm, lin, rot, t):
    """Express world target(s) t (...,3) in the arm base frame at pose(s)."""
    R, p = base_pose(arm, lin, rot)
    return np.einsum('...ij,...j->...i', R, t - p)   # R^T == R here


# --------------------------------------------------------------------------
# verify
# --------------------------------------------------------------------------
def cmd_verify(args):
    import pinocchio as pin

    full = pin.buildModelFromUrdf(args.urdf)
    data = full.createData()
    rng = np.random.default_rng(0)

    def fk(assign, frame):
        q = pin.neutral(full)
        for name, val in assign.items():
            q[full.idx_qs[full.getJointId(name)]] = val
        pin.forwardKinematics(full, data, q)
        fid = full.getFrameId(frame)
        pin.updateFramePlacement(full, data, fid)
        return data.oMf[fid].copy()

    print('=' * 72)
    print('A  base pose invariant to the 6 arm joints')
    print('B  closed-form T_a(g) vs pinocchio')
    print('C  world tool pose == T_a(g) . (base-frame tool pose)')
    print('=' * 72)
    worst = 0.0
    for arm in ARM_ORDER:
        _, pre, _ = ARMS[arm]
        tbl = ARMS[arm][0]
        aj = [f'{pre}_joint_{i}' for i in range(1, 7)]
        bf, tf = f'{pre}_base_link', f'{pre}_tool_frame'

        g0 = {f'{tbl}_linear_joint': 0.73, f'{tbl}_rotation_joint': 1.234}
        ref = fk(g0, bf)
        a_dev = max(
            np.abs((ref.inverse() * fk({**g0, **dict(zip(aj, rng.uniform(-ARM_LIMITS, ARM_LIMITS)))},
                                       bf)).homogeneous - np.eye(4)).max()
            for _ in range(25))

        b_dev = c_dev = 0.0
        for _ in range(50):
            l, r = rng.uniform(0, 2.0), rng.uniform(-np.pi, np.pi)
            M = fk({f'{tbl}_linear_joint': l, f'{tbl}_rotation_joint': r}, bf)
            R, p = base_pose(arm, l, r)
            b_dev = max(b_dev, np.abs(M.rotation - R).max(), np.abs(M.translation - p).max())

            qa = dict(zip(aj, rng.uniform(-ARM_LIMITS, ARM_LIMITS)))
            world = fk({f'{tbl}_linear_joint': l, f'{tbl}_rotation_joint': r, **qa}, tf)
            zero = {f'{tbl}_linear_joint': 0.0, f'{tbl}_rotation_joint': 0.0}
            rel = fk({**zero, **qa}, bf).inverse() * fk({**zero, **qa}, tf)
            c_dev = max(c_dev, np.linalg.norm(world.translation - (R @ rel.translation + p)))

        worst = max(worst, a_dev, b_dev, c_dev)
        print(f'  {arm} ({pre:6s})  A={a_dev:.2e}  B={b_dev:.2e}  C={c_dev:.2e}')

    # intra-gantry pi symmetry: T_left(rot) == T_right(rot + pi)
    print('-' * 72)
    for a_r, a_l in (('arm1', 'arm2'), ('arm3', 'arm4')):
        d = 0.0
        for _ in range(50):
            l, r = rng.uniform(0, 2.0), rng.uniform(-np.pi, np.pi)
            Rl, pl = base_pose(a_l, l, r)
            Rr, pr = base_pose(a_r, l, r + np.pi)
            d = max(d, np.abs(Rl - Rr).max(), np.abs(pl - pr).max())
        print(f'  {a_l}(rot) == {a_r}(rot+pi) : {d:.2e}')

    # The B/C residual is NOT a modelling error: the URDF writes the arm-base
    # roll as the literal 3.14159, which is pi - 2.65e-6. base_pose() uses
    # exact pi (the intended geometry), so the two disagree by ~2.7 um at the
    # tool -- four orders of magnitude below the 5 cm reach tolerance. Check A
    # and the pi-symmetry check are exact because neither involves that angle.
    print('-' * 72)
    print(f'worst deviation {worst:.2e} m '
          f'(URDF roll literal 3.14159 vs exact pi = {np.pi - 3.14159:.2e} rad)')
    print('RIGID, sweep assumption holds' if worst < 1e-4 else 'FAILED')
    return 0 if worst < 1e-4 else 1


# --------------------------------------------------------------------------
# cloud
# --------------------------------------------------------------------------
def cmd_cloud(args):
    import pinocchio as pin

    full = pin.buildModelFromUrdf(args.urdf)
    pre = ARMS['arm1'][1]
    aj = [f'{pre}_joint_{i}' for i in range(1, 7)]
    lock = [j for j in range(1, full.njoints) if full.names[j] not in aj]
    m = pin.buildReducedModel(full, lock, pin.neutral(full))
    d = m.createData()
    order = [m.names[j] for j in range(1, m.njoints)]
    assert order == aj, f'joint order mismatch: {order}'

    bid = m.getFrameId(f'{pre}_base_link')
    fids = [m.getFrameId(f'{pre}_{f}') for f in POLY_FRAMES]

    rng = np.random.default_rng(args.seed)
    Q = rng.uniform(-ARM_LIMITS, ARM_LIMITS, size=(args.n, 6))
    poly = np.empty((args.n, len(fids), 3), dtype=np.float32)
    axis = np.empty((args.n, 3), dtype=np.float32)
    manip = np.empty((args.n,), dtype=np.float32)
    sigmin = np.empty((args.n,), dtype=np.float32)

    t0 = time.time()
    for i, q in enumerate(Q):
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacement(m, d, bid)
        Tb = d.oMf[bid].inverse()
        for k, fid in enumerate(fids):
            pin.updateFramePlacement(m, d, fid)
            poly[i, k] = (Tb * d.oMf[fid]).translation
        # tool approach direction (tool-frame +z) expressed in the arm base frame
        axis[i] = (Tb.rotation @ d.oMf[fids[-1]].rotation)[:, 2]
        # Yoshikawa manipulability sqrt(det(J J^T)) -- the tie-breaker used to
        # pick ONE canonical configuration per (target, gantry pose). Frame
        # choice does not matter here: the gantry adds only a rigid transform,
        # which leaves the singular values of J untouched.
        J = pin.computeFrameJacobian(m, d, q, fids[-1])[:3]
        sv = np.linalg.svd(J, compute_uv=False)
        manip[i] = sv.prod()          # == sqrt(det(J J^T)), translational only
        sigmin[i] = sv[-1]            # distance to singularity
    dt = time.time() - t0

    tool = poly[:, -1]
    r = np.linalg.norm(tool, axis=1)
    np.savez_compressed(args.out, poly=poly, axis=axis, manip=manip,
                        sigmin=sigmin,
                        q=Q.astype(np.float32),
                        frames=np.array(POLY_FRAMES), seed=args.seed)
    print(f'{args.n} samples in {dt:.1f}s -> {args.out}')
    print(f'  tool radius   {r.min():.3f} .. {r.max():.3f} m')
    print(f'  base-frame bbox  {np.round(tool.min(0),3)}  {np.round(tool.max(0),3)}')
    # mean nearest-neighbour spacing tells us the sampling floor on tol
    from scipy.spatial import cKDTree
    sub = tool[rng.choice(len(tool), min(5000, len(tool)), replace=False)]
    nn, _ = cKDTree(tool).query(sub, k=2, workers=-1)
    print(f'  NN spacing    median {np.median(nn[:,1])*100:.2f} cm, '
          f'p95 {np.percentile(nn[:,1],95)*100:.2f} cm  '
          f'(tol must stay well above this)')
    return 0


# --------------------------------------------------------------------------
# sweep
# --------------------------------------------------------------------------
RAIL_MAX_M = 1.60
"""Operational limit of the linear rail, in metres.

NOT 2.0. The end stop was MEASURED at ~1.656 m from encoder zero on 2026-08-13,
when the gantry ran into it (docs/p1_g3_timing.md §B3b); 1.60 keeps a 56 mm
margin. The URDF and the bridge both claimed 2.0 m, which is ~344 mm of travel
that does not physically exist.

⚠️ Maps swept BEFORE 2026-08-13 used 0..2.0 m, i.e. 41 linear poses of which the
last 8 are UNREACHABLE. Any coverage/co-feasibility number taken from such a map
is optimistic and must be re-swept before it is quoted.
"""


def make_grid(lin_step, rot_step_deg, lin_max=RAIL_MAX_M):
    """Gantry pose grid. Rotation is HALF-OPEN [-180, 180) so that -180 and
    +180 -- the same physical pose -- are not double counted, and the grid is
    naturally cylindrical."""
    lin = np.arange(0.0, lin_max + 1e-9, lin_step)
    rot = np.deg2rad(np.arange(-180.0, 180.0, rot_step_deg))
    return lin, rot


def make_targets(args):
    x = np.arange(args.tx[0], args.tx[1] + 1e-9, args.tstep)
    y = np.arange(args.ty[0], args.ty[1] + 1e-9, args.tstep)
    z = np.asarray(args.tz, float)
    return np.stack(np.meshgrid(x, y, z, indexing='ij'), -1).reshape(-1, 3)


def approach_filter(axis, approach_deg):
    """Keep only samples whose tool +z points DOWN in world, within approach_deg.

    This filter is EXACT and pose-independent. The arm base orientation is
    R = Rz(theta).Rx(pi), so requiring the world tool axis near (0,0,-1) is the
    same as requiring the BASE-frame tool axis near (0,0,+1) -- the Rz(theta)
    part cannot tilt a vector away from the z axis. One static mask on the
    cloud, no per-pose work. (Only true because the grasp approach is parallel
    to the gantry rotation axis; a tilted approach would need a per-pose test.)
    """
    if approach_deg >= 180.0:
        return np.ones(len(axis), dtype=bool)
    return axis[:, 2] >= np.cos(np.deg2rad(approach_deg))


def cmd_sweep(args):
    from scipy.spatial import cKDTree

    cl = np.load(args.cloud)
    keep = approach_filter(cl['axis'], args.approach)
    tool = cl['poly'][:, -1][keep].astype(np.float64)
    if args.approach < 180.0:
        print(f'approach <= {args.approach:.0f} deg from vertical: '
              f'{keep.sum()}/{len(keep)} samples kept ({keep.mean()*100:.1f}%)')
    tree = cKDTree(tool)

    lin, rot = make_grid(args.lin_step, args.rot_step)
    L, Rn = len(lin), len(rot)
    G = np.stack(np.meshgrid(lin, rot, indexing='ij'), -1).reshape(-1, 2)
    targets = make_targets(args)
    T = len(targets)
    print(f'grid {L}x{Rn}={L*Rn} gantry poses, {T} targets, '
          f'cloud {len(tool)}, tol {args.tol} m')

    tols = np.atleast_1d(np.asarray(args.tol, float))
    masks = np.zeros((len(tols), 4, T, L, Rn), dtype=bool)

    t0 = time.time()
    for ai, arm in enumerate(ARM_ORDER):
        # (T, L*Rn, 3) targets expressed in this arm's base frame at every pose
        q = to_base_frame(arm, G[None, :, 0], G[None, :, 1], targets[:, None, :])
        dist, _ = tree.query(q.reshape(-1, 3), k=1, workers=-1)
        dist = dist.reshape(T, L, Rn)
        for ti, tv in enumerate(tols):
            masks[ti, ai] = dist < tv
        print(f'  {arm}: {time.time()-t0:5.1f}s')

    np.savez_compressed(args.out, masks=masks, tols=tols, lin=lin, rot=rot,
                        targets=targets, arms=np.array(ARM_ORDER),
                        cloud_n=len(tool), approach=args.approach)
    print(f'-> {args.out}')
    for ti, tv in enumerate(tols):
        frac = masks[ti].mean(axis=(2, 3))
        print(f'  tol {tv:.2f}: mean |G_a(t)|/|G| per arm '
              f'{np.round(frac.mean(1)*100, 2)} %  '
              f'targets never reached: '
              f'{int((~masks[ti].any(axis=(2,3))).sum())}/{4*T}')
    return 0


# --------------------------------------------------------------------------
# analyze
# --------------------------------------------------------------------------
def feasible_sets(masks_flat):
    """masks_flat: (4, T, P) bool over flattened gantry poses.

    Returns per-gantry pair/single feasibility lookups used by X0/X1:
      single[a][t]        -- arm a can reach target t at some pose
      pair[gantry][i][j]  -- the gantry's two arms can reach (t_i, t_j) at ONE
                             SHARED pose  (arm order within gantry: low, high)
    """
    single = masks_flat.any(axis=2)                       # (4, T)
    pair = []
    for g, (a, b) in enumerate(((0, 1), (2, 3))):
        # (T, T) : exists a pose where arm a reaches t_i and arm b reaches t_j
        pair.append(np.einsum('ip,jp->ij', masks_flat[a].astype(np.float32),
                              masks_flat[b].astype(np.float32)) > 0)
    return single, pair


def degree_stats(single, pair, tasks):
    """Largest feasible arm subset per task set, fixed vs coupling-aware.

    tasks: (K, 4) target indices. Fixed assignment = target j -> arm j.
    Coupling-aware = best over all injective arm->target matchings.
    Feasibility factorises per gantry (verified separately), so a subset is
    feasible iff each gantry's own constraint holds.
    """
    K = len(tasks)
    best = np.zeros(K, dtype=np.int8)
    fixed4 = np.zeros(K, dtype=bool)
    aware4 = np.zeros(K, dtype=bool)

    # all injective partial matchings arms -> targets, largest first
    matchings = []
    for k in range(4, 0, -1):
        for arms in itertools.combinations(range(4), k):
            for tg in itertools.permutations(range(4), k):
                matchings.append((arms, tg))

    def ok(assign, tk):
        """assign: dict arm_idx -> target slot."""
        for g, (a, b) in enumerate(((0, 1), (2, 3))):
            ha, hb = a in assign, b in assign
            if ha and hb:
                if not pair[g][tk[assign[a]], tk[assign[b]]]:
                    return False
            elif ha:
                if not single[a][tk[assign[a]]]:
                    return False
            elif hb:
                if not single[b][tk[assign[b]]]:
                    return False
        return True

    for i, tk in enumerate(tasks):
        if ok({0: 0, 1: 1, 2: 2, 3: 3}, tk):
            fixed4[i] = True
        for arms, tg in matchings:
            if ok(dict(zip(arms, tg)), tk):
                best[i] = len(arms)
                if len(arms) == 4:
                    aware4[i] = True
                break
    return best, fixed4, aware4


def sample_tasks(rng, pool, idx, radius, k):
    """k task sets of 4 target indices, all within `radius` of a random centre.

    radius = inf gives uniform draws over the pool. Rejection-samples centres
    that cannot supply 4 targets.
    """
    if np.isinf(radius):
        return np.stack([rng.choice(idx, 4, replace=False) for _ in range(k)])
    out = []
    while len(out) < k:
        c = pool[rng.integers(len(pool))]
        near = np.where(np.linalg.norm(pool - c, axis=1) <= radius)[0]
        if len(near) >= 4:
            out.append(idx[rng.choice(near, 4, replace=False)])
    return np.stack(out)


def cmd_analyze(args):
    sw = np.load(args.sweep)
    masks, tols = sw['masks'], sw['tols']
    lin, rot, targets = sw['lin'], sw['rot'], sw['targets']
    L, Rn = len(lin), len(rot)
    T = len(targets)
    ti = int(np.argmin(np.abs(tols - args.tol)))
    print(f'== sweep {L}x{Rn} poses, {T} targets, cloud {int(sw["cloud_n"])}, '
          f'tol {tols[ti]:.2f} m ==\n')

    M = masks[ti].reshape(4, T, L * Rn)
    live = M.any(axis=2)
    print('--- reach coverage ---')
    for ai, arm in enumerate(ARM_ORDER):
        f = M[ai].mean(axis=1)
        print(f'  {arm}: reaches {live[ai].mean()*100:5.1f}% of targets; '
              f'|G_a(t)|/|G| median {np.median(f[live[ai]])*100:5.2f}%  '
              f'(= {np.median(f[live[ai]])*L*Rn:.0f} of {L*Rn} poses)')

    # exact pi-symmetry check between the two arms of a gantry
    print('\n--- intra-gantry pi symmetry (structural) ---')
    half = Rn // 2
    for (a, b) in ((0, 1), (2, 3)):
        A = masks[ti][a]
        B = np.roll(masks[ti][b], half, axis=2)
        print(f'  {ARM_ORDER[a]} vs {ARM_ORDER[b]} rolled by pi: '
              f'{"identical" if np.array_equal(A, B) else "DIFFER"} '
              f'({np.mean(A != B)*100:.3f}% cells differ)')

    single, pair = feasible_sets(M)
    print('\n--- pairwise co-feasibility (shared gantry pose, distinct targets) ---')
    off = ~np.eye(T, dtype=bool)
    for g, (a, b) in enumerate(((0, 1), (2, 3))):
        valid = np.outer(single[a], single[b]) & off
        print(f'  gantry {g+1} ({ARM_ORDER[a]}+{ARM_ORDER[b]}): '
              f'{pair[g][off].mean()*100:5.2f}% of target pairs co-feasible '
              f'({pair[g][valid].mean()*100:5.2f}% of individually-reachable pairs)')

    # the diagonal of `pair` is not a target pair -- it is the handover
    # condition from the plan: G_a(t) & G_b(t) != {} for ONE target t.
    print('\n--- handover poses (both arms reach the SAME target) ---')
    for g, (a, b) in enumerate(((0, 1), (2, 3))):
        d = np.diag(pair[g])
        both = single[a] & single[b]
        print(f'  intra-gantry {g+1}: {d.mean()*100:5.1f}% of targets have a '
              f'shared-pose handover ({d[both].mean()*100:5.1f}% of targets '
              f'both arms can reach at all)')
    for a, b in ((0, 2), (0, 3), (1, 2), (1, 3)):
        h = single[a] & single[b]
        print(f'  inter-gantry {ARM_ORDER[a]}+{ARM_ORDER[b]}: {h.mean()*100:5.1f}% '
              f'of targets (independent gantry poses, so reach-only)')

    # ---------------- X0 / X1 ----------------
    # X0 is "degree vs TASK SPREAD", so the task distribution is the sweep
    # variable: draw 4 targets within radius r of a random centre. r = inf is
    # the unconstrained/uniform case.
    rng = np.random.default_rng(args.seed)
    reach_any = np.where(single.any(axis=0))[0]
    pool = targets[reach_any]

    print(f'\n--- X0 / X1: coordination degree vs task spread '
          f'({args.k} task sets each) ---')
    print(f'{"spread":>9} | {"deg 1":>6} {"deg 2":>6} {"deg 3":>6} {"deg 4":>6} | '
          f'{"mean":>5} | {"fixed4":>7} {"aware4":>7} {"gain":>5}')
    print('-' * 78)
    for r in list(args.spread) + [np.inf]:
        tasks = sample_tasks(rng, pool, reach_any, r, args.k)
        best, fixed4, aware4 = degree_stats(single, pair, tasks)
        lab = 'uniform' if np.isinf(r) else f'r<={r:.2f} m'
        gain = aware4.mean() / fixed4.mean() if fixed4.mean() > 0 else np.inf
        print(f'{lab:>9} | ' +
              ' '.join(f'{(best==k).mean()*100:5.2f}%' for k in range(1, 5)) +
              f' | {best.mean():5.2f} | {fixed4.mean()*100:6.2f}% '
              f'{aware4.mean()*100:6.2f}% {gain:4.1f}x')

    # ---------------- open question 2: connectivity ----------------
    print('\n--- Q2: is G_a(t) disconnected in (linear, rotation)? ---')
    for tv_i, tv in enumerate(tols):
        r = connectivity(masks[tv_i], args.min_cells)
        print(f'  tol {tv:.2f} m | cylinder: {r["cyl_multi"]*100:5.1f}% of '
              f'(arm,target) with >1 component, mean {r["cyl_mean"]:.2f} | '
              f'plane (no wrap): {r["pln_multi"]*100:5.1f}%, mean {r["pln_mean"]:.2f} '
              f'| wrap-only merges: {r["wrap_merges"]*100:5.1f}%')
    print(f'  (components smaller than {args.min_cells} cells discarded as '
          f'sampling speckle)')
    return 0


def _components(m, min_cells):
    """(plane count, cylinder count) of 8-connected components of mask m.

    Rotation (axis 1) wraps: the gantry rotation joint spans a full turn, so
    the pose grid is a CYLINDER, not a rectangle. Counted both ways -- the
    difference is exactly the number of maps that only LOOK disconnected
    because the +-pi seam was cut. Components below min_cells are discarded as
    sampling speckle.
    """
    from scipy.ndimage import label
    lab, n = label(m, structure=np.ones((3, 3), dtype=int))
    if n == 0:
        return 0, 0

    def big(labels):
        ids, cnt = np.unique(labels[labels > 0], return_counts=True)
        return int((cnt >= min_cells).sum())

    plane = big(lab)

    # merge across the seam: column -1 touches column 0, 8-connected, so a
    # cell (i, -1) also touches (i-1, 0) and (i+1, 0).
    parent = list(range(n + 1))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[max(rx, ry)] = min(rx, ry)

    left, right = lab[:, 0], lab[:, -1]
    for di in (-1, 0, 1):
        shifted = np.roll(left, di)
        if di == -1:
            shifted[-1] = 0
        elif di == 1:
            shifted[0] = 0
        both = (right > 0) & (shifted > 0)
        for u, v in zip(right[both], shifted[both]):
            union(int(u), int(v))

    roots = np.array([find(i) for i in range(n + 1)])
    merged = roots[lab]
    return plane, big(merged)


def connectivity(mask4, min_cells):
    cyl, pln = [], []
    for ai in range(mask4.shape[0]):
        for t in range(mask4.shape[1]):
            m = mask4[ai, t]
            if not m.any():
                continue
            p, c = _components(m, min_cells)
            pln.append(p)
            cyl.append(c)
    cyl, pln = np.array(cyl), np.array(pln)
    return dict(cyl_multi=(cyl > 1).mean(), cyl_mean=cyl.mean(),
                pln_multi=(pln > 1).mean(), pln_mean=pln.mean(),
                wrap_merges=(pln > cyl).mean(), n=len(cyl))


# --------------------------------------------------------------------------
# obstacle -- open question 1: is the obstacle effect amplified by coupling?
# --------------------------------------------------------------------------
def segments_hit_cylinder(pts, ox, oy, radius, z_lo, z_hi):
    """pts: (..., K+1, 3) world polyline vertices. True if ANY segment of the
    polyline enters the vertical cylinder (ox, oy, radius) over [z_lo, z_hi].

    The arm is approximated by its base->elbow->wrist->tool polyline inflated
    to `radius` (obstacle radius + arm radius folded together by the caller).
    This UNDER-estimates the swept volume -- a bent elbow bulges outside the
    chord -- so the reported capability loss is a lower bound.
    """
    a, b = pts[..., :-1, :], pts[..., 1:, :]
    d = b - a
    # closest point on each segment to the vertical axis, in XY
    dxy = d[..., :2]
    apxy = np.array([ox, oy]) - a[..., :2]
    denom = np.einsum('...i,...i->...', dxy, dxy)
    t = np.where(denom > 1e-12,
                 np.einsum('...i,...i->...', apxy, dxy) / np.maximum(denom, 1e-12),
                 0.0)
    t = np.clip(t, 0.0, 1.0)
    closest = a[..., :2] + t[..., None] * dxy
    near = np.linalg.norm(closest - np.array([ox, oy]), axis=-1) < radius
    # z overlap of the whole segment with the cylinder's z span (conservative
    # in the sense of decoupling xy from z; segments are short so the slack is
    # small compared with the 1.8 m cylinder height)
    zov = (np.minimum(a[..., 2], b[..., 2]) <= z_hi) & \
          (np.maximum(a[..., 2], b[..., 2]) >= z_lo)
    return (near & zov).any(axis=-1)


def cmd_obstacle(args):
    from scipy.spatial import cKDTree

    cl = np.load(args.cloud)
    keep = approach_filter(cl['axis'], args.approach)
    poly = cl['poly'][keep].astype(np.float64)          # (N, 3, 3)
    tool = poly[:, -1]
    tree = cKDTree(tool)

    lin, rot = make_grid(args.lin_step, args.rot_step)
    G = np.stack(np.meshgrid(lin, rot, indexing='ij'), -1).reshape(-1, 2)
    P = len(G)
    targets = make_targets(args)
    T = len(targets)
    K = args.k_cand
    infl = args.obs_radius + args.arm_radius
    print(f'{P} poses, {T} targets, cloud {len(tool)}, tol {args.tol} m, '
          f'{K} candidates/query, obstacle r={args.obs_radius} m '
          f'(+{args.arm_radius} m arm) z<={args.obs_height} m')

    canonical = args.canon != 'anyk'
    if canonical:
        # ONE configuration per (arm, target, pose). This is what makes the
        # online obstacle filter expressible as a mask at all -- see the note
        # in canonical_table. Also ~200x cheaper than the any-of-K sweep.
        score = canon_score(args.canon, cl['manip'][keep].astype(np.float64),
                            cl['sigmin'][keep].astype(np.float64),
                            cl['q'][keep].astype(np.float64))
        W = np.empty((4, T, P, 4, 3), dtype=np.float32)
        base_free = np.empty((4, T, P), dtype=bool)
        for ai, arm in enumerate(ARM_ORDER):
            canon, sat = canonical_table(arm, tree, score, targets, G,
                                         args.tol, K)
            base_free[ai] = canon >= 0
            W[ai] = np.nan_to_num(world_polylines(arm, poly, canon, G),
                                  nan=1e6).astype(np.float32)
        print(f'  canonical policy = {args.canon}, saturation {sat*100:.2f}%')
        cand_i = cand_ok = None
    else:
        # any-of-K: a pose survives if ANY of the K nearest reaching samples is
        # collision-free. Upper bound on capability -- not realisable online,
        # since it presumes an oracle that picks the surviving configuration.
        cand_i = np.empty((4, T, P, K), dtype=np.int32)
        cand_ok = np.empty((4, T, P, K), dtype=bool)
        for ai, arm in enumerate(ARM_ORDER):
            q = to_base_frame(arm, G[None, :, 0], G[None, :, 1], targets[:, None, :])
            dist, idx = tree.query(q.reshape(-1, 3), k=K, workers=-1)
            cand_i[ai] = idx.reshape(T, P, K)
            cand_ok[ai] = (dist < args.tol).reshape(T, P, K)
        sat = cand_ok[..., -1].mean()
        print(f'  any-of-K saturation (all {K} within tol): {sat*100:.2f}% '
              f'{"-- raise --k-cand" if sat > 0.02 else "-- ok"}')
        base_free = cand_ok.any(axis=-1)

    rng = np.random.default_rng(args.seed)
    rows = []
    for o in range(args.n_obs):
        ox = rng.uniform(args.tx[0], args.tx[1])
        oy = rng.uniform(args.ty[0], args.ty[1])
        free = np.empty_like(base_free)
        if canonical:
            for ai in range(4):
                hit = segments_hit_cylinder(W[ai], ox, oy, infl, 0.0, args.obs_height)
                free[ai] = base_free[ai] & ~hit
        else:
            for ai, arm in enumerate(ARM_ORDER):
                R, p = base_pose(arm, G[:, 0], G[:, 1])      # (P,3,3), (P,3)
                ok = np.zeros((T, P), dtype=bool)
                for t0 in range(0, T, args.chunk):           # chunk targets for RAM
                    sl = slice(t0, min(t0 + args.chunk, T))
                    ci = cand_i[ai, sl]                      # (t, P, K)
                    pts = poly[ci]                           # (t, P, K, 3, 3)
                    w = np.einsum('pij,tpkmj->tpkmi', R, pts) + p[None, :, None, None, :]
                    origin = np.broadcast_to(p[None, :, None, None, :],
                                             w.shape[:-2] + (1, 3))
                    w = np.concatenate([origin, w], axis=-2)  # base -> ... -> tool
                    hit = segments_hit_cylinder(w, ox, oy, infl, 0.0, args.obs_height)
                    ok[sl] = (cand_ok[ai, sl] & ~hit).any(axis=-1)
                free[ai] = ok

        rows.append((ox, oy) + losses(base_free, free))
        ox_, oy_, sc, pc, se, pe = rows[-1]
        print(f'  obs {o+1:2d} at ({ox:5.2f},{oy:5.2f}) | config: single {sc*100:5.2f}% '
              f'pair {pc*100:5.2f}% -> {pc/sc if sc else np.nan:4.2f}x | '
              f'exist: single {se*100:5.2f}% pair {pe*100:5.2f}% -> '
              f'{pe/se if se else np.nan:4.2f}x')

    A = np.array(rows)
    print('-' * 78)
    for name, si, pi in (('CONFIGURATION (shrinkage of G_a(t) vs of the intersection)', 2, 3),
                         ('EXISTENCE  (targets lost vs target-pairs lost)', 4, 5)):
        s, pp = A[:, si], A[:, pi]
        ok = s > 0
        amp = pp[ok] / s[ok]
        # Null baseline: if the obstacle removed each arm's configurations
        # INDEPENDENTLY at rate s, the pair set (a product of two) would shrink
        # by 1-(1-s)^2 ~ 2s all by itself. Amplification only means "coupling
        # amplifies" if it beats this. Below it => the two arms lose the SAME
        # poses (positively correlated), which is de-amplification.
        null = (1 - (1 - s[ok]) ** 2) / s[ok]
        print(f'{name}')
        print(f'   single-arm loss : mean {s.mean()*100:5.2f}%  max {s.max()*100:5.2f}%')
        print(f'   coupled   loss  : mean {pp.mean()*100:5.2f}%  max {pp.max()*100:5.2f}%')
        print(f'   amplification   : mean {amp.mean():.2f}x  median {np.median(amp):.2f}x  '
              f'max {amp.max():.2f}x')
        print(f'   null (independent removal) : {null.mean():.2f}x  -> '
              f'{"AMPLIFIED by coupling" if amp.mean() > null.mean() * 1.05 else "NOT amplified (losses are correlated)"}')
    return 0


def losses(base_free, free):
    """Obstacle-induced loss, measured two ways that must not be mixed.

    CONFIGURATION level -- what open question 1 actually asks ("removing ~10%
    of configurations from an already thin intersection"): the shrinkage of
    G_a(t) against the shrinkage of G_a(t_i) & G_b(t_j), both counted over
    (target, pose) configurations.

    EXISTENCE level -- the decision-relevant version: how many targets become
    unreachable at ANY pose, against how many target pairs become co-infeasible
    at any shared pose.

    Amplification is the coupled loss divided by the single-arm loss, computed
    within one level. Comparing across levels (as an earlier draft did) divides
    a per-configuration rate by a per-target rate and means nothing.
    """
    def pair_counts(m):
        """(config count, existence count) of co-feasible pairs, summed over
        both gantries. i == j (both arms on one target) is excluded."""
        cfg = ex = 0
        for a, b in ((0, 1), (2, 3)):
            na, nb = m[a].sum(0).astype(np.int64), m[b].sum(0).astype(np.int64)
            same = (m[a] & m[b]).sum(0).astype(np.int64)
            cfg += int((na * nb - same).sum())
            p = np.einsum('ip,jp->ij', m[a].astype(np.float32),
                          m[b].astype(np.float32)) > 0
            np.fill_diagonal(p, False)
            ex += int(p.sum())
        return cfg, ex

    s_cfg = 1.0 - free.sum() / base_free.sum()
    s_ex = 1.0 - free.any(axis=2).sum() / base_free.any(axis=2).sum()
    c0, e0 = pair_counts(base_free)
    c1, e1 = pair_counts(free)
    return (float(s_cfg), 1.0 - c1 / max(c0, 1),
            float(s_ex), 1.0 - e1 / max(e0, 1))


# --------------------------------------------------------------------------
# canonical redundancy resolution + arm-arm interference
# --------------------------------------------------------------------------
def seg_seg_dist(p1, d1, p2, d2):
    """Min distance between segments p1->p1+d1 and p2->p2+d2 (Ericson).

    All args broadcast to (..., 3). Handles degenerate (zero-length) segments.
    """
    r = p1 - p2
    a = np.einsum('...i,...i->...', d1, d1)
    e = np.einsum('...i,...i->...', d2, d2)
    f = np.einsum('...i,...i->...', d2, r)
    c = np.einsum('...i,...i->...', d1, r)
    b = np.einsum('...i,...i->...', d1, d2)
    eps = 1e-12
    a_s, e_s = np.maximum(a, eps), np.maximum(e, eps)

    denom = a * e - b * b
    s0 = np.where(denom > eps, (b * f - c * e) / np.where(denom > eps, denom, 1.0), 0.0)
    s0 = np.clip(s0, 0.0, 1.0)
    t_raw = (b * s0 + f) / e_s
    t = np.clip(t_raw, 0.0, 1.0)
    # Ericson: s is recomputed ONLY when t had to be clamped.
    s = np.where(t_raw < 0.0, np.clip(-c / a_s, 0.0, 1.0),
                 np.where(t_raw > 1.0, np.clip((b - c) / a_s, 0.0, 1.0), s0))
    # degenerate segments: a zero-length segment collapses to its start point
    s = np.where(a <= eps, 0.0, s)
    t = np.where(e <= eps, 0.0, np.clip((b * s + f) / e_s, 0.0, 1.0))
    s = np.where((a > eps) & (e <= eps), np.clip(-c / a_s, 0.0, 1.0), s)
    cp1 = p1 + s[..., None] * d1
    cp2 = p2 + t[..., None] * d2
    return np.linalg.norm(cp1 - cp2, axis=-1)


def polyline_min_dist(A, B):
    """Min distance between two 4-vertex polylines. A,B: (..., 4, 3) -> (...)."""
    a0, a1 = A[..., :-1, :], A[..., 1:, :]
    b0, b1 = B[..., :-1, :], B[..., 1:, :]
    da, db = a1 - a0, b1 - b0
    # all 3x3 segment pairs
    d = seg_seg_dist(a0[..., :, None, :], da[..., :, None, :],
                     b0[..., None, :, :], db[..., None, :, :])
    return d.reshape(d.shape[:-2] + (-1,)).min(axis=-1)


def canon_score(policy, manip, sigmin, q):
    """Score used to pick THE configuration per (target, gantry pose).

    manip     Yoshikawa sqrt(det(J J^T)), translational rows only -- so unit
              consistent, unlike the classic 6-row form.
    sigmin    smallest singular value of J: true distance to singularity.
              Unlike a determinant it cannot be masked by one large direction.
    limits    joint-limit margin: min over joints of (1 - |q_j| / q_j^max).
              Yoshikawa is blind to this, and a configuration jammed against a
              limit is useless for the motion that follows.
    home      least joint travel from neutral -- cheapest to reach, no J needed.
    combo     sigmin scaled by the limit margin: avoid singularities AND limits.
    """
    lim = (1.0 - np.abs(q) / ARM_LIMITS).min(axis=-1)
    return {'manip': manip,
            'sigmin': sigmin,
            'limits': lim,
            'home': -np.linalg.norm(q, axis=-1),
            'combo': sigmin * np.clip(lim, 0.0, None)}[policy]


def canonical_table(arm, tree, score, targets, G, tol, K):
    """Pick ONE configuration per (target, gantry pose): the reachable cloud
    sample with the highest manipulability.

    Returns (T, P) int32 index into the cloud, -1 where unreachable, plus the
    candidate-set saturation. Committing to a single configuration is what
    makes capability a well-defined function of (t, g) -- without it, obstacle
    and interference filtering cannot be expressed as a mask at all.
    """
    T, P = len(targets), len(G)
    q = to_base_frame(arm, G[None, :, 0], G[None, :, 1], targets[:, None, :])
    dist, idx = tree.query(q.reshape(-1, 3), k=K, workers=-1)
    ok = dist < tol
    sc = np.where(ok, score[idx], -np.inf)
    best = sc.argmax(axis=1)
    sel = idx[np.arange(len(idx)), best]
    sel = np.where(ok.any(axis=1), sel, -1)
    return sel.reshape(T, P).astype(np.int32), float(ok[:, -1].mean())


def world_polylines(arm, poly, canon, G):
    """(T, P, 4, 3) world polylines base->elbow->wrist->tool, NaN where -1."""
    R, p = base_pose(arm, G[:, 0], G[:, 1])           # (P,3,3), (P,3)
    T, P = canon.shape
    out = np.full((T, P, 4, 3), np.nan)
    valid = canon >= 0
    ti, pi = np.nonzero(valid)
    pts = poly[canon[ti, pi]]                          # (M,3,3)
    w = np.einsum('mij,mkj->mki', R[pi], pts) + p[pi][:, None, :]
    out[ti, pi, 0] = p[pi]                             # arm base origin
    out[ti, pi, 1:] = w
    return out


def pair_feasible(arm_a, arm_b, poly, score, tree, targets, G, tol, K, clearance):
    """(T,T) bool: exists a SHARED gantry pose where arm_a reaches t_i and
    arm_b reaches t_j with their canonical configurations not interfering.

    Also returns the reach-only version, so the cost of interference is
    directly visible.
    """
    ca, sat_a = canonical_table(arm_a, tree, score, targets, G, tol, K)
    cb, sat_b = canonical_table(arm_b, tree, score, targets, G, tol, K)
    Wa = world_polylines(arm_a, poly, ca, G)
    Wb = world_polylines(arm_b, poly, cb, G)
    T = len(targets)
    reach = np.zeros((T, T), dtype=bool)
    free = np.zeros((T, T), dtype=bool)
    n_co = n_hit = 0          # CONFIGURATION level: (i, j, pose) triples
    for p in range(len(G)):
        ia = np.nonzero(ca[:, p] >= 0)[0]
        ib = np.nonzero(cb[:, p] >= 0)[0]
        if len(ia) == 0 or len(ib) == 0:
            continue
        reach[np.ix_(ia, ib)] = True
        d = polyline_min_dist(Wa[ia, p][:, None], Wb[ib, p][None, :])
        same = ia[:, None] == ib[None, :]       # i == j is not an assignment
        hit = (d < clearance) & ~same
        n_co += int((~same).sum())
        n_hit += int(hit.sum())
        free[np.ix_(ia, ib)] |= (d >= clearance)
    np.fill_diagonal(reach, False)
    np.fill_diagonal(free, False)
    cfg_loss = n_hit / max(n_co, 1)
    return reach, free, max(sat_a, sat_b), cfg_loss

def cmd_interfere(args):
    """Does arm-arm interference actually bite? Re-runs X0/X1 with it."""
    from scipy.spatial import cKDTree

    cl = np.load(args.cloud)
    keep = approach_filter(cl['axis'], args.approach)
    poly = cl['poly'][keep].astype(np.float64)
    score = canon_score(args.canon, cl['manip'][keep].astype(np.float64),
                        cl['sigmin'][keep].astype(np.float64),
                        cl['q'][keep].astype(np.float64))
    tree = cKDTree(poly[:, -1])

    lin, rot = make_grid(args.lin_step, args.rot_step)
    G = np.stack(np.meshgrid(lin, rot, indexing='ij'), -1).reshape(-1, 2)
    targets = make_targets(args)
    T = len(targets)
    print(f'{len(G)} poses, {T} targets, cloud {len(poly)}, tol {args.tol} m, '
          f'clearance {args.clearance} m, canonical policy = {args.canon}')

    t0 = time.time()
    reach_pairs, free_pairs, singles = [], [], []
    for a, b in (('arm1', 'arm2'), ('arm3', 'arm4')):
        r, f, sat, cfg = pair_feasible(a, b, poly, score, tree, targets, G,
                                       args.tol, args.k_cand, args.clearance)
        reach_pairs.append(r)
        free_pairs.append(f)
        print(f'  {a}+{b}: configuration level {cfg*100:5.2f}% of co-reaching '
              f'(i,j,pose) collide | existence level {r.mean()*100:5.2f}% -> '
              f'{f.mean()*100:5.2f}% of pairs '
              f'(lost {(1 - f.sum()/max(r.sum(),1))*100:5.2f}%)  '
              f'[sat {sat*100:.1f}%, {time.time()-t0:.0f}s]')

    # single-arm feasibility is unchanged: the idle partner is assumed parked
    # clear of the workspace, so it imposes no constraint.
    for arm in ARM_ORDER:
        c, _ = canonical_table(arm, tree, score, targets, G, args.tol, args.k_cand)
        singles.append((c >= 0).any(axis=1))
    single = np.array(singles)

    # INTER-gantry: the two gantry poses are INDEPENDENT, so the (g1, g2)
    # space is 2952^2 -- far too large to enumerate. Sample it instead: the
    # question is only whether the collision rate is high enough to matter.
    rng = np.random.default_rng(args.seed)
    for a, b in (('arm1', 'arm3'),):
        cA, _ = canonical_table(a, tree, score, targets, G, args.tol, args.k_cand)
        cB, _ = canonical_table(b, tree, score, targets, G, args.tol, args.k_cand)
        WA = world_polylines(a, poly, cA, G)
        WB = world_polylines(b, poly, cB, G)
        vA = np.argwhere(cA >= 0)
        vB = np.argwhere(cB >= 0)
        sA = vA[rng.integers(len(vA), size=args.n_inter)]
        sB = vB[rng.integers(len(vB), size=args.n_inter)]
        m = sA[:, 0] != sB[:, 0]
        d = polyline_min_dist(WA[sA[m, 0], sA[m, 1]], WB[sB[m, 0], sB[m, 1]])
        print(f'  {a}+{b} (inter-gantry, {len(d)} sampled (t_i,g1,t_j,g2)): '
              f'{(d < args.clearance).mean()*100:5.2f}% collide at configuration '
              f'level; independent poses, so existence loss is far smaller')

    idx = np.where(single.any(axis=0))[0]
    pool = targets[idx]
    print(f'\n--- X0 / X1 with intra-gantry interference ({args.k} task sets) ---')
    print(f'{"spread":>9} | {"deg3":>6} {"deg4":>6} | {"mean":>5} | '
          f'{"fixed4":>7} {"aware4":>7} {"gain":>5} | vs reach-only')
    print('-' * 82)
    for r in list(args.spread) + [np.inf]:
        tasks = sample_tasks(rng, pool, idx, r, args.k)
        b0, f0, a0 = degree_stats(single, reach_pairs, tasks)
        b1, f1, a1 = degree_stats(single, free_pairs, tasks)
        lab = 'uniform' if np.isinf(r) else f'r<={r:.2f} m'
        gain = a1.mean() / f1.mean() if f1.mean() > 0 else np.inf
        print(f'{lab:>9} | {(b1==3).mean()*100:5.2f}% {(b1==4).mean()*100:5.2f}% | '
              f'{b1.mean():5.2f} | {f1.mean()*100:6.2f}% {a1.mean()*100:6.2f}% '
              f'{gain:4.1f}x | deg4 was {a0.mean()*100:5.2f}%, mean was {b0.mean():.2f}')
    return 0


def cmd_policy(args):
    """Exact-collision vs Meso zone-exclusion, swept over the exclusion radius r.

    Sensei's GNG_HSR_Topological_Main (Meso-HSR/GNG.h:1223) kills every node of
    one arm's map that lies within r of the other arm's map, and cuts its edges.
    Read as an operating rule: ONLY ONE ARM MAY WORK IN THE SHARED ZONE.

    Here that becomes, per gantry pose p:

        in_zone_a[i,p] = arm a reaches t_i at p  AND  t_i is within r of
                         arm b's reachable set at p
        co-feasible    = both reach  AND  NOT (both work inside the zone)

    "within r of the other arm's reachable set" is exactly the reach mask
    evaluated at tolerance r -- so the whole r sweep comes out of one sweep file.

    r -> tol collapses the zone onto the strict intersection, which is the
    HANDOVER set: the points both arms can reach. Same set, two roles.
    """
    sw = np.load(args.sweep)
    masks, tols = sw['masks'], sw['tols']
    T = masks.shape[2]
    P = masks.shape[3] * masks.shape[4]
    bi = int(np.argmin(np.abs(tols - args.tol)))
    M = masks[bi].reshape(4, T, P)
    single = M.any(axis=2)
    print(f'reach tol {tols[bi]:.2f} m, {P} poses, {T} targets\n')

    rng = np.random.default_rng(args.seed)
    idx = np.where(single.any(axis=0))[0]
    tasks = sample_tasks(rng, sw['targets'][idx], idx, np.inf, args.k)

    def evaluate(fa, fb):
        pairs = []
        for a, b in ((0, 1), (2, 3)):
            A, B = fa(a, b).astype(np.float32), fb(a, b).astype(np.float32)
            ok = (A @ B.T) > 0
            np.fill_diagonal(ok, False)
            pairs.append(ok)
        best, _, aware4 = degree_stats(single, pairs, tasks)
        return np.mean([q.mean() for q in pairs]), aware4.mean(), best.mean()

    hand = np.mean([((M[a] & M[b]).any(axis=1)).mean() for a, b in ((0, 1), (2, 3))])
    print(f'handover set (strict intersection, both arms reach the same target): '
          f'{hand*100:.1f}% of targets\n')
    pr, d4, mn = evaluate(lambda a, b: M[a], lambda a, b: M[b])
    print(f'{"policy":>34} | {"pairs":>7} | {"deg4":>7} | {"mean":>5}')
    print('-' * 62)
    print(f'{"reach only (upper bound)":>34} | {pr*100:6.2f}% | {d4*100:6.2f}% | {mn:5.2f}')
    for ri, r in enumerate(tols):
        if r < tols[bi]:
            continue
        N = masks[ri].reshape(4, T, P)
        # (a) literal GNG_HSR_Topological_Main: map m loses the shared zone
        pr, d4, mn = evaluate(lambda a, b, N=N: M[a] & ~N[b], lambda a, b: M[b])
        print(f'{f"zone r={r:.2f}: one arm loses it":>34} | {pr*100:6.2f}% | '
              f'{d4*100:6.2f}% | {mn:5.2f}')
        # (b) applied both ways = static spatial partition, no shared zone at all
        pr, d4, mn = evaluate(lambda a, b, N=N: M[a] & ~N[b],
                              lambda a, b, N=N: M[b] & ~N[a])
        print(f'{f"zone r={r:.2f}: BOTH lose it":>34} | {pr*100:6.2f}% | '
              f'{d4*100:6.2f}% | {mn:5.2f}   <- handover impossible')
    return 0


# --------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    v = sub.add_parser('verify')
    v.add_argument('--urdf', default=URDF_DEFAULT)
    v.set_defaults(fn=cmd_verify)

    c = sub.add_parser('cloud')
    c.add_argument('--urdf', default=URDF_DEFAULT)
    c.add_argument('--n', type=int, default=500_000)
    c.add_argument('--seed', type=int, default=0)
    c.add_argument('--out', default='/tmp/irm_cloud.npz')
    c.set_defaults(fn=cmd_cloud)

    s = sub.add_parser('sweep')
    s.add_argument('--cloud', default='/tmp/irm_cloud.npz')
    s.add_argument('--lin-step', type=float, default=0.05)
    s.add_argument('--rot-step', type=float, default=5.0)
    s.add_argument('--tol', type=float, nargs='+', default=[0.05, 0.10, 0.15, 0.20])
    s.add_argument('--tx', type=float, nargs=2, default=[0.0, 2.0])
    s.add_argument('--ty', type=float, nargs=2, default=[-0.6, 0.6])
    s.add_argument('--tz', type=float, nargs='+', default=[1.05, 1.25])
    s.add_argument('--tstep', type=float, default=0.2)
    s.add_argument('--approach', type=float, default=180.0,
                   help='max tool tilt from straight-down, deg (180 = free)')
    s.add_argument('--out', default='/tmp/irm_sweep.npz')
    s.set_defaults(fn=cmd_sweep)

    a = sub.add_parser('analyze')
    a.add_argument('--sweep', default='/tmp/irm_sweep.npz')
    a.add_argument('--tol', type=float, default=0.05)
    a.add_argument('--k', type=int, default=20_000)
    a.add_argument('--seed', type=int, default=1)
    a.add_argument('--min-cells', type=int, default=2)
    a.add_argument('--spread', type=float, nargs='*',
                   default=[0.3, 0.5, 0.8, 1.2, 1.8])
    a.set_defaults(fn=cmd_analyze)

    f = sub.add_parser('interfere')
    f.add_argument('--cloud', default='/tmp/irm_cloud.npz')
    f.add_argument('--tol', type=float, default=0.05)
    f.add_argument('--approach', type=float, default=45.0)
    f.add_argument('--lin-step', type=float, default=0.05)
    f.add_argument('--rot-step', type=float, default=5.0)
    f.add_argument('--tx', type=float, nargs=2, default=[0.0, 2.0])
    f.add_argument('--ty', type=float, nargs=2, default=[-0.6, 0.6])
    f.add_argument('--tz', type=float, nargs='+', default=[1.05, 1.25])
    f.add_argument('--tstep', type=float, default=0.2)
    f.add_argument('--clearance', type=float, default=0.10,
                   help='min centreline separation between the two arms (m)')
    f.add_argument('--k-cand', type=int, default=64)
    f.add_argument('--canon', default='manip',
                   choices=['manip', 'sigmin', 'limits', 'home', 'combo'])
    f.add_argument('--n-inter', type=int, default=400_000)
    f.add_argument('--k', type=int, default=8000)
    f.add_argument('--seed', type=int, default=1)
    f.add_argument('--spread', type=float, nargs='*',
                   default=[0.3, 0.5, 0.8, 1.2, 1.8])
    f.set_defaults(fn=cmd_interfere)

    y = sub.add_parser('policy')
    y.add_argument('--sweep', default='/tmp/irm_sweep.npz')
    y.add_argument('--tol', type=float, default=0.05)
    y.add_argument('--k', type=int, default=8000)
    y.add_argument('--seed', type=int, default=1)
    y.set_defaults(fn=cmd_policy)

    o = sub.add_parser('obstacle')
    o.add_argument('--cloud', default='/tmp/irm_cloud.npz')
    o.add_argument('--tol', type=float, default=0.05)
    o.add_argument('--approach', type=float, default=180.0)
    o.add_argument('--lin-step', type=float, default=0.10)
    o.add_argument('--rot-step', type=float, default=10.0)
    o.add_argument('--tx', type=float, nargs=2, default=[0.0, 2.0])
    o.add_argument('--ty', type=float, nargs=2, default=[-0.6, 0.6])
    o.add_argument('--tz', type=float, nargs='+', default=[1.05, 1.25])
    o.add_argument('--tstep', type=float, default=0.2)
    o.add_argument('--obs-radius', type=float, default=0.25)
    o.add_argument('--arm-radius', type=float, default=0.05)
    o.add_argument('--obs-height', type=float, default=1.80)
    o.add_argument('--n-obs', type=int, default=12)
    o.add_argument('--k-cand', type=int, default=64)
    o.add_argument('--canon', default='manip',
                   choices=['manip', 'sigmin', 'limits', 'home', 'combo', 'anyk'],
                   help="'anyk' = old upper-bound behaviour (any of K candidates)")
    o.add_argument('--chunk', type=int, default=8)
    o.add_argument('--seed', type=int, default=2)
    o.set_defaults(fn=cmd_obstacle)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    raise SystemExit(main())
