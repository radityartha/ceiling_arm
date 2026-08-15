#!/usr/bin/env python3
"""G11 / REACH-1: arm-arm collision BETWEEN gantries, as a scheduling constraint.

docs/p1_g11_arm.md A2 locks the model addition this file implements. Read that
section first -- the dangerous part of this session is not the geometry, it is
that arm CONFIGURATION does not exist in the scheduling model at all
(p1_g7 A4.3: arm motion inside one pose costs 0), so a predicate that depends on
it is a MODEL ADDITION and had to be locked before any code.

A2.1, quoted, because everything here is a consequence of it:

    arm a of gantry g, at time t:
      CANONICAL  while a works task i at a stop whose dwell window contains t
                 -> q = canonical_table(a, target_i, pose_g), policy `manip`
      HANGING    otherwise -- traversing, waiting, idle, and during a stop whose
                 task is NOT assigned to a
                 -> q = rest pose, straight down

Nothing here plans arm MOTION. The predicate evaluates the canonical and the
hanging configurations, never the path between them; T_fold stays 0.0 (A4.1).

Frozen files (A0) are imported, never edited: sched, sched_coll, sched_coupled.
"""

from __future__ import annotations

import numpy as np

from reachability_gng import sched
from reachability_gng import sched_coll as sc
from reachability_gng.capability import GANTRY_ARM, PARTNER, CapabilityMap
from reachability_gng.irm_sweep import (approach_filter, base_pose,
                                        polyline_min_dist, world_polylines)

# --- provenance of the canonical table --------------------------------------
# cap_g{1,2}_rail160.npz stores `canon`, an index into the APPROACH-FILTERED
# cloud. Both are reproduced bit-identically from these three values (L1), so
# they are named here rather than left implicit: a canonical index read against
# the wrong cloud is a silent, plausible-looking wrong answer.
# G12 U0: the cloud lived ONLY in /tmp through all of G11 -- a reboot would have
# made every arm polyline of that session unreproducible. The repo copy in data/
# is now the primary; /tmp stays as fallback so an existing checkout keeps
# working. Both are the same file (md5 d62c2f54...), and L1a re-proves the
# canonical table is BIT-IDENTICAL when read through this resolution.
def _cloud_path():
    from pathlib import Path
    here = Path(__file__).resolve()
    for up in here.parents:
        cand = up / 'data' / 'irm_cloud_pol.npz'
        if cand.exists():
            return str(cand)
    return '/tmp/irm_cloud_pol.npz'


CLOUD = _cloud_path()
APPROACH_DEG = 45.0
CANON_POLICY = 'manip'

C_ARM = 0.0           # LOCKED default, A2.3 -- same treatment as c_clear
N_ROT = 72            # rotation grid size, cap map
N_LIN = 33

# A2.1 HANGING. All six joints at zero IS "straight down" for a ceiling mount:
# the base frame is Rz(theta).Rx(pi), so base +z points down in world, and the
# q = 0 polyline runs monotonically along base +z (z = 0.243 -> 0.663 -> 1.003 m
# out from the base) with a horizontal excursion of at most 58 mm. Verified in
# L5. This is a CHOICE of representative configuration -- p1_g3 B4 measured the
# torque of the rest pose but never recorded its joint vector.
HANG_Q = np.zeros(6)


def _rot_shift(idx=None):
    """Pose-index permutation p -> the pose at (lin, rot + pi).

    The two arms of one gantry are EXACTLY half a turn apart (p1_state 3), so
    the partner arm's canonical table and its world polylines are the stored
    arm's, read at the shifted pose. Exact, 8.9e-16 in L1 -- not an
    approximation, and the reason one map serves two arms.
    """
    g = np.arange(N_LIN * N_ROT).reshape(N_LIN, N_ROT)
    return np.roll(g, -N_ROT // 2, axis=1).ravel()


ROT_SHIFT = _rot_shift()


def hang_poly_base(urdf=None):
    """(3, 3) base-frame polyline of the HANGING arm: arm_link, wrist, tool.

    Read from the URDF through pinocchio, the same chain irm_sweep.cmd_cloud
    samples, so the hanging pose and the canonical poses live in one frame by
    construction rather than by agreement.
    """
    import pinocchio as pin
    from reachability_gng.irm_sweep import ARMS, POLY_FRAMES, URDF_DEFAULT
    urdf = urdf or _urdf_path(URDF_DEFAULT)
    full = pin.buildModelFromUrdf(urdf)
    pre = ARMS['arm1'][1]
    aj = [f'{pre}_joint_{i}' for i in range(1, 7)]
    lock = [j for j in range(1, full.njoints) if full.names[j] not in aj]
    m = pin.buildReducedModel(full, lock, pin.neutral(full))
    d = m.createData()
    bid = m.getFrameId(f'{pre}_base_link')
    pin.forwardKinematics(m, d, HANG_Q)
    pin.updateFramePlacement(m, d, bid)
    Tb = d.oMf[bid].inverse()
    out = []
    for f in POLY_FRAMES:
        fid = m.getFrameId(f'{pre}_{f}')
        pin.updateFramePlacement(m, d, fid)
        out.append((Tb * d.oMf[fid]).translation.copy())
    return np.array(out)


def _urdf_path(rel):
    from pathlib import Path
    here = Path(__file__).resolve()
    for up in here.parents:
        cand = up / rel
        if cand.exists():
            return str(cand)
    raise FileNotFoundError(rel)


def hang_world(arm, poses, hb):
    """(P, 4, 3) world polyline of `arm` hanging, at every pose in `poses`."""
    R, p = base_pose(arm, poses[:, 0], poses[:, 1])
    w = np.einsum('pij,kj->pki', R, hb) + p[:, None, :]
    return np.concatenate([p[:, None, :], w], axis=1)


# ===========================================================================
# A2.3 -- the predicate
# ===========================================================================
class ArmGeom:
    """Every world polyline an instance can need, precomputed once.

    Shapes, per gantry g and arm slot k in (0, 1):
        canon[g][k]  (n, P, 4, 3)   arm k of g working task i at pose p
        hang[g][k]   (P, 4, 3)      arm k of g hanging at pose p
    NaN where the task is unreachable by that arm at that pose -- which is
    exactly where the scheduling model already forbids the assignment, so a NaN
    can only be reached through a schedule the gate would reject anyway.

    Memory is n * P * 4 * 3 * 8 B per (gantry, slot): 1.4 MB at n = 6,
    P = 2376. Precomputing is therefore free, and A2.4 point 1's warning --
    that ARM_BLOCK has no precomputable (pose, pose) table because it depends
    on the TASK SETS -- applies to the pair table, not to these.
    """

    def __init__(self, inst, cloud=CLOUD, approach=APPROACH_DEG,
                 maps=('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz')):
        nodes = inst.meta.get('nodes')
        if nodes is None:
            raise ValueError('ArmGeom needs a real instance (meta["nodes"])')
        node_idx = np.asarray(nodes)
        # A probe may restrict the candidate pose set (gen_real_rotcrowded).
        # The polylines are then indexed by the RESTRICTED set, so that a pose
        # index means the same thing here as it does in inst.poses.
        pk = inst.meta.get('pose_keep')
        pk = None if pk is None else np.asarray(pk)
        cl = np.load(cloud)
        keep = approach_filter(cl['axis'], approach)
        poly = cl['poly'][keep].astype(np.float64)
        hb = hang_poly_base()
        self.hang_base = hb
        # horizontal envelope of the hanging arm about the plate centre; the
        # base origin (0, 0) is a vertex of the polyline too.
        self.r_h = float(np.linalg.norm(
            np.vstack([np.zeros(2), hb[:, :2]]), axis=1).max())
        self.canon, self.hang, self.arms = {}, {}, {}
        for g in inst.gantries:
            cap = CapabilityMap.load(maps[g - 1])
            G = np.stack(np.meshgrid(cap.lin, cap.rot, indexing='ij'),
                         -1).reshape(-1, 2)
            c0 = cap.canon[node_idx]                       # (n, P), slot 0
            c1 = c0[:, ROT_SHIFT]                          # slot 1, half turn
            if pk is not None:
                c0, c1, G = c0[:, pk], c1[:, pk], G[pk]
            a0, a1 = GANTRY_ARM[g], PARTNER[g]
            self.arms[g] = (a0, a1)
            self.canon[g] = (world_polylines(a0, poly, c0, G),
                             world_polylines(a1, poly, c1, G))
            self.hang[g] = (hang_world(a0, G, hb), hang_world(a1, G, hb))
        self.n = inst.n
        self.inst = inst

    # -- per-arm configuration set, straight off A2.1 ------------------------
    def configs(self, g, p, tasks, assign):
        """[(slot, (m, 4, 3))] -- what each arm of gantry g looks like there.

        A2.1 gives one canonical configuration per (arm, task). A stop may hand
        the SAME arm several tasks (dur = slots x dwell), and A2.1's window is
        the whole stop, so the arm visits several canonical configurations
        inside it. The model does not order the slots, so the set is carried
        and the caller decides how to quantify over it -- see `pair_min`.
        """
        out = []
        for k in (0, 1):
            got = []
            for i in range(self.n):
                if not (tasks >> i & 1):
                    continue
                a = (assign or {}).get(i)
                if a == 'both' or a == self.arms[g][k]:
                    got.append(self.canon[g][k][i, p])
            out.append(np.array(got) if got else self.hang[g][k][p][None])
        return out


def pair_min(geom, g1, p1, t1, a1, g2, p2, t2, a2, mode='any'):
    """Min arm-arm distance between the two gantries at (pose, task set).

    mode 'any'  min over EVERY (config of an arm of g1, config of an arm of g2)
                pair -- the literal reading of A2.3's `min over a, b`, and the
                pessimistic one when an arm carries several tasks: if any slot
                ordering collides, the pair is refused.
    mode 'all'  the optimistic one: an arm's several configurations are
                assumed orderable, so a pair is refused only if EVERY pairing
                of the two arms' configurations collides.

    They differ only where some arm carries more than one task at the stop.
    Both are reported (B) rather than one being chosen quietly.
    """
    A = geom.configs(g1, p1, t1, a1)
    B = geom.configs(g2, p2, t2, a2)
    best = np.inf
    for X in A:
        for Y in B:
            d = polyline_min_dist(X[:, None], Y[None, :])   # (mA, mB)
            d = np.where(np.isnan(d), np.inf, d)
            v = float(d.min()) if mode == 'any' else float(d.max(axis=1).min())
            best = min(best, v)
    return best


def arm_block(geom, g1, p1, t1, a1, g2, p2, t2, a2, c_arm=C_ARM, mode='any'):
    """ARM_BLOCK((p1, U1), (p2, U2); c_arm), A2.3."""
    return pair_min(geom, g1, p1, t1, a1, g2, p2, t2, a2, mode) <= c_arm


# ===========================================================================
# A2.4 point 3 -- Lemma 1, the ARM version
# ===========================================================================
def arm_half_extent(rot, r_h):
    """(h_x, h_y) of a gantry carrying two HANGING arms, at rotation `rot`.

    Same shape as sched_coll.half_extent, with the bar dropped (the hanging arm
    hangs 1 m BELOW the bar, so the bar is not part of this body) and the plate
    radius replaced by the hanging arm's own horizontal envelope r_h about the
    plate centre. Horizontal distance is convex along a segment, so the
    polyline's max horizontal excursion sits at a vertex and r_h is exact.
    """
    s, c = np.abs(np.sin(rot)), np.abs(np.cos(rot))
    return sc.PLATE_OFF * c + r_h, sc.PLATE_OFF * s + r_h


def arm_safe_poses(poses, r_h, r_arm, c_arm=C_ARM):
    """Poses whose HANGING arms nothing from the other gantry can reach.

    Lemma 1 for arms. The other gantry's arms extend at most `r_arm` from its
    rotation axis (L5 measures it over the canonical table actually used), so
    with our own hanging half-extent h_y the pair cannot meet when

        Y_SEP - h_y(rot) - r_arm > c_arm

    Contrapositive of (N1), the sound direction -- p1_g9 B1 records what the
    unsound direction cost. Returns pose INDICES.
    """
    _, hy = arm_half_extent(np.asarray(poses, float)[:, 1], r_h)
    return np.flatnonzero((sc.Y_SEP - hy - r_arm) > c_arm)


def arm_may_block(lin1, rot1, lin2, rot2, r_h, r_arm, c_arm=C_ARM):
    """Sound (N1)/(N2) prefilter for a HANGING pair. Never False when they meet."""
    hx1, hy1 = arm_half_extent(rot1, r_h)
    hx2, hy2 = arm_half_extent(rot2, r_h)
    n1 = (sc.Y_SEP - hy1 - hy2) <= c_arm
    n2 = (np.abs(np.asarray(lin1, float) - np.asarray(lin2, float))
          - hx1 - hx2) <= c_arm
    return n1 & n2


def hang_block_poses(geom, g1, g2, poses, c_arm=C_ARM, chunk=64):
    """(P, P) bool: the two gantries' HANGING arms interfere. Lemma A's subject.

    This is the traverse x traverse regime of A2.2 frozen at one instant, and
    L2 is what compares it against BLOCK with an inflated c_clear instead of
    assuming the two are the same body.
    """
    P = len(poses)
    H1 = np.stack(geom.hang[g1])        # (2, P, 4, 3)
    H2 = np.stack(geom.hang[g2])
    r_h = geom.r_h
    out = np.zeros((P, P), bool)
    for s in range(0, P, chunk):
        a = poses[s:s + chunk]
        cand = arm_may_block(a[:, 0][:, None], a[:, 1][:, None],
                             poses[None, :, 0], poses[None, :, 1],
                             r_h, r_h, c_arm)
        if not cand.any():
            continue
        ii, jj = np.nonzero(cand)
        d = np.full(len(ii), np.inf)
        for k1 in (0, 1):
            for k2 in (0, 1):
                d = np.minimum(d, polyline_min_dist(H1[k1][s + ii],
                                                    H2[k2][jj]))
        blk = np.zeros(cand.shape, bool)
        blk[ii, jj] = d <= c_arm
        out[s:s + chunk] = blk
    return out


# ===========================================================================
# A2.4 point 2 -- the continuous-time constraint, with ARM_BLOCK in it
# ===========================================================================
# Swept radius of a configuration about the gantry rotation axis has a closed
# form, and it is INDEPENDENT of (lin, rot): for a base-frame point (x, y, z),
#     r = hypot(PLATE_OFF + x, y)
# for BOTH mount sides (the left plate's sign flip on the offset and the pi in
# its yaw offset cancel). Measured in L5:
#     hanging                                  0.4571 m   (R_MAX struct 0.455)
#     max over the canonical table in use      1.0623 m
#     max over the whole approach-filtered cloud 1.0780 m
R_HANG = 0.45713                        # asserted against the URDF in L5
R_ARM_MAX = 1.06230                     # asserted against the maps in L5
V_ARM = sc.V_LIN_M_S + sc.OMEGA_RAD_S * R_HANG
V_REL_ARM = 2.0 * V_ARM


def swept_radius(pts):
    """Distance of base-frame polyline point(s) from the gantry rotation axis."""
    pts = np.asarray(pts, float)
    return np.hypot(sc.PLATE_OFF + pts[..., 0], pts[..., 1])


class ArmTraj:
    """One gantry's arms over time: hanging except inside its own dwells.

    Built from a wait-aware stop list (`depart` + `start` + `dur`, the
    representation p1_g10 A2.4 introduced). Answers `polys(t)`: the arm
    polylines of BOTH arms at time t, following A2.1 exactly --

      inside [start, start + dur) of a stop that carries tasks:
          each arm is at the canonical configuration(s) of ITS assigned tasks;
          an arm with no task at that stop HANGS
      everywhere else (before departure, mid-traverse, waiting, after the last
      stop, and during an empty evasive stop):
          both arms HANG

    The gantry pose during a traverse comes from sched_coll.leg_pose, the
    A2.3 trajectory W1 verified -- not from a second copy of it.
    """

    def __init__(self, inst, geom, g, stops):
        self.geom, self.g = geom, g
        self.tr = sc.Traj(g, tuple(inst.poses[g][inst.p0[g]]))
        self.win = []              # (t0, t1, pose_idx, tasks, assign)
        cur = inst.p0[g]
        for st in stops:
            q = tuple(inst.poses[g][st['pose']])
            dep = st.get('depart', st['start'] - sc.leg_duration(
                tuple(inst.poses[g][cur]), q))
            self.tr.append(dep, q)
            if st['tasks']:
                self.win.append((st['start'], st['start'] + st['dur'],
                                 st['pose'], st['tasks'], st.get('assign')))
            cur = st['pose']
        self.hb = geom.hang_base

    def end_time(self):
        return max([self.tr.end_time()] + [w[1] for w in self.win] + [0.0])

    def polys(self, t):
        """[(m, 4, 3), (m, 4, 3)] -- configuration sets of arm slot 0 and 1."""
        for t0, t1, p, tasks, assign in self.win:
            # 🔴 G12 BUG FIX, and A0's freeze on this file is VOID (p1_g12 B).
            # The right edge was CLOSED (`t < t1 + 1e-12`), so at t == t1 --
            # which is a certificate-walk mark, i.e. a time the walk always
            # samples -- this returned the configuration of the dwell that had
            # just ENDED. The arm state is discontinuous there (T_fold = 0), so
            # the walk then took a Lipschitz step sized by the clearance of the
            # EXTENDED arm and strode straight over the violation the HANGING
            # arm was about to have. Measured on n4_s1_mr1: d(t1) = 0.7828 m
            # with the stale extended config vs 0.1243 m one microsecond later
            # -- a 0.66 m jump, and a step 6x too long.
            # The window is half-open by the model's own convention
            # ([start, start + dur)), so this is the convention being applied,
            # not a new one being chosen.
            if t0 - 1e-12 <= t < t1:
                return self.geom.configs(self.g, p, tasks, assign)
        lin, rot = self.tr.pose_at(np.asarray(t, float))
        out = []
        for k in (0, 1):
            R, o = base_pose(self.geom.arms[self.g][k], float(lin), float(rot))
            w = (R @ self.hb.T).T + o
            out.append(np.concatenate([o[None], w])[None])
        return out

    def moving(self, t):
        """Is the gantry mid-traverse at t? Static arms need no certificate."""
        for ts, _, _, T in self.tr.legs:
            if ts - 1e-12 <= t <= ts + T + 1e-12:
                return True
        return False


def arm_distance(A, B, t):
    """Min distance between the two gantries' four arm pairs at time t."""
    PA, PB = A.polys(t), B.polys(t)
    best = np.inf
    for X in PA:
        for Y in PB:
            d = polyline_min_dist(X[:, None], Y[None, :])
            d = np.where(np.isnan(d), np.inf, d)
            best = min(best, float(d.min()))
    return best


def arm_first_block(A, B, t_lo, t_hi, c_arm=C_ARM, eps=sc.EPS_CERT):
    """Earliest t in [t_lo, t_hi] where ARM_BLOCK holds, or None.

    Same certificate as sched_coll.first_block and for the same reason: no
    point of either arm can close faster than V_REL_ARM, so a separation d
    buys (d - c) / V_REL_ARM seconds. Conservative in one direction only --
    it may declare a block when the true clearance lies in (c, c + eps].

    Configuration CHANGES are instantaneous in this model (arm motion inside a
    pose costs 0, p1_g7 A4.3), so the walk is restarted at every dwell boundary
    of either gantry: a Lipschitz step cannot be allowed to stride across a
    discontinuity it does not model.
    """
    marks = sorted({t_lo, t_hi}
                   | {x for w in A.win for x in w[:2] if t_lo < x < t_hi}
                   | {x for w in B.win for x in w[:2] if t_lo < x < t_hi}
                   | {x for tr in (A.tr, B.tr) for ts, _, _, T in tr.legs
                      for x in (ts, ts + T) if t_lo < x < t_hi})
    for lo, hi in zip(marks, marks[1:] + [t_hi]):
        if hi < lo:
            continue
        t = lo
        while t <= hi:
            d = arm_distance(A, B, t)
            if d <= c_arm + eps:
                return t
            if not (A.moving(t) or B.moving(t)):
                break                      # nothing moves in this piece
            t += (d - c_arm) / V_REL_ARM
        if arm_distance(A, B, hi) <= c_arm + eps:
            return hi
    return None


def arm_schedule_conflict(inst, geom, stops, c_arm=C_ARM, eps=sc.EPS_CERT):
    """Earliest ARM_BLOCK violation in a whole two-gantry schedule, or None."""
    gs = sorted(stops)
    if len(gs) < 2:
        return None
    A = ArmTraj(inst, geom, gs[0], stops[gs[0]])
    B = ArmTraj(inst, geom, gs[1], stops[gs[1]])
    return arm_first_block(A, B, 0.0, max(A.end_time(), B.end_time()),
                           c_arm, eps)


# ===========================================================================
# A3-U2 -- the S2 probe that actually binds
# ===========================================================================
SIN31 = 0.525          # (N1): |sin rot| >= 0.525 (31.7 deg) on BOTH gantries


def rot_dominance(maps=('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz'),
                  sin_min=SIN31):
    """Per node, the fraction of its feasible gantry poses that satisfy (N1).

    This is the measurement that decides whether A3-U2's locked probe design is
    BUILDABLE at all, and it is run before the probe rather than after it fails.
    """
    caps = {g: CapabilityMap.load(maps[g - 1]) for g in (1, 2)}
    ref = caps[1]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    hot = np.abs(np.sin(poses[:, 1])) >= sin_min
    N = len(ref.nodes)
    m = np.zeros((N, len(poses)), bool)
    for g in (1, 2):
        r, _, _ = sched._gantry_oracles(caps[g], np.arange(N))
        m |= r.any(axis=2)
    live = m.any(axis=1)
    frac = np.where(m.sum(1) > 0, (m & hot).sum(1) / np.maximum(m.sum(1), 1),
                    0.0)
    return poses, hot, m, live, frac


def gen_real_rotcrowded(n_tasks, seed, n_mr=0, gantries=(1, 2), x_c=0.80,
                        maps=('/tmp/cap_g1_rail160.npz',
                              '/tmp/cap_g2_rail160.npz'),
                        sin_min=SIN31, band=0.30, max_reject=200_000):
    """S2, rebuilt. p1_g10 B7 measured why the old one could not bind.

    `gen_real_crowded` narrowed the task pool in `lin`; (N1) demands `rot`. The
    result was 40/40 instances answered by Lemma 4 with the coupled solver never
    running, and D10/D17 dodging judgement for two sessions.

    The binding shape is already proven in verify_sched_coupled.gen_small_crowded
    (15 of 31 bind). Ported to the REAL map, per A3-U2:

      p0 for both gantries  = grid pose nearest (lin = 0.80, rot = 0), which is
                              universally safe (Lemma 1), so the start state is
                              collision-free and Lemma 1's serialisation exists
      node pool             = nodes whose feasible-pose set is DOMINATED by
                              |sin rot| >= sin_min
      rejected              = any node feasible at some pose with |rot| < 31.7 deg

    🔺 CONTRADICTION with A3-U2, MEASURED before the probe was written and not
    after it failed (`rot_dominance`, reported in B): the locked design cannot
    be built on the real map. It asks for nodes whose feasible-pose set is
    DOMINATED by |sin rot| >= 0.525 and rejects any node reachable at some
    rail-parallel pose. Measured over all 3132 live nodes:

        hot poses in the grid                        1518 / 2376 = 63.9 %
        nodes with hot fraction 1.00 / >= 0.90 / >= 0.70      0 / 0 / 0
        hot fraction, p5 .. p95                      0.487 .. 0.626
        |G(t)| median                                1501 of 2376 poses

    i.e. every task's feasible-pose set sits at the grid's own base rate, and
    NO task forces a rotation. The mechanism is geometric and worth stating:
    the arm reaches 1.06 m about a rotation axis its mount plate orbits at only
    0.40 m, so rotating the gantry re-aims a reach that already covers the
    target. Reachability on this cell is very nearly rotation-invariant.

    Consequently (N1) -- which needs BOTH gantries more than 31.7 deg off
    rail-parallel -- cannot be forced by any choice of TASKS on this map. The
    probe therefore forces it the only way left, and says so out loud: it
    restricts the CANDIDATE POSE SET, the axis p1_g7 A2-K1 pre-authorised for
    exactly this kind of declaration.

        p0            = grid pose nearest (x_c, rot = 0), universally safe
        candidate set = {p0} U {p : |sin rot| >= sin_min, |lin - x_c| <= band}
        node pool     = nodes feasible inside that set, drawn from the band

    So the probe answers "is there ANY regime on this grid where structural
    coordination is expensive", which is the binary question p1_g10 C task 2
    poses -- but it answers it about a RESTRICTED cell, and every number from it
    carries that qualifier. It is a probe DESIGN, never a physical constant, and
    no S2 number is ever averaged into S1.
    """
    caps = {g: CapabilityMap.load(maps[g - 1]) for g in gantries}
    ref = caps[gantries[0]]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    p0_full = int(np.argmin(np.hypot(poses[:, 0] - x_c, poses[:, 1])))
    hot = ((np.abs(np.sin(poses[:, 1])) >= sin_min)
           & (np.abs(poses[:, 0] - x_c) <= band))
    keep = np.flatnonzero(hot)
    if p0_full not in keep:
        keep = np.sort(np.append(keep, p0_full))
    p0_idx = int(np.searchsorted(keep, p0_full))
    poses_r = poses[keep]

    pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - x_c) <= band)
    if not len(pool):
        raise ValueError('crowded band contains no nodes')
    rng = np.random.default_rng(seed)
    kinds = np.array(['MR'] * n_mr + ['SR'] * (n_tasks - n_mr), dtype='<U2')
    chosen, rejects = [], 0
    while len(chosen) < n_tasks:
        cand = int(pool[rng.integers(0, len(pool))])
        want_mr = kinds[len(chosen)] == 'MR'
        ok, at_p0 = False, False
        for g in gantries:
            r, _, hd = sched._gantry_oracles(caps[g], np.array([cand]))
            m = hd[0][keep] if want_mr else r[0][keep].any(axis=1)
            ok |= bool(m.any())
            at_p0 |= bool(m[p0_idx])
        # A task doable AT p0 is done at p0, where rot = 0 and (N1) fails, so
        # the instance never enters the regime the probe exists to test. This
        # is gen_small_crowded's "tasks feasible ONLY at the +-90 deg poses",
        # ported. Without it the probe returned makespan 4-6 s with both
        # gantries standing still at p0 -- measured, then fixed.
        if at_p0:
            ok = False
        if ok and cand not in chosen:
            chosen.append(cand)
        else:
            rejects += 1
            if rejects > max_reject:
                raise RuntimeError('rotcrowded probe found no feasible tasks')
    node_idx = np.array(chosen)
    reach, zone, hand = {}, {}, {}
    for g in gantries:
        r, z, h = sched._gantry_oracles(caps[g], node_idx)
        reach[g], zone[g], hand[g] = r[:, keep], z[:, keep], h[:, keep]
    return sched.Instance(
        kinds, {g: poses_r for g in gantries}, reach, zone, hand,
        {g: p0_idx for g in gantries}, ref.nodes[node_idx], sched.DWELL, 0.0,
        f'rotcrowded(n={n_tasks},mr={n_mr},seed={seed})',
        dict(nodes=node_idx.tolist(), rejects=rejects, n_poses=len(poses_r),
             pose_keep=keep.tolist(), x_c=x_c, sin_min=sin_min, band=band))
