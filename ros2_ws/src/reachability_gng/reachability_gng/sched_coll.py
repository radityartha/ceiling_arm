"""Layer 3 -- gantry-gantry structural collision: the only TIME coupling.

docs/p1_g9_sched3.md A2. Everything here was locked in A BEFORE this file
existed. The model half of it lives here; the proof half lives in
test/verify_sched_coll.py (W0..W4), which shares no code with it.

Why this file exists at all: through G8 the two gantries coupled ONLY through
task allocation (p1_g7 B7.2). solve_exact solves each gantry independently and
takes the max, which is exact for that model and NOT exact once the structures
can hit each other. So every loss number through G8 is a LOWER bound, and this
is the hole (p1_state 6, p1_g7 A4.1, p1_g8 B10.1).

GEOMETRY -- copied from workcell_description/urdf/moving_table.urdf.xacro, not
invented (p1_state 7.2 forbids inventing constants):

    rotation_link   box 0.08 x 0.80 x 0.015, centred on the rotation axis,
                    long axis carried by `rot`                        (:54)
    mount plates    cylinder r = 0.055, at +-0.40 m along the bar     (:86-90)
    platform_link   box 0.35 x 0.34 x 0.065, does NOT rotate          (:23)
    rotation axis   vertical at (x = lin, y = y_g), y_1 = +0.36, y_2 = -0.36

    body_g(lin, rot) = OBB(centre (lin, y_g), axes u(rot) and u_perp,
                           half-sizes (0.40, 0.04))
                       U Disc((lin, y_g) + 0.40 u, 0.055)
                       U Disc((lin, y_g) - 0.40 u, 0.055)
    u(rot) = (cos rot, sin rot)

    BLOCK(p1, p2; c) <=> dist_XY(body_1(p1), body_2(p2)) <= c

Two things this is NOT, both of which would be wrong:

  * it is NOT the r = 0.4 swept disc that the prose in p1_state 6 / p1_g7 A4.1
    uses. That disc is the union over ALL rotations. It answers "can they ever
    meet"; as a PER-POSE predicate it forbids most of the pose space falsely.
  * it is NOT a capsule around the bar. The bar is 0.08 wide, the plates are
    0.11 across, so a 0.055 capsule would over-cover the bar flanks by 15 mm.
    The exact URDF footprint is cheap enough that the approximation buys
    nothing.

The 2-D reduction is EXACT, not a proxy (A2.1): bar and plates of both
gantries occupy the same z band (z in [-0.0975, -0.0325] relative to the
platform centre, and both platforms share z), and two bodies in one z band
intersect in 3-D iff their XY projections intersect. platform_link is dropped
by proof, not by assumption: the platforms span y_g +- 0.17, i.e. [0.19, 0.53]
and [-0.53, -0.19], never meeting each other, and the farthest a bar+plate
reaches across is y = 0.36 - 0.455 = -0.095, still 0.095 m clear of the other
platform's edge.

c_clear = 0.0 is the LOCKED default (A2.2). A safety margin was never measured
and 7.2 forbids inventing one -- exactly the treatment `T_fold` gets in
p1_g7 A4.3. Sensitivity is reported as a sweep instead.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from reachability_gng.sched import (T_LIN_OFFSET, T_ROT_OFFSET,  # noqa: F401
                                    V_LIN_MM_S, V_ROT_DEG_S)

# ---------------------------------------------------------------------------
# URDF constants. Every one of these is a citation, not a choice.
# ---------------------------------------------------------------------------
Y_GANTRY = {1: 0.36, 2: -0.36}      # irm_sweep.py:90, p1_g2 1
BAR_HALF_LEN = 0.40                 # 0.80 / 2
BAR_HALF_WID = 0.04                 # 0.08 / 2
PLATE_R = 0.055                     # mount_plate_radius
PLATE_OFF = 0.40                    # mount_plate_y_offset
R_MAX = PLATE_OFF + PLATE_R         # 0.455, farthest footprint point
Y_SEP = Y_GANTRY[1] - Y_GANTRY[2]   # 0.72

C_CLEAR = 0.0                       # LOCKED default, A2.2


# ---------------------------------------------------------------------------
# Footprint
# ---------------------------------------------------------------------------
def _frame(g, lin, rot):
    """(centre, u, v) for gantry g at pose (lin, rot). Broadcasting."""
    lin = np.asarray(lin, float)
    rot = np.asarray(rot, float)
    lin, rot = np.broadcast_arrays(lin, rot)
    c = np.stack([lin, np.full(lin.shape, Y_GANTRY[g])], axis=-1)
    u = np.stack([np.cos(rot), np.sin(rot)], axis=-1)
    v = np.stack([-np.sin(rot), np.cos(rot)], axis=-1)
    return c, u, v


def rect_verts(g, lin, rot):
    """(..., 4, 2) corners of the rotation bar, counter-clockwise in u/v."""
    c, u, v = _frame(g, lin, rot)
    hu, hv = BAR_HALF_LEN * u, BAR_HALF_WID * v
    return np.stack([c + hu + hv, c + hu - hv, c - hu - hv, c - hu + hv],
                    axis=-2)


def plate_centres(g, lin, rot):
    """(..., 2, 2) mount-plate centres. W0b ties these to irm_sweep.base_pose."""
    c, u, _ = _frame(g, lin, rot)
    return np.stack([c + PLATE_OFF * u, c - PLATE_OFF * u], axis=-2)


def half_extent(rot):
    """(h_x, h_y): footprint half-extent on each world axis. A2.2 (N1)/(N2).

    The plate radius always beats the bar half-width (0.055 > 0.04 >=
    0.04|cos|), so the max collapses to a single term on each axis.
    """
    rot = np.asarray(rot, float)
    return (PLATE_OFF * np.abs(np.cos(rot)) + PLATE_R,
            PLATE_OFF * np.abs(np.sin(rot)) + PLATE_R)


# ---------------------------------------------------------------------------
# Exact 2-D distance primitives
# ---------------------------------------------------------------------------
def _pt_seg(p, a, b):
    """Point-to-segment distance, broadcasting over leading axes."""
    ab = b - a
    den = np.maximum((ab * ab).sum(-1), 1e-300)
    t = np.clip(((p - a) * ab).sum(-1) / den, 0.0, 1.0)
    q = a + t[..., None] * ab
    return np.sqrt(((p - q) ** 2).sum(-1))


def _pt_obb(p, c, u, v, hu, hv):
    """Point-to-OBB distance (0 inside), broadcasting."""
    d = p - c
    a = np.maximum(np.abs((d * u).sum(-1)) - hu, 0.0)
    b = np.maximum(np.abs((d * v).sum(-1)) - hv, 0.0)
    return np.hypot(a, b)


def _convex_overlap(P, Q):
    """SAT: do the two convex polygons (..., k, 2) intersect?"""
    sep = np.zeros(np.broadcast_shapes(P.shape[:-2], Q.shape[:-2]), bool)
    for poly in (P, Q):
        e = np.roll(poly, -1, axis=-2) - poly
        n = np.stack([-e[..., 1], e[..., 0]], axis=-1)
        pp = np.einsum('...ka,...ma->...km', n, P)
        qq = np.einsum('...ka,...ma->...km', n, Q)
        sep |= (pp.min(-1) > qq.max(-1)).any(-1)
        sep |= (qq.min(-1) > pp.max(-1)).any(-1)
    return ~sep


def _poly_dist(P, Q):
    """Exact distance between two convex polygons (0 if they intersect)."""
    d = np.full(np.broadcast_shapes(P.shape[:-2], Q.shape[:-2]), np.inf)
    k = P.shape[-2]
    for A, B in ((P, Q), (Q, P)):
        for i in range(k):
            for j in range(k):
                d = np.minimum(d, _pt_seg(A[..., i, :], B[..., j, :],
                                          B[..., (j + 1) % k, :]))
    return np.where(_convex_overlap(P, Q), 0.0, d)


# ---------------------------------------------------------------------------
# The predicate
# ---------------------------------------------------------------------------
def may_block(lin1, rot1, lin2, rot2, c_clear=C_CLEAR):
    """Necessary conditions (N1) and (N2) -- a cheap, SOUND prefilter.

    Never False for a pair that actually blocks; W0 checks that on every case
    it sees. Useful because a full grid is 2376^2 pairs and almost none of
    them can possibly touch.
    """
    hx1, hy1 = half_extent(rot1)
    hx2, hy2 = half_extent(rot2)
    n1 = (Y_SEP - hy1 - hy2) <= c_clear
    n2 = (np.abs(np.asarray(lin1, float) - np.asarray(lin2, float))
          - hx1 - hx2) <= c_clear
    return n1 & n2


def pair_distance(lin1, rot1, lin2, rot2):
    """Exact XY distance between the two gantry footprints. Broadcasting.

    min over the 9 primitive pairs of (bar, plate, plate) x (bar, plate,
    plate). Written as the plain minimum rather than something clever: this
    function is what W0 attacks, so it should be readable, not fast.
    """
    R1 = rect_verts(1, lin1, rot1)
    R2 = rect_verts(2, lin2, rot2)
    D1 = plate_centres(1, lin1, rot1)
    D2 = plate_centres(2, lin2, rot2)
    c1, u1, v1 = _frame(1, lin1, rot1)
    c2, u2, v2 = _frame(2, lin2, rot2)

    d = _poly_dist(R1, R2)
    for k in range(2):
        d = np.minimum(d, np.maximum(
            _pt_obb(D2[..., k, :], c1, u1, v1, BAR_HALF_LEN, BAR_HALF_WID)
            - PLATE_R, 0.0))
        d = np.minimum(d, np.maximum(
            _pt_obb(D1[..., k, :], c2, u2, v2, BAR_HALF_LEN, BAR_HALF_WID)
            - PLATE_R, 0.0))
    for i in range(2):
        for j in range(2):
            gap = np.linalg.norm(D1[..., i, :] - D2[..., j, :], axis=-1)
            d = np.minimum(d, np.maximum(gap - 2 * PLATE_R, 0.0))
    return d


def blocked(lin1, rot1, lin2, rot2, c_clear=C_CLEAR):
    """BLOCK(p1, p2; c_clear) -- the predicate of A2.2."""
    return pair_distance(lin1, rot1, lin2, rot2) <= c_clear


def blocked_poses(poses1, poses2, c_clear=C_CLEAR, chunk=200):
    """(P1, P2) boolean BLOCK table for two pose grids (P, 2) = (lin, rot).

    Prefilters with may_block, so the exact test only runs where (N1)/(N2)
    allow a hit at all. Chunked because the full 2376 x 2376 exact test would
    allocate tens of GB of temporaries.
    """
    poses1 = np.asarray(poses1, float)
    poses2 = np.asarray(poses2, float)
    out = np.zeros((len(poses1), len(poses2)), bool)
    for s in range(0, len(poses1), chunk):
        a = poses1[s:s + chunk]
        l1 = a[:, 0][:, None]
        r1 = a[:, 1][:, None]
        l2 = poses2[None, :, 0]
        r2 = poses2[None, :, 1]
        cand = may_block(l1, r1, l2, r2, c_clear)
        if not cand.any():
            continue
        ii, jj = np.nonzero(cand)
        d = pair_distance(a[ii, 0], a[ii, 1], poses2[jj, 0], poses2[jj, 1])
        blk = np.zeros(cand.shape, bool)
        blk[ii, jj] = d <= c_clear
        out[s:s + chunk] = blk
    return out


# ---------------------------------------------------------------------------
# A2.3 -- the traverse TRAJECTORY.
#
# p1_state 5.6 locks the DURATION of a traverse and nothing more; before this
# session the path was never needed. Collision needs it, so A2.3 locked it:
# each axis is dead for its command offset, then runs at constant speed to the
# target, then holds. The two axes run concurrently -- that concurrency is
# where max(T_lin, T_rot) comes from in the first place.
#
# This is an INTERPRETATION of the offset (dead time, not acceleration ramp),
# chosen because it is the only reading consistent with the already-locked
# T(p, p) = 0, and because a ramp would need an acceleration constant that was
# never measured (7.2). Bias direction is known and stated in A2.3.
# ---------------------------------------------------------------------------
V_LIN_M_S = V_LIN_MM_S / 1000.0                  # 0.0314160 m/s
OMEGA_RAD_S = np.deg2rad(V_ROT_DEG_S)            # 0.1745329 rad/s
V_POINT = V_LIN_M_S + OMEGA_RAD_S * R_MAX        # 0.1108285 m/s
V_REL = 2.0 * V_POINT                            # 0.2216570 m/s

EPS_CERT = 0.005                                 # LOCKED in A2.4


def short_deg(d_rad):
    """Signed shortest rotation in DEGREES, in [-180, 180).

    Exactly 180 deg maps to -180: the two ways round are the same length, so
    the duration is unaffected and the tie-break only has to be deterministic.
    """
    d = np.degrees(np.asarray(d_rad, float))
    return (d + 180.0) % 360.0 - 180.0


def axis_times(p, q):
    """(T_lin, T_rot) for one traverse. Same constants as sched.traverse_time.

    Imported from sched rather than retyped, so the two can never drift; W1
    asserts max(T_lin, T_rot) == sched.traverse_time to 1e-12.
    """
    d_mm = abs(q[0] - p[0]) * 1000.0
    d_deg = abs(float(short_deg(q[1] - p[1])))
    t_lin = T_LIN_OFFSET + d_mm / V_LIN_MM_S if d_mm > 1e-6 else 0.0
    t_rot = T_ROT_OFFSET + d_deg / V_ROT_DEG_S if d_deg > 1e-6 else 0.0
    return t_lin, t_rot


def leg_duration(p, q):
    return max(axis_times(p, q))


def leg_pose(u, p, q):
    """Pose at elapsed time u into the traverse p -> q. Vectorised over u."""
    u = np.asarray(u, float)
    d_m = q[0] - p[0]
    d_deg = float(short_deg(q[1] - p[1]))
    if abs(d_m) > 1e-9:
        moved = np.minimum(np.maximum(u - T_LIN_OFFSET, 0.0) * V_LIN_M_S,
                           abs(d_m))
        lin = p[0] + np.sign(d_m) * moved
    else:
        lin = np.full(u.shape, p[0])
    if abs(d_deg) > 1e-9:
        moved = np.minimum(np.maximum(u - T_ROT_OFFSET, 0.0) * V_ROT_DEG_S,
                           abs(d_deg))
        rot = p[1] + np.deg2rad(np.sign(d_deg) * moved)
    else:
        rot = np.full(u.shape, p[1])
    return lin, rot


def _max_abs_sin(a, b):
    """max |sin| over the closed arc [a, b]. Exact: the maxima sit at pi/2 + k pi."""
    lo, hi = (a, b) if a <= b else (b, a)
    if np.floor((hi - np.pi / 2) / np.pi) >= np.ceil((lo - np.pi / 2) / np.pi):
        return 1.0
    return max(abs(np.sin(lo)), abs(np.sin(hi)))


@dataclass
class Traj:
    """One gantry's committed motion: hold at p0, then a list of traverses.

    Legs must be appended in non-decreasing start time and must not overlap;
    `append` enforces both. Before the first leg and after the last, the
    gantry holds -- which is what makes a Traj answer pose_at(t) for EVERY t,
    including times past the end. The coupled search leans on that: a gantry
    that has finished is still an obstacle.
    """
    g: int
    p0: tuple
    legs: list = field(default_factory=list)     # (t_start, p, q, T)

    def copy(self):
        return Traj(self.g, self.p0, list(self.legs))

    def append(self, t_start, q):
        p = self.end_pose()
        T = leg_duration(p, q)
        if self.legs and t_start < self.legs[-1][0] + self.legs[-1][3] - 1e-12:
            raise ValueError('overlapping legs')
        if T > 0.0:
            self.legs.append((float(t_start), p, tuple(q), T))
        return t_start + T

    def end_pose(self):
        return self.legs[-1][2] if self.legs else self.p0

    def end_time(self):
        return self.legs[-1][0] + self.legs[-1][3] if self.legs else 0.0

    def pose_at(self, t):
        t = np.asarray(t, float)
        lin = np.full(t.shape, self.p0[0], float)
        rot = np.full(t.shape, self.p0[1], float)
        for ts, p, q, T in self.legs:
            m = t >= ts
            if not m.any():
                continue
            l_, r_ = leg_pose(np.clip(t - ts, 0.0, T), p, q)
            lin = np.where(m, l_, lin)
            rot = np.where(m, r_, rot)
        return lin, rot

    def _span(self, t_lo, t_hi):
        """(lin interval, max |sin rot|) OVER-approximated on [t_lo, t_hi].

        Only ever used as a SOUND early-out, never as an answer -- that is the
        lesson B1 had to learn the hard way about (N1)/(N2). Over-approximating
        is therefore the safe direction: the endpoints are sampled exactly, and
        any leg that touches the window contributes its WHOLE range even if
        only part of it falls inside.
        """
        l_end, r_end = self.pose_at(np.array([t_lo, t_hi], float))
        lo, hi = float(l_end.min()), float(l_end.max())
        smax = float(np.abs(np.sin(r_end)).max())
        for ts, p, q, T in self.legs:
            if ts + T < t_lo or ts > t_hi:
                continue
            lo = min(lo, p[0], q[0])
            hi = max(hi, p[0], q[0])
            smax = max(smax, _max_abs_sin(
                p[1], p[1] + np.deg2rad(float(short_deg(q[1] - p[1])))))
        return (lo, hi), smax


def _cannot_meet(A, B, t_lo, t_hi, c_clear):
    """Sound early-out over a whole time window. False means "must test"."""
    (a_lo, a_hi), sa = A._span(t_lo, t_hi)
    (b_lo, b_hi), sb = B._span(t_lo, t_hi)
    if Y_SEP - (PLATE_OFF * sa + PLATE_R) - (PLATE_OFF * sb + PLATE_R) > c_clear:
        return True
    gap = max(0.0, max(a_lo - b_hi, b_lo - a_hi))
    return gap - 2 * R_MAX > c_clear


def first_block(A, B, t_lo, t_hi, c_clear=C_CLEAR, eps=EPS_CERT):
    """Earliest t in [t_lo, t_hi] where the two gantries BLOCK, or None.

    Conservative advancement (A2.4), not sampling-and-hoping. If the current
    separation is d, no point of either body can close more than V_REL per
    second, so nothing can reach clearance c before (d - c) / V_REL has
    elapsed -- the step is a CERTIFICATE, not a guess.

    `eps` is the termination guard and the only source of conservatism: the
    walk stops and declares a conflict once d <= c + eps, so the answer may be
    "blocked" when the true clearance sits in (c, c + eps]. It is NEVER wrong
    the other way. eps = 0.005 m is locked in A2.4, an order below the 0.05 m
    rail grid quantum.

    🔺 G10 FIX -- reported as a finding in p1_g10 B1, not filed as a tidy-up.
    `pair_distance(l1, r1, l2, r2)` puts its FIRST argument at y = +0.36 and its
    second at y = -0.36: the arguments are gantry-indexed, not interchangeable.
    Callers written as "my trajectory, the other one's" therefore evaluated a
    MIRRORED world whenever the caller was gantry 2 -- roughly half of every
    feasibility test the coupled search makes. Measured: a pose pair whose true
    clearance is 0.000 m (a real collision) reads 0.189 m clear when swapped,
    and the boolean flips in both directions on the real grid. Ordering the two
    trajectories by gantry id here fixes every caller at once, `_first_start`
    and `_repair_ub` included, which is where it actually bit.
    """
    if A.g > B.g:
        A, B = B, A
    if t_hi < t_lo or _cannot_meet(A, B, t_lo, t_hi, c_clear):
        return None
    t = t_lo
    while t <= t_hi:
        l1, r1 = A.pose_at(t)
        l2, r2 = B.pose_at(t)
        d = float(pair_distance(l1, r1, l2, r2))
        if d <= c_clear + eps:
            return t
        t += (d - c_clear) / V_REL
    return None


def conflict_free(A, B, t_lo=0.0, t_hi=None, c_clear=C_CLEAR, eps=EPS_CERT):
    """A2.4 for two committed trajectories over [t_lo, t_hi].

    t_hi defaults to the last motion either gantry makes: after that both hold
    and the pair distance is constant, so one extra test at that instant
    settles the whole infinite tail.
    """
    if t_hi is None:
        t_hi = max(A.end_time(), B.end_time())
    return first_block(A, B, t_lo, t_hi, c_clear, eps) is None


# ===========================================================================
# A2.5 + A3-K1 -- the coupled model and its exact solver
#
# 🔺 Two restrictions A did NOT anticipate. Both are written up as explicit
# contradictions in docs/p1_g9_sched3.md B; neither is silent.
#
#   (i)  EVASIVE MOVES. A assumed a gantry only ever travels to a pose where
#        it works -- true in the uncoupled model, false here: a gantry that
#        has finished can BLOCK the other and must move purely to get out of
#        the way, or instances get reported infeasible that are not. Evade
#        targets are restricted to keep_g & safe_g so the DP bound stays
#        addressable, and only the cheapest is tried.
#   (ii) START TIMES. A2.5 lets a gantry wait for any real duration and
#        Lemma 5 argues the useful set is finite. Computing "the instant a
#        blocked interval ends" exactly means solving a trig inequality along
#        the path. The start set used is
#            {own readiness} U {the other gantry's leg boundaries}
#        which terminates because the other gantry is static after its last
#        leg. W2 tests it against a dense start-time grid.
#
# So this solver is EXACT WITH RESPECT TO those candidate sets -- the language
# p1_g7 A2-K1 pre-authorised for pose sparsification, applied to another axis.
# Where B&B cannot close UB vs LB inside the budget it returns proved = False
# and the caller reports a BRACKET, the pattern p1_g8 A3-K3 established.
# ===========================================================================
import heapq                                                    # noqa: E402
import itertools                                                # noqa: E402
import time                                                     # noqa: E402

from reachability_gng import sched                              # noqa: E402

NODE_BUDGET = 200_000
ACTION_BUDGET = 400        # certificate walks per node; see B3
EVADE_CANDIDATES = 12      # cheapest safe poses tried when standing aside
UB_START_PROBES = 40       # extra start times probed by the UB constructor only


def safe_poses(poses, c_clear=C_CLEAR):
    """Poses that NO pose of the other gantry can block. Lemma 1's set.

    Contrapositive of (N1): the other gantry's y half-extent is at most R_MAX,
    so Y_SEP - h_y(rot) - R_MAX > c_clear means nothing can reach. Note the
    direction -- (N1) is NECESSARY for blocking, so its negation is SUFFICIENT
    for safety. That is the sound use of it; B1 records what the unsound use
    cost.
    """
    _, hy = half_extent(np.asarray(poses, float)[:, 1])
    return np.flatnonzero((Y_SEP - hy - R_MAX) > c_clear)


def traj_of(inst, g, stops):
    """Rebuild a gantry's trajectory from a G7/G8-shaped stop list."""
    tr = Traj(g, tuple(inst.poses[g][inst.p0[g]]))
    for st in stops:
        q = tuple(inst.poses[g][st['pose']])
        tr.append(st['start'] - leg_duration(tr.end_pose(), q), q)
    return tr


def schedule_conflict(inst, stops, c_clear=C_CLEAR, eps=EPS_CERT):
    """Earliest A2.4 violation in a whole two-gantry schedule, or None."""
    gs = sorted(stops)
    if len(gs) < 2:
        return None
    A, B = traj_of(inst, gs[0], stops[gs[0]]), traj_of(inst, gs[1], stops[gs[1]])
    return first_block(A, B, 0.0, max(A.end_time(), B.end_time()), c_clear, eps)


def _start_candidates(other, t_min):
    """{own readiness} U {other gantry's leg boundaries}. Restriction (ii)."""
    out = {round(t_min, 9)}
    for ts, _, _, T in other.legs:
        for t in (ts, ts + T):
            if t > t_min + 1e-9:
                out.add(round(t, 9))
    out.add(round(max(t_min, other.end_time()), 9))
    return sorted(out)


def _first_start(tr, q, dur, t_min, other, c_clear, eps, n_probe=0):
    """Earliest feasible start to traverse to q and hold `dur` there.

    Terminates: past other.end_time() the other gantry is static, so the whole
    action becomes a pure time shift and feasibility stops depending on s.

    `n_probe` adds a uniform grid of start times between t_min and the other
    gantry's last leg end, on top of the locked candidate set. Used ONLY by the
    upper-bound constructor, never by the exactness claim -- a feasible
    schedule is a valid UB however its start times were found, so probing here
    tightens the bracket without touching what `solve_coupled` may claim.

    It exists because the locked set {own readiness} U {other's leg boundaries}
    turns out to be far too coarse: on the binding instances the two gantries
    collide MID-TRAVERSE, both sweeping through rot ~ -90 deg at the same lin,
    and the fix is to delay one by a few seconds -- a time that is not any
    boundary of anything. That is Restriction (ii) biting, measured
    (p1_g9 B6).
    """
    T = leg_duration(tr.end_pose(), q)
    cands = _start_candidates(other, t_min)
    if n_probe:
        hi = max(t_min, other.end_time())
        if hi > t_min:
            cands = sorted(set(cands) | {
                round(t_min + (hi - t_min) * k / n_probe, 9)
                for k in range(1, n_probe)})
    for s in cands:
        cand = tr.copy()
        cand.append(s, q)
        if first_block(cand, other, s, s + T + dur, c_clear, eps) is None:
            return s, cand
    return None, None


def _split_lb(dps, gs, R, r, t):
    """min over task splits of max_g (t_g + h_g[A_g][p_g]) -- ADMISSIBLE.

    h_g[R] alone is NOT a lower bound when R is still shared: gantry g only
    has to do a SUBSET of R, and h_g is monotone, so h_g[R] over-estimates.
    Taking the min over splits is exactly the uncoupled optimum from this
    state, which Lemma 3 makes a valid lower bound on the coupled one.
    2**|R| <= 64 here, so the tightest bound is also the affordable one.
    """
    a, b = gs
    best = np.inf
    sub = R
    while True:
        v1 = dps[a].h[sub, r[a]]
        v2 = dps[b].h[R ^ sub, r[b]]
        if np.isfinite(v1) and np.isfinite(v2):
            best = min(best, max(t[a] + float(v1), t[b] + float(v2)))
        if sub == 0:
            break
        sub = (sub - 1) & R
    return best


def _repair_ub(inst, sol0, c_clear, eps, safe=None):
    """A genuinely feasible schedule: keep the uncoupled optimum's (pose, task
    set) choices and only push each action to its earliest feasible start.

    An incumbent for the B&B, and -- since a feasible schedule is a valid UPPER
    BOUND however it was found -- the thing that sets how tight the reported
    bracket is. Every number in it is a real traverse or a real dwell; no
    invented slack.

    `safe` enables ONE extra move that the uncoupled schedule never contains: a
    gantry that has run out of tasks steps aside to the nearest universally
    safe pose. Without it this deadlocks exactly when the collision binds --
    the finished gantry parks in the way and the other can never reach its
    pose -- which is why the first G9 sweep fell back to the very loose
    serialisation bound on all 13 binding instances (p1_g9 B6).

    This is an upper-BOUND constructor, not a scheduler: nothing it produces is
    ever reported as an optimum or as a heuristic result, so it does not
    trespass on p1_state 7.1.
    """
    gs = inst.gantries
    trajs = {g: Traj(g, tuple(inst.poses[g][inst.p0[g]])) for g in gs}
    todo = {g: list(sol0.stops[g]) for g in gs}
    t = {g: 0.0 for g in gs}
    fin = {g: 0.0 for g in gs}
    out = {g: [] for g in gs}
    stood_aside = {g: False for g in gs}

    def _try(g, h):
        """Commit g's next stop at its earliest feasible start, or return False."""
        st = todo[g][0]
        q = tuple(inst.poses[g][st['pose']])
        s_, newtr = _first_start(trajs[g], q, st['dur'], t[g], trajs[h],
                                 c_clear, eps, n_probe=UB_START_PROBES)
        if s_ is None:
            return False
        todo[g].pop(0)
        arrive = s_ + leg_duration(trajs[g].end_pose(), q)
        trajs[g] = newtr
        t[g] = fin[g] = arrive + st['dur']
        out[g].append(dict(st, start=arrive))
        return True

    while any(todo[g] for g in gs):
        ready = sorted((k for k in gs if todo[k]), key=lambda k: (t[k], k))
        placed = False
        # Try the earliest-ready gantry, then the OTHER one. That second try is
        # the fix that matters: _first_start only sees the other gantry's
        # COMMITTED trajectory, so a gantry that still has work but has not
        # planned its next leg looks parked forever, and waiting can never
        # clear it. Letting it move first can. (The first attempt at this only
        # handled a FINISHED gantry standing in the way, and changed nothing on
        # any of the 13 binding instances -- p1_g9 B6.)
        for g in ready:
            h = [k for k in gs if k != g][0]
            if _try(g, h):
                placed = True
                break
        if placed:
            continue
        # Both blocked: let an idle gantry step aside, once each.
        for g in gs:
            h = [k for k in gs if k != g][0]
            if stood_aside[g] or not safe:
                continue
            stood_aside[g] = True
            cur = trajs[g].end_pose()
            cand = sorted(safe[g], key=lambda p: leg_duration(
                cur, tuple(inst.poses[g][p])))[:EVADE_CANDIDATES]
            moved = False
            for p_safe in cand:
                qg = tuple(inst.poses[g][p_safe])
                if qg == cur:
                    break
                s_g, tr_g = _first_start(trajs[g], qg, 0.0, t[g], trajs[h],
                                         c_clear, eps)
                if s_g is not None:
                    trajs[g] = tr_g
                    t[g] = s_g + leg_duration(cur, qg)
                    # 🔺 G10 FIX, reported in p1_g10 B1. The stand-aside was
                    # committed to `trajs` but never to `out`, so the schedule
                    # this function RETURNED omitted a leg the trajectory it
                    # validated contained. Everything downstream -- the gate,
                    # the replay, the reported optimum -- then saw a gantry
                    # standing where it no longer was, and the reported
                    # schedule could collide even though the checked one did
                    # not. Caught by the wait-aware gate on Q3.
                    out[g].append(dict(pose=p_safe, start=t[g], dur=0.0,
                                       tasks=0, assign={}))
                    moved = True
                    break
            if moved and any(_try(k, [m for m in gs if m != k][0])
                             for k in sorted((x for x in gs if todo[x]),
                                             key=lambda x: (t[x], x))):
                placed = True
                break
        if not placed:
            return np.inf, None
    return max(fin.values()), out


def _serial_ub(inst, sol0, safe, c_clear, eps):
    """Lemma 1 made concrete: a genuinely feasible schedule, always.

    Gantry a runs its whole uncoupled schedule while b sits at p0; a then parks
    somewhere universally safe; b then runs its own schedule, time-shifted.
    Feasible by construction because (a) p0 is rot = 0 for gen_real and so is
    universally safe, and (b) a safe pose blocks nothing by definition. No
    invented slack constant anywhere -- every number is a real traverse.

    This exists because _repair_ub can deadlock, and a B&B with no incumbent
    prunes nothing and grinds (measured: 2.2 s per node before this).
    """
    gs = inst.gantries
    a, b = gs
    p0a, p0b = inst.p0[a], inst.p0[b]
    if not (p0a in set(safe[a]) and p0b in set(safe[b])):
        return np.inf, None
    fin_a = max((s['start'] + s['dur'] for s in sol0.stops[a]), default=0.0)
    if not sol0.stops[b]:
        return fin_a, {a: list(sol0.stops[a]), b: []}
    end_a = sol0.stops[a][-1]['pose'] if sol0.stops[a] else p0a
    park = min(safe[a], key=lambda p: leg_duration(
        tuple(inst.poses[a][end_a]), tuple(inst.poses[a][p])))
    shift = fin_a + leg_duration(tuple(inst.poses[a][end_a]),
                                 tuple(inst.poses[a][park]))
    # the park is a real leg and must appear in the schedule -- same defect,
    # same G10 fix as in _repair_ub above: gantry b's whole schedule is shifted
    # on the assumption that a got out of the way, so a schedule that omits the
    # park is not the schedule whose feasibility was argued.
    stops = {a: list(sol0.stops[a]) + [dict(pose=int(park), start=shift,
                                            dur=0.0, tasks=0, assign={})],
             b: [dict(s, start=s['start'] + shift) for s in sol0.stops[b]]}
    return max(fin_a, stops[b][-1]['start'] + stops[b][-1]['dur']), stops


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

    @property
    def bracket(self):
        return self.makespan / self.lb if self.lb > 0 else float('nan')

    @property
    def delta(self):
        return self.makespan - self.lb


def solve_coupled(inst, c_clear=C_CLEAR, eps=EPS_CERT, budget=NODE_BUDGET,
                  time_budget=120.0, force_bnb=False, ub_seed=None,
                  action_budget=ACTION_BUDGET):
    """Exact minimum makespan under A2.4, w.r.t. the candidate sets above.

    Route 1 (Lemma 4): if the UNCOUPLED optimum's schedule is already
    collision-free it IS the coupled optimum -- Lemma 3 makes the uncoupled
    optimum a valid lower bound and this touches it. One replay. How often
    this fires is reported as a number (K5.3), because "always" would mean the
    search below was never exercised at all (the p1_g8 B1 lesson).

    Route 2: best-first branch & bound over joint states, LB from _split_lb.
    Popping in LB order means the first goal node popped is optimal.
    """
    t0 = time.time()
    gs = inst.gantries
    dps = {g: sched.solve_gantry(inst, g) for g in gs}
    sol0 = sched.solve_exact(inst)
    lb = sol0.makespan

    if len(gs) < 2:
        return CoupledSolution(sol0.makespan, lb, True, sol0.stops,
                               sol0.finish, time.time() - t0, 0, 'single')
    # force_bnb exists for W2b only: with collisions disabled the B&B must
    # reproduce sched.solve_exact, and the Lemma 4 shortcut would otherwise
    # answer without the search ever running.
    if not force_bnb and \
            schedule_conflict(inst, sol0.stops, c_clear, eps) is None:
        return CoupledSolution(sol0.makespan, lb, True, sol0.stops,
                               sol0.finish, time.time() - t0, 0, 'lemma4')

    a, b = gs
    keep_idx = {g: {int(p): i for i, p in enumerate(dps[g].keep)} for g in gs}
    safe = {g: [int(p) for p in safe_poses(inst.poses[g], c_clear)
                if int(p) in keep_idx[g]] for g in gs}
    full = (1 << inst.n) - 1

    best_m, best_stops = _repair_ub(inst, sol0, c_clear, eps, safe)
    if not np.isfinite(best_m):
        best_m, best_stops = _serial_ub(inst, sol0, safe, c_clear, eps)
    if ub_seed is not None:          # W2b only: force the search to work
        best_m, best_stops = float(ub_seed), None
    n_nodes, n_evade, exhausted = 0, 0, False
    counter = itertools.count()
    t_root = {g: 0.0 for g in gs}
    r_root = {g: keep_idx[g][inst.p0[g]] for g in gs}
    heap = [(lb, next(counter), t_root, dict(t_root), r_root,
             {g: Traj(g, tuple(inst.poses[g][inst.p0[g]])) for g in gs},
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

        g = min(gs, key=lambda k: (t[k], k))
        h = b if g == a else a
        dp, other = dps[g], trajs[h]

        # R is still SHARED, so dp.w[R] / dp.h[R^U] are NOT usable here: they
        # price gantry g doing the whole remainder alone, which is usually
        # infeasible (inf) and would reject every action. Same conflation
        # _split_lb documents; fixing it in only one of the two places was the
        # first real bug of this session.
        #
        # The bound below is the successor's _split_lb, computed VECTORISED
        # over all candidate poses at once, and evaluated BEFORE the start-time
        # search. That ordering is the whole performance story: _first_start
        # walks a certificate, so it must only ever run on actions that already
        # survived pruning. Total vector ops per node is sum_U 2^|R\U| = 3^|R|
        # <= 729, against 2368 * 64 certificate walks before.
        Tg = dp.T[r[g]]
        dp_h_g, dp_h_h = dps[g].h, dps[h].h
        actions = []
        U = R
        while U:
            d = dp.dur[U, :]
            end = t[g] + Tg + d
            R2 = R ^ U
            split = np.full(end.shape, np.inf)
            A = R2
            while True:
                v2 = dp_h_h[R2 ^ A, r[h]]
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

        # p1_g8 B4 all over again, in a new place: the expensive thing is the
        # CHECKER, not the scheduler. _first_start walks a certificate, and a
        # node with a loose incumbent can generate ~1.5e5 actions, so a single
        # expansion ran for minutes and the loop-top deadline never got a look
        # in. Budget it here, and DOWNGRADE to a bracket rather than pretend.
        produced = False
        for k, (_, r2, U, d) in enumerate(actions):
            if k >= action_budget or time.time() - t0 > time_budget:
                exhausted = True
                break
            p2 = int(dp.keep[r2])
            q = tuple(inst.poses[g][p2])
            s, newtr = _first_start(trajs[g], q, d, t[g], other, c_clear, eps)
            if s is None:
                continue
            arrive = s + leg_duration(trajs[g].end_pose(), q)
            end = arrive + d
            nt, nf, nr = dict(t), dict(fin), dict(r)
            nt[g] = nf[g] = end
            nr[g] = r2
            ntr, nst = dict(trajs), dict(stops)
            ntr[g] = newtr
            nst[g] = stops[g] + [dict(
                pose=p2, start=arrive, dur=d, tasks=U,
                assign=sched.stop_duration(inst, g, U, p2)[1])]
            R2 = R ^ U
            # recompute with the ACTUAL start (waiting can only push it later)
            nlb = max(max(nf.values()), _split_lb(dps, gs, R2, nr, nt))
            if nlb < best_m - 1e-9:
                heapq.heappush(heap, (nlb, next(counter), nt, nf, nr, ntr,
                                      nst, R2))
                produced = True

        # Restriction (i): every action was blocked, and the other gantry is
        # parked in the way. Let it step aside to the cheapest safe pose.
        # Skipped when h already sits somewhere universally safe -- then h is
        # not what is blocking g, and evading again could loop forever.
        h_safe = int(dp_pose_index(inst, h, trajs[h].end_pose())) in safe[h]
        if not produced and not h_safe:
            for p_safe in sorted(safe[h],
                                 key=lambda p: leg_duration(
                                     trajs[h].end_pose(),
                                     tuple(inst.poses[h][p]))):
                q = tuple(inst.poses[h][p_safe])
                if q == trajs[h].end_pose():
                    continue
                s, newtr = _first_start(trajs[h], q, 0.0, t[h], trajs[g],
                                        c_clear, eps)
                if s is None:
                    continue
                nt, nr = dict(t), dict(r)
                nt[h] = s + leg_duration(trajs[h].end_pose(), q)
                nr[h] = keep_idx[h][p_safe]
                ntr = dict(trajs)
                ntr[h] = newtr
                nlb = max(max(fin.values()), _split_lb(dps, gs, R, nr, nt))
                if nlb < best_m - 1e-9:
                    n_evade += 1
                    heapq.heappush(heap, (nlb, next(counter), nt, dict(fin),
                                          nr, ntr, dict(stops), R))
                break

    finish = {g: (best_stops[g][-1]['start'] + best_stops[g][-1]['dur'])
              if best_stops and best_stops.get(g) else 0.0 for g in gs}
    return CoupledSolution(best_m, lb, not exhausted, best_stops or {},
                           finish, time.time() - t0, n_nodes, 'bnb', n_evade)


def dp_pose_index(inst, g, pose):
    """Grid index of a (lin, rot) pose. Exact lookup -- poses come off the grid."""
    P = inst.poses[g]
    d = np.abs(P[:, 0] - pose[0]) + np.abs(P[:, 1] - pose[1])
    return int(np.argmin(d))


# ---------------------------------------------------------------------------
# A3-K2 -- the S2 adversarial probe generator
# ---------------------------------------------------------------------------
def gen_real_crowded(n_tasks, seed, n_mr=0, gantries=(1, 2), band=0.30,
                     x_c=0.80, maps=('/tmp/cap_g1_rail160.npz',
                                     '/tmp/cap_g2_rail160.npz')):
    """`gen_real`, but with the task pool confined to a narrow x band and both
    gantries starting at the same mid-rail pose.

    (N1)+(N2) prove collision needs BOTH gantries rotated well off rail-parallel
    AND their lin close together. Nothing guarantees `gen_real` ever produces
    that, and a session that reports "never binds" without once testing the
    regime where it CAN bind repeats the gate-that-never-fired weakness of
    p1_g8 B1. `band` and `x_c` are the DESIGN of a probe, not physical
    constants -- A3-K2 says so, and no number from S2 is ever averaged into S1.
    """
    from reachability_gng.capability import CapabilityMap
    caps = {g: CapabilityMap.load(maps[g - 1]) for g in gantries}
    ref = caps[gantries[0]]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    p0_idx = int(np.argmin(np.hypot(poses[:, 0] - x_c, poses[:, 1])))

    pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - x_c) <= band)
    if len(pool) == 0:
        raise ValueError('crowded band contains no nodes')
    rng = np.random.default_rng(seed)
    kinds = np.array(['MR'] * n_mr + ['SR'] * (n_tasks - n_mr), dtype='<U2')
    chosen, rejects = [], 0
    while len(chosen) < n_tasks:
        cand = int(pool[rng.integers(0, len(pool))])
        want_mr = kinds[len(chosen)] == 'MR'
        ok = False
        for g in gantries:
            r, _, hd = sched._gantry_oracles(caps[g], np.array([cand]))
            ok |= bool(hd[0].any()) if want_mr else bool(r[0].any())
        if ok:
            chosen.append(cand)
        else:
            rejects += 1
            if rejects > 10_000:
                raise RuntimeError('crowded generator found no feasible tasks')
    node_idx = np.array(chosen)
    reach, zone, hand = {}, {}, {}
    for g in gantries:
        reach[g], zone[g], hand[g] = sched._gantry_oracles(caps[g], node_idx)
    return sched.Instance(
        kinds, {g: poses for g in gantries}, reach, zone, hand,
        {g: p0_idx for g in gantries}, ref.nodes[node_idx], sched.DWELL, 0.0,
        f'crowded(n={n_tasks},mr={n_mr},seed={seed})',
        dict(nodes=node_idx.tolist(), rejects=rejects, n_poses=len(poses),
             band=band, x_c=x_c))
