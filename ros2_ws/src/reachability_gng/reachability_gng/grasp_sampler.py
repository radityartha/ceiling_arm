"""Antipodal grasp sampler for the wrist camera (Isaac grasping, session B).

Serves `~/sample_grasps` (reachability_gng_interfaces/SampleGrasps): takes the
wrist depth cloud AS SEEN FROM THE ARM'S CURRENT LOOK POSE, isolates the target
object in it, and returns ranked antipodal grasp poses for the gripper.

    ros2 run reachability_gng grasp_sampler
    ros2 service call /grasp_sampler/sample_grasps \
        reachability_gng_interfaces/srv/SampleGrasps "{target: '5'}"

Why a SERVICE and not a topic: a topic hands the caller whatever was published
last, which may describe a scene, an object identity, or an arm pose that has
since changed -- the exact shape of the stale-target bugs this stack has already
been bitten by. A service computes from frames captured during the caller's own
call and either succeeds with fresh geometry or fails loudly.

Grasp geometry comes from the WRIST camera only (locked design decision): the
ceiling cameras keep their localization role and are used here for one thing
only -- a rough object position to crop around. A wrong ceiling label therefore
means "we looked at the wrong object", never "we grasped with wrong geometry".

PIPELINE
    1. accumulate `captures` wrist cloud frames (stop-and-stare; the caller is
       expected to have the arm stopped at a look pose already)
    2. drop the robot's OWN HAND from the cloud -- see "self-filtering" below
    3. crop to `roi` about the ceiling-reported object position
    4. drop the support surface (table) if a dominant horizontal plane is there
    5. cluster what remains; the cluster NEAREST the reported position wins
       (nearest, not largest: with two adjacent objects, largest picks the wrong
       one whenever the neighbour is bigger)
    6. PCA the cluster -> principal axis + extents
    7. per-point surface normals from a FIXED-K nearest-neighbour query
    8. antipodal contact pairs, filtered by pad gap and opposed normals, scored
       by antipodality + gravity alignment, de-duplicated, ranked

SELF-FILTERING (step 2) is NOT optional. depth_cloud performs no robot-body
masking whatsoever -- it deprojects every depth pixel that passes a scalar
min/max range gate, with no link geometry and no segmentation involved. That is
by design for its original job (a geometry-only feed for the topo map) but it
means a WRIST camera, which is bolted to the hand and stares straight past the
fingers, returns a cloud in which the hand itself is a large fraction of the
points: measured live on this scene, ~36% of the wrist depth image sits at or
inside the 8 cm mark and the fingers are a solid band beyond that, with points
landing within a millimetre of the finger link origins. Left in, the hand is a
dense, conveniently object-sized blob right where the object should be, and it
would be clustered, PCA'd and grasped like any other geometry -- silently.

FIXED-K NEIGHBOURS, NOT RADIUS QUERIES: neighbour search here is a fixed-k
argpartition over a dense distance block, never scipy's `query_ball_point`. A
radius query in this repo has already hung on a dense frame and frozen a
single-threaded executor; a fixed-k query cannot, because its work per point is
bounded no matter how dense the cloud gets. The cluster is voxel-downsampled to
`max_points` first, which bounds that block's size too.

The geometry core (`sample_grasps_from_cloud` and everything above it) is pure
numpy and takes no ROS types, so it can be run against synthetic clouds of known
dimension -- which is how it is checked, rather than by eyeballing RViz.
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np

# The ROS half is optional at IMPORT time so the geometry core stays runnable on
# its own -- that is the whole point of keeping it free of ROS types, and it is
# how test/verify_grasp_sampler.py checks the geometry against known dimensions
# without a workspace build, a robot, or a running graph. Missing ROS only costs
# the node; every function above `class GraspSampler` still imports and runs.
try:
    import rclpy
    from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Vector3
    from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                       ReentrantCallbackGroup)
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                           qos_profile_sensor_data)
    from rclpy.time import Time
    from std_msgs.msg import String
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                         LookupException, TransformListener)

    from reachability_gng_interfaces.srv import SampleGrasps
    from reachability_gng.gantry_reach_executor import R_to_quat, _cloud_xyz
    from reachability_gng.object_localizer import quat_to_R
    _HAVE_ROS = True
except ImportError:                     # geometry-core-only import
    _HAVE_ROS = False
    Node = object

# Default geometry config. Every entry is also a ROS parameter of the same name;
# the dict is what the pure core consumes so it can run without a ROS node.
DEFAULTS = dict(
    roi=0.15,               # m, crop radius about the reported object position
    captures=3,             # wrist frames to accumulate per call
    max_candidates=10,      # grasps returned
    # --- self-filter: the gripper's own envelope, in gripper_base_link local
    # coords (local +Z points at the fingertips, measured +0.116 m; local Y is
    # the finger-spread axis). A cylinder about the approach axis reaching just
    # past the fingertips, with no lower bound -- everything behind the
    # fingertip plane inside that column is hand, wrist or forearm.
    gripper_radius=0.10,    # m
    gripper_ahead=0.16,     # m along local +Z, past the 0.116 m fingertips
    # --- support-plane removal
    plane_bin=0.005,        # m, z-histogram bin
    # Deliberately a LOW bar: how big a share of the crop the tabletop wins
    # depends entirely on how much object surface happens to face the camera,
    # so a high fraction silently fails to fire on exactly the tall objects
    # that hide the most table. plane_min_span below is the real discriminator;
    # this only rejects noise.
    plane_frac=0.03,        # modal bin must hold this share of ROI points
    plane_tol=0.012,        # m, cut this far above the plane
    plane_min_span=0.10,    # m, a support surface spans wider than any object
                            # that fits the 0.085 m pads
    # --- clustering / downsampling
    cluster_res=0.012,      # m, voxel size for connected components
    min_cluster_points=40,
    voxel=0.004,            # m, thinning voxel for the normal/neighbour set
    max_points=700,         # bound on the dense distance block (700^2 floats)
    voxel_fine=0.002,       # m, voxel for the set widths are MEASURED on; no
                            # point cap, since an extent read off a thinned
                            # sample is systematically narrow (see
                            # antipodal_candidates)
    # --- normals / contacts
    knn=12,                 # FIXED k -- never a radius query, see module docs
    max_width=0.085,        # m, Gen3 Lite 2F open pad gap (measured in sim)
    min_width=0.012,        # m, below this the fingers foul each other
    antipodal_min_deg=25.0, # max deviation of each contact normal from the axis
    support_radius=0.015,   # m, a neighbour this close counts as real support
    min_support=4,          # neighbours needed to call a contact "observed"
    # --- candidate generation (see antipodal_candidates)
    n_axes=18,              # closing axes per sweep, over 180 deg
    extent_trim=0.005,      # fraction trimmed off each end when measuring width
    grasp_depth=0.020,      # m to sink below the observed top surface
    min_clearance=0.005,    # m to stay clear of the support below
    w_antipodal=0.35,
    w_gravity=0.30,
    w_fit=0.35,
    nms_dist=0.015,         # m, centres closer than this collapse...
    nms_angle_deg=20.0,     # ...unless their closing axes differ by more
    finger_offset=0.1156,   # m, gripper_base_link origin -> finger midpoint
)


# --------------------------------------------------------------------------
# pure geometry core -- no ROS types below this line until GraspSampler
# --------------------------------------------------------------------------

def voxel_downsample(pts, voxel):
    """One representative (the voxel centroid) per occupied voxel.

    Also collapses the exact duplicates that stop-and-stare accumulation
    produces: N captures of a static scene are N copies of the same surface, and
    duplicates would otherwise inflate every neighbour count and make a thin,
    poorly-seen surface look as well-supported as a solid one."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    n = inv.max() + 1
    out = np.zeros((n, 3))
    cnt = np.bincount(inv, minlength=n).astype(float)
    for a in range(3):
        out[:, a] = np.bincount(inv, weights=pts[:, a], minlength=n)
    return out / cnt[:, None]


def remove_support_plane(pts, cfg):
    """Drop a dominant horizontal support surface (the table the object sits on).

    Needed before clustering, not after: an object resting on a table is
    physically connected to it, so any connectivity-based clustering merges the
    two into one blob and the object's own geometry is then a rounding error in
    the PCA of a tabletop.

    Only cuts when a horizontal plane actually dominates -- a z-histogram whose
    modal bin holds at least `plane_frac` of the points. An object held in
    mid-air, or a view with no table in it, has no such spike and is returned
    untouched, instead of having its bottom sliced off by a blind percentile.

    Returns (kept_points, n_dropped).
    """
    if len(pts) < 20:
        return pts, 0
    z = pts[:, 2]
    lo, hi = z.min(), z.max()
    if hi - lo < cfg['plane_bin']:
        return pts, 0
    nb = max(1, int(np.ceil((hi - lo) / cfg['plane_bin'])))
    idx = np.minimum(((z - lo) / cfg['plane_bin']).astype(int), nb - 1)
    counts = np.bincount(idx, minlength=nb)

    # Search the LOWER half only. A box's own lid is flat, horizontal and often
    # holds more points than the tabletop does (it faces the camera squarely
    # while the table is partly occluded by the object standing on it), so a
    # global argmax finds the lid and then, correctly refusing to cut there,
    # gives up -- leaving the table in and letting it dominate the cluster PCA.
    # Support is by definition underneath, so only look underneath.
    half = max(1, int(np.ceil(0.5 * nb)))
    m = int(counts[:half].argmax())
    if counts[m] < cfg['plane_frac'] * len(pts):
        return pts, 0                      # no dominant plane -> nothing to cut
    plane_z = lo + (m + 0.5) * cfg['plane_bin']

    # A support surface SPANS the crop; a flat patch of the object itself does
    # not. Requiring horizontal span separates the two without needing to know
    # the object's size (any graspable object is <= the 0.085 m pad gap wide).
    band = pts[np.abs(z - plane_z) <= 2.0 * cfg['plane_bin']]
    if len(band) < 3:
        return pts, 0
    span = max(band[:, 0].max() - band[:, 0].min(),
               band[:, 1].max() - band[:, 1].min())
    if span < cfg['plane_min_span']:
        return pts, 0

    keep = z > plane_z + cfg['plane_tol']
    return pts[keep], int((~keep).sum())


def cluster_voxel(pts, res, min_points):
    """Connected components over occupied voxels (26-connectivity).

    Voxel connectivity rather than a radius query on purpose: the work is set
    lookups over occupied cells, bounded by the number of occupied voxels, so it
    cannot blow up on a dense frame the way a radius query can.

    Returns a list of index arrays, largest first.
    """
    if len(pts) == 0:
        return []
    keys = np.floor(pts / res).astype(np.int64)
    cells = {}
    for i, k in enumerate(map(tuple, keys)):
        cells.setdefault(k, []).append(i)

    offs = [(dx, dy, dz)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
            if (dx, dy, dz) != (0, 0, 0)]
    seen, clusters = set(), []
    for start in cells:
        if start in seen:
            continue
        seen.add(start)
        stack, member = [start], [start]
        while stack:
            c = stack.pop()
            cx, cy, cz = c
            for dx, dy, dz in offs:
                nb = (cx + dx, cy + dy, cz + dz)
                if nb in cells and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
                    member.append(nb)
        idx = np.concatenate([np.asarray(cells[c], dtype=int) for c in member])
        if len(idx) >= min_points:
            clusters.append(idx)
    clusters.sort(key=len, reverse=True)
    return clusters


def pick_cluster(pts, clusters, obj_xyz):
    """The cluster whose centroid is NEAREST the reported object position.

    Nearest, not largest: the reported position is the one piece of information
    that actually identifies WHICH object was asked for, and with two objects
    side by side "largest" silently grasps whichever happens to be bigger."""
    if not clusters:
        return None
    d = [np.linalg.norm(pts[idx].mean(axis=0) - obj_xyz) for idx in clusters]
    return clusters[int(np.argmin(d))]


def estimate_normals_knn(P, k, view_xyz):
    """Per-point surface normals from a FIXED-K neighbourhood (see module docs
    for why this is never a radius query).

    Returns (normals, knn_dist) where knn_dist[i] holds the distances to point
    i's k neighbours -- reused later as an observation-support measure, so the
    distance block is computed once and not thrown away.

    A covariance eigenvector has an arbitrary sign, so it has to be resolved
    against something physical or every antipodality score is a coin flip. The
    physical fact is that a depth sensor only ever returns the surface facing
    it, so the outward normal is the one pointing back at the camera that saw
    that point.

    `view_xyz` may therefore be a single camera position OR several: when clouds
    from more than one look pose are fused -- which is the whole point of taking
    an oblique second look -- each point is oriented toward its NEAREST view,
    not toward one global viewpoint. Orienting a fused cloud against a single
    camera flips the far side's normals inward, which destroys precisely the
    opposed-normal evidence the second look was taken to collect: the sampler
    would report both_sides_observed false no matter how many views it was
    given, and escalating would never terminate.
    """
    n = len(P)
    k = int(min(k, n - 1))
    if n < 3 or k < 2:
        return np.zeros((n, 3)), np.zeros((n, max(k, 1)))
    D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2)
    np.fill_diagonal(D, np.inf)
    nn = np.argpartition(D, k - 1, axis=1)[:, :k]           # fixed k, bounded
    knn_dist = np.take_along_axis(D, nn, axis=1)

    nbr = P[nn]                                             # (n, k, 3)
    nbr = nbr - nbr.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nbr, nbr) / k
    _, vecs = np.linalg.eigh(cov)                           # ascending
    normals = vecs[:, :, 0]                                 # smallest spread

    V = np.atleast_2d(np.asarray(view_xyz, float))
    if len(V) == 1:
        to_view = V[0][None, :] - P
    else:
        nearest = np.argmin(np.linalg.norm(P[:, None, :] - V[None, :, :], axis=2),
                            axis=1)
        to_view = V[nearest] - P
    flip = np.einsum('ni,ni->n', normals, to_view) < 0.0
    normals[flip] *= -1.0
    return normals, knn_dist


def grasp_frame(center, axis, finger_offset):
    """Gripper pose for a contact pair.

    `axis` is the closing direction, which must land on the gripper's local Y
    (the measured finger-spread axis). The approach is then the most DOWNWARD
    direction perpendicular to that axis -- the gripper comes down onto the
    object rather than sideways into the table it is standing on.

    Returns (position, R) with R's columns the gripper's local x, y, z, so local
    +Z is the approach (it points at the fingertips) and the returned position
    puts the FINGER MIDPOINT on `center`.
    """
    y = np.asarray(axis, float)
    y = y / np.linalg.norm(y)
    down = np.array([0.0, 0.0, -1.0])
    z = down - float(down @ y) * y                  # project out the axis
    nz = np.linalg.norm(z)
    if nz < 1e-6:
        # Closing axis is vertical, so no approach has a downward component;
        # any perpendicular direction is as good as another.
        alt = np.array([1.0, 0.0, 0.0])
        if abs(float(alt @ y)) > 0.9:
            alt = np.array([0.0, 1.0, 0.0])
        z = alt - float(alt @ y) * y
        nz = np.linalg.norm(z)
    z = z / nz
    x = np.cross(y, z)                              # right-handed: z = x cross y
    R = np.column_stack([x, y, z])
    return np.asarray(center, float) - finger_offset * z, R


def candidate_axes(principal, cfg):
    """Closing-axis directions to try.

    Two sweeps, de-duplicated later by NMS. The first is the plane perpendicular
    to the cluster's principal axis -- gripping across an object's long axis
    rather than along it. The second is simply the horizontal plane, because the
    principal axis of a SINGLE VIEW is not the object's: a view that catches a
    lid and one side wall has a principal axis lying diagonally across the two,
    and the perpendicular plane to that can miss every sensible horizontal grip.
    The horizontal sweep costs 18 more axes and removes that dependence on which
    faces happened to be visible.
    """
    principal = np.asarray(principal, float)
    principal = principal / np.linalg.norm(principal)
    n = int(cfg['n_axes'])
    out = []
    for axis in (principal, np.array([0.0, 0.0, 1.0])):
        tmp = np.array([1.0, 0.0, 0.0])
        if abs(float(tmp @ axis)) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0])
        b1 = tmp - float(tmp @ axis) * axis
        b1 /= np.linalg.norm(b1)
        b2 = np.cross(axis, b1)
        for m in range(n):
            th = np.pi * m / n              # a and -a are the same grasp
            out.append(np.cos(th) * b1 + np.sin(th) * b2)
    return out


def antipodal_candidates(P, normals, knn_dist, Q, principal, cfg):
    """Candidate grasps: one per closing axis, sized by the object's FULL extent
    across that axis.

    Why the full extent and not a local chord. A parallel-jaw gripper closes two
    flat pads on the object as a whole; the distance they must span is the
    object's entire width along the closing direction. Sizing a grasp from a
    thin slice of the cloud instead reports whatever chord that slice happens to
    cut -- so a diagonal across a rectangular box measures as a comfortable
    5 cm grip while the parts of the box outside the slice, which the fingers
    would actually collide with, are never counted. Measuring the full extent
    also makes the pad-gap filter do the discriminating for free: for a
    6x10 cm box the 6 cm axis measures 6 cm, the 10 cm axis measures 10 cm and
    the diagonals measure 11 cm, so only the axis that physically fits survives.

    Why candidates are inferred rather than paired off observed points: a depth
    sensor returns one side of an object at a time, so for a box seen from above
    the only surface in the cloud is the lid -- the two faces a gripper would
    pinch are, between them, never both visible from any single viewpoint.
    Requiring both contacts to be observed points yields no grasps at all for
    exactly the objects and viewpoints this sampler exists to serve. What one
    view DOES pin down exactly is the cross-section: the lid's own extent across
    an axis IS the span the fingers need, whether or not the walls below it were
    ever imaged.

    That inference is reported, not hidden. `both_sides_observed` is true only
    when both extremes carry genuinely observed surface -- enough close
    neighbours to be real rather than a lone noise spike, AND an outward normal
    that actually opposes the closing direction rather than a lid point standing
    in for a wall nobody saw. It is the trigger for an oblique second look.

    Two point sets are used on purpose. `P` is the thinned set carrying normals
    and neighbour distances; it has to stay small because that search is
    quadratic in it. `Q` is the dense cluster, and extents are measured on Q
    alone: an extent is a max minus a min, and the max of a sparse sample falls
    inside the true max, so measuring width on the thinned set underestimates
    every object by several millimetres, plausibly and invisibly.

    Scored terms:
      antipodal  how opposed the two contact normals are; near 0 when the far
                 side was never observed, near 1 for a truly measured pinch
      gravity    how horizontal the closing axis is; a vertical closing axis
                 holds the object between a top and a bottom pad, from which its
                 own weight slides it straight out
      fit        how much pad clearance is left over; among axes that all fit,
                 the narrower grip is the one with margin for error

    Returns a list of dicts, best first.
    """
    if len(P) < 3 or len(Q) < 3:
        return []

    support = (knn_dist <= cfg['support_radius']).sum(axis=1) >= cfg['min_support']
    cos_anti = np.cos(np.radians(cfg['antipodal_min_deg']))
    down = np.array([0.0, 0.0, -1.0])

    cands = []
    for a in candidate_axes(principal, cfg):
        a = a / np.linalg.norm(a)
        # Approach: the most downward direction perpendicular to the closing
        # axis -- the same convention grasp_frame builds the pose with.
        zc = down - float(down @ a) * a
        if np.linalg.norm(zc) < 1e-6:
            continue                              # closing axis vertical
        zc /= np.linalg.norm(zc)
        b = np.cross(zc, a)

        qa, qb, qz = Q @ a, Q @ b, Q @ zc
        # Trimmed extent, not raw min/max: range statistics sit exactly on the
        # tails, so the widest and narrowest points are whichever two samples
        # caught the most range noise, and a raw extent reads several
        # millimetres wide on every object -- always in the unsafe direction,
        # since the pads then close on a gap wider than the object really is.
        order = np.argsort(qa)
        trim = min(int(cfg['extent_trim'] * len(qa)), (len(qa) - 1) // 2)
        i_lo, i_hi = int(order[trim]), int(order[len(qa) - 1 - trim])
        a_lo, a_hi = float(qa[i_lo]), float(qa[i_hi])
        width = a_hi - a_lo
        if not (cfg['min_width'] <= width <= cfg['max_width']):
            continue

        # Sink below the topmost observed surface so the pads close on the body
        # rather than skidding off the top edge, but never past the lowest
        # observed point -- that is where the object meets whatever it stands on.
        # zc points DOWNWARD, so along it the top is the MINIMUM.
        z_top, z_bot = float(qz.min()), float(qz.max())
        depth = min(z_top + cfg['grasp_depth'],
                    max(z_top, z_bot - cfg['min_clearance']))
        centre = (0.5 * (a_lo + a_hi) * a
                  + 0.5 * float(qb.min() + qb.max()) * b
                  + depth * zc)

        # Normals and observation support live on the thinned set, so map each
        # dense contact onto its nearest thinned neighbour.
        j_lo = int(np.argmin(np.linalg.norm(P - Q[i_lo], axis=1)))
        j_hi = int(np.argmin(np.linalg.norm(P - Q[i_hi], axis=1)))

        # The closing axis is `a` -- the direction the pads travel -- NOT the
        # vector between the two extreme points. Those two points are the
        # extremes ALONG a, but they sit at unrelated positions in the other two
        # coordinates, so the line joining them can run off at a wide angle to
        # the direction the fingers actually close along. Scoring that line
        # instead of `a` rates a grasp by geometry the gripper never performs.
        anti = 0.5 * (-float(normals[j_lo] @ a) + float(normals[j_hi] @ a))
        grav = 1.0 - abs(float(a[2]))
        fit = 1.0 - width / cfg['max_width']
        both = bool(support[j_lo] and support[j_hi] and anti >= cos_anti)
        # Antipodality only counts toward the RANKING when both contacts were
        # actually observed. With one side unseen, one of the two normals
        # belongs to whatever surface the view did catch -- a lid point standing
        # in for a wall -- so the number is noise, and letting noise rank
        # candidates picks an axis a few degrees off the true one and hides it
        # behind a healthy-looking score. The raw value is still reported, so
        # the caller sees what was measured; it just does not get a vote it
        # cannot back up. Once an oblique look observes both sides, it does.
        anti_eff = anti if both else 0.0
        score = (cfg['w_antipodal'] * max(anti_eff, 0.0)
                 + cfg['w_gravity'] * grav
                 + cfg['w_fit'] * fit)
        cands.append(dict(
            center=centre, axis=a.copy(), width=width, antipodal=anti, gravity=grav,
            fit=fit, score=score, both_sides_observed=both,
            contacts=(Q[i_lo].copy(), Q[i_hi].copy()),
        ))

    cands.sort(key=lambda d: -d['score'])
    cos_nms = np.cos(np.radians(cfg['nms_angle_deg']))
    out = []
    for cd in cands:
        if any(np.linalg.norm(cd['center'] - o['center']) <= cfg['nms_dist']
               and abs(float(cd['axis'] @ o['axis'])) >= cos_nms for o in out):
            continue
        out.append(cd)
        if len(out) >= cfg['max_candidates']:
            break
    return out


def sample_grasps_from_cloud(pts, obj_xyz, view_xyz, cfg, hand_filter=None):
    """Full pipeline on a raw world-frame cloud. Pure numpy -- no ROS.

    `hand_filter(pts) -> keep_mask` removes the robot's own hand; it is injected
    rather than done here because it needs live TF. Passing None skips it, which
    is what a synthetic-cloud check wants (no robot in the scene at all).

    Returns a dict with 'grasps' plus the diagnostics the service reports.
    """
    res = dict(grasps=[], n_gripper_filtered=0, n_support_filtered=0,
               n_cluster_points=0, principal_axis=np.array([0.0, 0.0, 1.0]),
               extent=np.zeros(3), centroid=np.asarray(obj_xyz, float),
               message='')
    pts = np.asarray(pts, float)
    if len(pts) == 0:
        res['message'] = 'empty cloud'
        return res

    if hand_filter is not None:
        keep = hand_filter(pts)
        res['n_gripper_filtered'] = int((~keep).sum())
        pts = pts[keep]
        if len(pts) == 0:
            res['message'] = 'every point was the robot\'s own hand'
            return res

    roi = pts[np.linalg.norm(pts - np.asarray(obj_xyz, float), axis=1) <= cfg['roi']]
    if len(roi) < cfg['min_cluster_points']:
        res['message'] = (f'only {len(roi)} points within roi={cfg["roi"]:.3f} m '
                          f'of the reported object position')
        return res

    roi, n_plane = remove_support_plane(roi, cfg)
    res['n_support_filtered'] = n_plane
    if len(roi) < cfg['min_cluster_points']:
        res['message'] = (f'only {len(roi)} points left after removing the '
                          f'support surface ({n_plane} dropped)')
        return res

    clusters = cluster_voxel(roi, cfg['cluster_res'], cfg['min_cluster_points'])
    idx = pick_cluster(roi, clusters, obj_xyz)
    if idx is None:
        res['message'] = (f'no cluster of >= {cfg["min_cluster_points"]} points '
                          f'in the roi ({len(roi)} points, '
                          f'res={cfg["cluster_res"]:.3f} m)')
        return res
    obj = roi[idx]
    res['n_cluster_points'] = len(obj)

    # Q keeps the cluster's true surface extent (widths are measured here); P is
    # thinned only as far as the quadratic neighbour search requires.
    Q = voxel_downsample(obj, cfg['voxel_fine'])
    P = voxel_downsample(obj, cfg['voxel'])
    if len(P) > cfg['max_points']:
        sel = np.random.default_rng(0).choice(len(P), cfg['max_points'],
                                              replace=False)
        P = P[sel]

    centroid = P.mean(axis=0)
    cov = np.cov((P - centroid).T)
    vals, vecs = np.linalg.eigh(cov)
    res['centroid'] = centroid
    res['principal_axis'] = vecs[:, -1]
    proj = (P - centroid) @ vecs
    res['extent'] = proj.max(axis=0) - proj.min(axis=0)

    normals, knn_dist = estimate_normals_knn(P, cfg['knn'], view_xyz)
    grasps = antipodal_candidates(P, normals, knn_dist, Q, res['principal_axis'],
                                  cfg)
    res['grasps'] = grasps
    if not grasps:
        res['message'] = (f'no candidate survived on {len(P)} cluster points '
                          f'(width {cfg["min_width"]:.3f}-'
                          f'{cfg["max_width"]:.3f} m over '
                          f'{cfg["n_axes"]} closing axes)')
    else:
        res['message'] = f'{len(grasps)} grasp(s) from {len(P)} cluster points'
    return res


# --------------------------------------------------------------------------
# ROS wrapper
# --------------------------------------------------------------------------

class GraspSampler(Node):
    def __init__(self):
        super().__init__('grasp_sampler')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('wrist_cloud_topic', '/wrist1/depth_cloud')
        self.declare_parameter('wrist_optical_frame', 'wrist1_camera_optical')
        self.declare_parameter('gripper_link', 't1_a1_gripper_base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('objects_topic', '/detected_objects')
        self.declare_parameter('target_topic', '/target_object')
        self.declare_parameter('capture_timeout', 5.0)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.cloud_topic = g('wrist_cloud_topic')
        self.optical_frame = g('wrist_optical_frame')
        self.gripper_link = g('gripper_link')
        self.world_frame = g('world_frame')
        self.capture_timeout = float(g('capture_timeout'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._lock = threading.Lock()
        self._buf = []
        self._capturing = False
        self._objects = None
        self._target = None
        self._tracks_xyz = {}
        self._tracks_lbl = {}

        # Multi-view accumulator for tier-2/3 oblique escalation (request
        # field `accumulate`). Points and the camera position they were seen
        # from are kept TOGETHER: normals must be oriented per-point toward
        # the nearest view, so a fused cloud cannot be reduced to one blob
        # plus one viewpoint without destroying the far-side evidence the
        # second look exists to collect.
        self._acc_pts = []          # list of (N,3) arrays, one per batch
        self._acc_views = []        # list of (3,) camera positions, index-aligned
        self._acc_target = None     # world xyz the accumulation belongs to

        # The cloud sub and the service run in SEPARATE callback groups under a
        # multi-threaded executor: the service handler blocks while it waits for
        # fresh frames, and on a single-threaded executor that wait would also
        # block the subscription delivering them -- a guaranteed deadlock, not a
        # slow path.
        sub_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud,
                                 qos_profile_sensor_data, callback_group=sub_cb)
        self.create_subscription(PoseArray, g('objects_topic'),
                                 self._on_objects, 10, callback_group=sub_cb)
        self.create_subscription(PoseStamped, g('target_topic'),
                                 self._on_target, 10, callback_group=sub_cb)
        # Stable-identity handles from object_localizer. Latched, to match the
        # publisher. See _resolve_target for why an index is not good enough.
        tq = QoSProfile(depth=1)
        tq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, g('objects_topic') + '/tracks',
                                 self._on_tracks, tq, callback_group=sub_cb)
        self.srv = self.create_service(SampleGrasps, '~/sample_grasps',
                                       self._on_request, callback_group=srv_cb)
        self.debug_pub = self.create_publisher(PoseArray, '~/grasp_candidates', 1)

        self.get_logger().info(
            f'grasp_sampler up; cloud={self.cloud_topic} '
            f'gripper={self.gripper_link} -- call ~/sample_grasps')

    # ---- inputs ----------------------------------------------------------
    def _on_cloud(self, msg):
        with self._lock:
            if self._capturing:
                self._buf.append(msg)

    def _on_objects(self, msg):
        self._objects = msg

    def _on_target(self, msg):
        self._target = msg

    def _on_tracks(self, msg):
        try:
            arr = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._tracks_xyz = {int(t['tid']): np.array(
            [float(t['x']), float(t['y']), float(t['z'])]) for t in arr}
        self._tracks_lbl = {int(t['tid']): str(t.get('label', '')) for t in arr}

    def _cfg(self, req):
        cfg = {k: self.get_parameter(k).value for k in DEFAULTS}
        if req.roi > 0:
            cfg['roi'] = float(req.roi)
        if req.captures > 0:
            cfg['captures'] = int(req.captures)
        if req.max_candidates > 0:
            cfg['max_candidates'] = int(req.max_candidates)
        return cfg

    def _accumulate(self, pts, view_xyz, obj_xyz, keep):
        """Fold this call's captures into the multi-view accumulator and return
        the (points, views) to sample from.

        `keep` False = one-shot behaviour: this call samples from its own
        captures only, so a normal nadir call is never contaminated by an older
        look -- but the batch is still REMEMBERED as the start of a fresh
        accumulation, so a follow-up `keep` True call (the oblique escalation)
        has that first view to fuse against.

        The accumulator is also dropped when the target has MOVED more than
        acc_reset_dist -- either a different object, or the same one after it
        was nudged. Fusing a stale far-side cloud onto a moved object would
        invent contact evidence that is no longer physically there, which is
        exactly the silent-wrong-geometry failure this whole node is careful
        about."""
        acc_reset_dist = 0.05
        pts = np.asarray(pts, float)
        view_xyz = np.asarray(view_xyz, float)
        with self._lock:
            if not keep:
                # Start a NEW accumulation from this batch: this call still
                # samples from its own points only, but they are retained so a
                # follow-up accumulate call (the oblique escalation) has the
                # first view to fuse against. Dropping them here left every
                # escalation running on a single view -- observed live as
                # "1 view(s) [accumulated]" on tier 1, with
                # both_sides_observed stuck False no matter how many tiers ran.
                self._acc_pts = [pts]
                self._acc_views = [view_xyz]
                self._acc_target = np.asarray(obj_xyz, float)
                return pts, np.atleast_2d(view_xyz)
            moved = (self._acc_target is not None
                     and float(np.linalg.norm(np.asarray(obj_xyz, float)
                                              - self._acc_target)) > acc_reset_dist)
            if moved:
                self.get_logger().info(
                    'sample_grasps: target moved '
                    f'{np.linalg.norm(np.asarray(obj_xyz, float) - self._acc_target):.3f} m '
                    f'(> {acc_reset_dist:.2f} m) -- dropping the accumulated '
                    'views rather than fusing onto a different pose')
                self._acc_pts, self._acc_views = [], []
            self._acc_pts.append(np.asarray(pts, float))
            self._acc_views.append(np.asarray(view_xyz, float))
            self._acc_target = np.asarray(obj_xyz, float)
            return np.vstack(self._acc_pts), np.array(self._acc_views)

    def _resolve_target(self, target):
        """Resolve a target handle to a world position.

        `#<tid>` -- a persistent object_localizer track id -- is the PREFERRED
        form and the only one that is safe across calls. A bare index is a
        position in the current /detected_objects PoseArray, and that array is
        rebuilt per publish: its LENGTH and ORDER both change as detections come
        and go, so index 5 can be one object when the caller looks and a
        different one moments later when the grasp is sampled. Measured live on
        this scene: index 5 was the 0.06 m cube at (0.19, 0.40) in one frame and
        an object at (2.83, -0.62) in another, purely from the list resizing
        between 6 and 7 entries. A track id names the same physical object
        regardless.

        Index is still accepted -- it is what pick_cli lists and what
        gantry_reach_executor's ~/pick and ~/look take -- but it warns, so a
        mismatched result is traceable instead of mysterious. /target_object is
        the last fallback: it refreshes on a timer and lags a fresh selection.
        """
        t = (target or '').strip()
        if t.startswith('#'):
            try:
                tid = int(t[1:])
            except ValueError:
                return None, f'malformed track handle "{t}" (expected #<tid>)'
            if tid not in self._tracks_xyz:
                known = sorted(self._tracks_xyz)
                return None, (f'track #{tid} not among {len(known)} known '
                              f'tracks {known}')
            lbl = self._tracks_lbl.get(tid, '')
            return self._tracks_xyz[tid].copy(), f'track #{tid} ({lbl})'

        objs = self._objects
        if t.isdigit() and objs is not None and objs.poses:
            i = int(t)
            if not (0 <= i < len(objs.poses)):
                return None, f'object index {i} out of range (have {len(objs.poses)})'
            p = objs.poses[i].position
            self.get_logger().warn(
                f'target "{i}" is a positional index into a list that is '
                f'rebuilt every publish (currently {len(objs.poses)} entries) '
                f'-- it can name a different object than the caller meant. '
                f'Prefer a #<tid> track handle.')
            return np.array([p.x, p.y, p.z]), f'index {i}'
        if self._target is not None:
            p = self._target.pose.position
            return np.array([p.x, p.y, p.z]), '/target_object'
        return None, (f'target "{t}": not a #<tid> track handle, no matching '
                      f'/detected_objects index, and no /target_object yet')

    def _capture(self, n):
        with self._lock:
            self._buf = []
            self._capturing = True
        deadline = time.time() + self.capture_timeout
        while time.time() < deadline:
            with self._lock:
                got = len(self._buf)
            if got >= n:
                break
            time.sleep(0.02)
        with self._lock:
            self._capturing = False
            msgs = list(self._buf)
        return msgs

    def _hand_filter(self, cfg):
        """Mask out the robot's own hand using live TF -- see module docstring
        for why the cloud is full of it. Returns None if the gripper TF is
        unavailable, so the caller can fail loudly rather than sample grasps on
        the robot's own fingers."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.gripper_link, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        def keep(pts):
            loc = (pts - T) @ R                    # world -> gripper local
            radial = np.hypot(loc[:, 0], loc[:, 1])
            inside = (radial <= cfg['gripper_radius']) & (loc[:, 2] <= cfg['gripper_ahead'])
            return ~inside
        return keep

    def _camera_xyz(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.optical_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z])

    # ---- service ---------------------------------------------------------
    def _on_request(self, req, resp):
        cfg = self._cfg(req)
        resp.grasps.header.frame_id = self.world_frame
        resp.grasps.header.stamp = self.get_clock().now().to_msg()

        obj_xyz, why = self._resolve_target(req.target)
        if obj_xyz is None:
            resp.success, resp.message = False, why
            self.get_logger().warn(f'sample_grasps: {why}')
            return resp

        view_xyz = self._camera_xyz()
        if view_xyz is None:
            resp.success = False
            resp.message = (f'no TF {self.world_frame} <- {self.optical_frame}; '
                            'without the camera position, surface normals cannot '
                            'be oriented and antipodality is meaningless')
            self.get_logger().error(f'sample_grasps: {resp.message}')
            return resp

        hand = self._hand_filter(cfg)
        if hand is None:
            resp.success = False
            resp.message = (f'no TF {self.world_frame} <- {self.gripper_link}; '
                            'refusing to sample, the wrist cloud contains the '
                            'hand itself and would be grasped as if it were the '
                            'object')
            self.get_logger().error(f'sample_grasps: {resp.message}')
            return resp

        msgs = self._capture(cfg['captures'])
        if not msgs:
            resp.success = False
            resp.message = (f'no wrist cloud on {self.cloud_topic} within '
                            f'{self.capture_timeout:.1f}s')
            self.get_logger().error(f'sample_grasps: {resp.message}')
            return resp
        if len(msgs) < cfg['captures']:
            self.get_logger().warn(
                f'sample_grasps: only {len(msgs)}/{cfg["captures"]} captures '
                f'arrived in {self.capture_timeout:.1f}s -- using what came in')
        pts = np.vstack([_cloud_xyz(m) for m in msgs])

        # Hand-filter BEFORE accumulating: the gripper sits somewhere different
        # in every look pose, so a batch stored raw would carry the hand from
        # the pose it was taken at, and a later call's TF cannot mask a hand
        # that is no longer there.
        keep_mask = hand(pts)
        n_hand_here = int((~keep_mask).sum())
        pts = pts[keep_mask]

        pts, views = self._accumulate(pts, view_xyz, obj_xyz,
                                      bool(req.accumulate))

        self.get_logger().info(
            f'sample_grasps: {why} at {np.round(obj_xyz, 3)}, '
            f'{len(pts)} points from {len(msgs)} capture(s) in '
            f'{len(views)} view(s)'
            + (' [accumulated]' if req.accumulate else ''))
        # hand_filter=None: already applied per-batch above.
        out = sample_grasps_from_cloud(pts, obj_xyz, views, cfg, None)
        out['n_gripper_filtered'] = n_hand_here

        resp.n_cluster_points = int(out['n_cluster_points'])
        resp.n_gripper_filtered = int(out['n_gripper_filtered'])
        resp.n_support_filtered = int(out['n_support_filtered'])
        resp.principal_axis = Vector3(**dict(zip('xyz', map(float, out['principal_axis']))))
        resp.cluster_extent = Vector3(**dict(zip('xyz', map(float, out['extent']))))
        resp.cluster_centroid = Point(**dict(zip('xyz', map(float, out['centroid']))))
        resp.message = out['message']

        for gr in out['grasps']:
            p, R = grasp_frame(gr['center'], gr['axis'], cfg['finger_offset'])
            qx, qy, qz, qw = R_to_quat(R)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, p)
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = map(float, (qx, qy, qz, qw))
            resp.grasps.poses.append(pose)
            resp.scores.append(gr['score'])
            resp.widths.append(gr['width'])
            resp.antipodal.append(gr['antipodal'])
            resp.gravity.append(gr['gravity'])
            resp.both_sides_observed.append(gr['both_sides_observed'])

        resp.success = bool(out['grasps'])
        if resp.success:
            top = out['grasps'][0]
            self.get_logger().info(
                f'sample_grasps: {len(out["grasps"])} candidate(s); top '
                f'width={top["width"]:.4f} m antipodal={top["antipodal"]:.3f} '
                f'gravity={top["gravity"]:.3f} '
                f'both_sides={top["both_sides_observed"]} '
                f'center={np.round(top["center"], 4)} '
                f'axis={np.round(top["axis"], 3)} | cluster={resp.n_cluster_points} '
                f'pts extent={np.round(out["extent"], 4)} '
                f'(hand {resp.n_gripper_filtered}, plane {resp.n_support_filtered} '
                f'dropped)')
            self.debug_pub.publish(resp.grasps)
        else:
            self.get_logger().warn(f'sample_grasps: {out["message"]}')
        return resp


def main():
    if not _HAVE_ROS:
        raise ImportError('grasp_sampler needs a sourced ROS 2 workspace '
                          '(and reachability_gng_interfaces built) to run as a '
                          'node; the geometry core alone imports without it')
    rclpy.init()
    node = GraspSampler()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
