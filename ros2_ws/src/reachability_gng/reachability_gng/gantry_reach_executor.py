"""Energy-aware redundancy resolution + arm selection for the gantry workcell.

Given a detected object, this node decides WHICH arm (any configured subset of
arm_1..arm_4, each pair riding a shared gantry) reaches it and at WHICH 8-DOF
configuration -- by
ENERGY, not by nearest seed -- then hands MoveIt a joint-space goal so MoveIt
does the actual collision-aware planning (and optionally execution) against the
live octomap.

Pipeline per pick (`~/pick`):
  1. Take the target object pose from /detected_objects.
  2. From EACH arm's GNG map, gather the POOL of nodes within `pool_radius`
     (task-space) of the object -- the reachability filter (hard).
  3. Score every pooled candidate by
        J = w_gantry_lin*d_gantry_lin/ref_gantry_lin
            + w_gantry_rot*d_gantry_rot/ref_gantry_rot + w_arm*d_arm/ref_arm
            + w_dist*ee_dist/ref_dist + w_hold*hold/ref_hold
            - w_manip*manip/ref_manip
     where d_* are joint travel from the current state, each divided by a fixed
     physical reference (ref_*) so every term is dimensionless ~O(1) and the
     w_* are pure, directly-comparable priorities. The gantry's linear
     (prismatic, metres) and rotation (radians) axes are weighted separately
     because their units and cost differ; gantry travel is the dominant,
     expensive term. `ee_dist` is the arm's CURRENT tool-frame distance (TF,
     metres) to the object -- one value per arm, so this term biases the
     allocation toward whichever arm's end-effector is already nearer. (The
     per-node task-space `dist` still gates the pool and is logged for the
     rank-by-distance diagnostic.) `hold` is the ||generalized gravity torque||
     at the node's config -- the static load of holding that pose, so J prefers
     configs that fight gravity less. `manip` is the node manipulability. J is
     the OBJECTIVE; arm selection emerges from J.
  4. Evaluate candidates in ascending-J order: IK to the EXACT object pose
     (seeded by the candidate q) on that arm's `gantry_1_with_arm_<n>` group ->
     MoveGroup plan (plan_only unless `execute`). Accept the FIRST candidate
     with a collision-free plan; otherwise fall through to the next (robustness).
  5. Log per-pick CSV (chosen arm, J + components, rank-by-J vs rank-by-distance,
     resulting gantry placement, IK/plan time, optional trajectory energy).

Prerequisites (assumed already up, like the other launches): move_group with
/compute_ik + the move_action server (my_workcell.launch.py) and the perception
stack publishing /detected_objects (perception.launch.py).

    ros2 launch reachability_gng gantry_pick.launch.py
    ros2 topic pub --once /gantry_reach_executor/pick std_msgs/String "{data: '0'}"
"""
from __future__ import annotations

import csv
import json
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseArray, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.gng import GNG
from reachability_gng.object_localizer import quat_to_R
from reachability_gng.pause_gate import PAUSE_TOPIC, latched_qos
from reachability_gng.seed_ik import build_ik_request
from reachability_gng_interfaces.srv import SampleGrasps


def _cloud_xyz(msg):
    """Finite (x, y, z) rows of an xyz-float32 PointCloud2 as (N,3) float64.
    Same 12-byte-stride layout depth_cloud.py publishes -- identical helper to
    env_gng._cloud_xyz, duplicated rather than imported so grasp geometry does
    not depend on the (unrelated) topo-map module."""
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    a = a.reshape(-1, msg.point_step)[:, :12].copy().view(np.float32)
    return a[np.isfinite(a).all(axis=1)].astype(np.float64)


def quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions -> (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def R_to_quat(R):
    """Rotation matrix (3x3) -> (x, y, z, w) quaternion (Shepperd's method: pick
    the branch with the largest divisor so no near-180-deg rotation divides by
    ~0). Inverse of object_localizer.quat_to_R."""
    R = np.asarray(R, float)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        q = (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, \
            (R[1, 0] - R[0, 1]) / s, 0.25 * s
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, \
            (R[2, 1] - R[1, 2]) / s
    elif R[1, 1] >= R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, \
            (R[0, 2] - R[2, 0]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, \
            (R[1, 0] - R[0, 1]) / s
    q = np.array(q, float)
    return tuple(q / np.linalg.norm(q))


def look_camera_pose(obj_xyz, distance, roll, tilt=0.0, azimuth=0.0):
    """Desired WORLD pose of the wrist camera's OPTICAL frame staring at
    `obj_xyz` from `distance` metres away. Returns (p_cam (3,), R_cam (3,3)),
    R_cam holding the optical axes as COLUMNS.

    The view direction here is the unit vector object -> camera: straight up
    (world +Z, i.e. the camera looks straight DOWN) at tilt=0 -- the nadir tier,
    the only one wired up so far -- swung `tilt` rad off vertical toward compass
    direction `azimuth` for the later oblique tiers.

    Optical-frame convention, matching what Isaac publishes on
    <ns>_camera_optical and what object_localizer.deproject assumes: +Z forward
    along the view ray, +X right, +Y down. Rotation ABOUT that ray leaves the
    view unchanged (same pixels, rotated image), so it is a free DOF -- `roll`
    parametrizes it, and it is the only handle for making an otherwise-identical
    view reachable by IK.

    (The wrist camera's static TF in launch_workcell.sh is a hand-computed
    number, deliberately kept REP-103-compliant -- see that file's comment on
    the wrist1_camera_optical static_transform_publisher for why this matters:
    an earlier version of this function compensated here for a TF that had
    been published backwards, which was the wrong place to fix it -- it made
    IK targeting correct but left depth_cloud's point-cloud deprojection
    silently wrong, since that node trusts the same TF and has no equivalent
    compensation. Fixed at the TF source instead; this function needs none.)
    """
    n = np.array([np.sin(tilt) * np.cos(azimuth),
                  np.sin(tilt) * np.sin(azimuth),
                  np.cos(tilt)], float)
    p_cam = np.asarray(obj_xyz, float) + float(distance) * n
    z_c = -n                                   # optical +Z looks AT the object
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, z_c))) > 0.95:    # ref ~parallel to the ray
        ref = np.array([0.0, 1.0, 0.0])
    x0 = ref - float(np.dot(ref, z_c)) * z_c
    x0 /= np.linalg.norm(x0)
    y0 = np.cross(z_c, x0)
    x_c = np.cos(roll) * x0 + np.sin(roll) * y0
    y_c = np.cross(z_c, x_c)
    return p_cam, np.column_stack((x_c, y_c, z_c))


class ArmModel:
    """A per-arm GNG map + stats + its MoveIt group / ee-frame / joint order."""

    def __init__(self, name, model_path, group, ee_frame, gripper_link):
        self.name = name
        self.group = group
        self.ee_frame = ee_frame
        self.gripper_link = gripper_link
        self.gng = GNG.load(model_path)
        self.W = self.gng.W
        self.task_dim = self.gng.task_dim
        base = model_path[:-4] if model_path.endswith('.npz') else model_path
        stats = np.load(base + '_stats.npz')
        n = len(self.W)
        self.manip = stats['manip'] if 'manip' in stats else np.zeros(n)
        self.hold = stats['hold'] if 'hold' in stats else np.zeros(n)
        names = stats['joint_names'] if 'joint_names' in stats else []
        self.joint_names = [str(x) for x in names] if len(names) else None
        self.spacing = self._node_spacing()

    def _node_spacing(self, chunk=1024):
        """Median nearest-neighbour distance between node xyz (m). Lets the pool
        radius scale with map resolution instead of a brittle fixed metric."""
        W = np.asarray(self.W[:, :3], float)
        n = len(W)
        if n < 2:
            return 0.0
        nn = np.empty(n)
        for s in range(0, n, chunk):
            blk = W[s:s + chunk]
            d2 = np.einsum('nmk,nmk->nm',
                           blk[:, None, :] - W[None, :, :],
                           blk[:, None, :] - W[None, :, :])
            rows = np.arange(blk.shape[0])
            d2[rows, s + rows] = np.inf
            nn[s:s + blk.shape[0]] = np.sqrt(d2.min(axis=1))
        return float(np.median(nn))

    def candidates(self, task_vec, radius, max_k):
        """[(node_idx, q, task_dist, manip, hold)] within radius, nearest-first."""
        idx = self.gng.query_radius(task_vec, radius, max_k)
        x = np.asarray(task_vec, float)
        out = []
        for i in idx:
            i = int(i)
            q = self.W[i, self.task_dim:].copy()
            dist = float(np.linalg.norm(self.W[i, :len(x)] - x))
            out.append((i, q, dist,
                        float(np.nan_to_num(self.manip[i])),
                        float(np.nan_to_num(self.hold[i]))))
        return out


class GantryReachExecutor(Node):
    def __init__(self):
        super().__init__('gantry_reach_executor')
        # --- arms ---
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter('arm_models',
                               ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz'])
        self.declare_parameter('arm_groups',
                               ['gantry_1_with_arm_1', 'gantry_1_with_arm_2'])
        self.declare_parameter('arm_ee_frames',
                               ['t1_a1_tool_frame', 't1_a2_tool_frame'])
        self.declare_parameter('gripper_links',
                               ['t1_a1_gripper_base_link',
                                't1_a2_gripper_base_link'])
        self.declare_parameter('arm_pin_configs', [''])  # for trajectory energy
        # --- task / pool ---
        self.declare_parameter('task', 'pos')
        self.declare_parameter('ori_weight', 0.3)
        # Centroid detection gives NO real orientation (identity quat), which is
        # unreachable for the ceiling arm -> IK returns -31. For task='pos' we
        # override the IK target orientation with this fixed grasp approach
        # (xyzw). Default (1,0,0,0) = tool flipped to point DOWN at the table.
        self.declare_parameter('grasp_orientation', [1.0, 0.0, 0.0, 0.0])
        # Yaw is free for a top-down grasp: sample this many rotations about world
        # Z around grasp_orientation and accept the first IK that solves. KDL solves
        # the exact 6-DOF pose, so a single unreachable orientation -> -31; sampling
        # yaw gives the redundant arm+gantry several reachable targets. 1 = off.
        self.declare_parameter('grasp_yaw_samples', 8)
        # Tilt fallback: the GNG reach map is POSITION-only, so it can flag an xyz
        # reachable while a straight-DOWN grasp there is infeasible (the arm can
        # only touch that spot at an angle) -> IK -31 for every yaw. Sample this
        # many tilt magnitudes (up to grasp_tilt_max deg off vertical), each swung
        # around grasp_tilt_azimuths compass directions, and accept the first IK
        # that solves. Vertical is ALWAYS tried first (preferred); tilts are pure
        # fallback so a clean top-down grasp still wins when available. 0 = off
        # (vertical + yaw only, legacy).
        self.declare_parameter('grasp_tilt_samples', 2)
        self.declare_parameter('grasp_tilt_max', 45.0)     # deg off vertical
        self.declare_parameter('grasp_tilt_azimuths', 4)
        # Pre-grasp: aim this many metres ABOVE the object centroid (world +Z) so
        # the gripper stops over the object instead of plunging to the centroid and
        # hitting the table/environment octomap (goal-pose collision -> -2).
        self.declare_parameter('approach_offset', 0.10)
        # EE stand-off: when object_collision has published a fitted 3D box for
        # the target (/target_collision_boxes), aim the pre-grasp this many metres
        # ABOVE the box top instead of approach_offset above the centroid -- a
        # tall object's centroid sits inside its body, so a centroid+offset goal
        # can still be within the object (goal-in-collision -2). Standing off the
        # box top keeps a safe EE<->box clearance regardless of object height.
        self.declare_parameter('box_clearance', 0.05)
        self.declare_parameter('world_frame', 'world')
        # pool_radius <= 0 -> density-adaptive (pool_radius_factor * node
        # spacing), so the pool size is independent of the GNG `lam`; > 0 ->
        # absolute task-space metres (mirrors reachability_check's convention).
        self.declare_parameter('pool_radius', 0.0)
        self.declare_parameter('pool_radius_factor', 2.5)
        self.declare_parameter('max_candidates', 20)
        self.declare_parameter('n_gantry_dofs', 2)
        # --- energy weights (J) --- single source of truth for the calibrated
        # defaults; the launch no longer overrides these.
        #
        # Each term is NORMALISED by a fixed physical reference (ref_*, its
        # representative full-scale) BEFORE its weight is applied, so every term
        # feeds J as a dimensionless ~O(1) quantity:
        #     J = sum_i  w_i * (value_i / ref_i)   (manip enters with a minus).
        # That decouples the two jobs a single weight would otherwise have to do
        # at once: ref_* absorbs the unit/scale conversion (metres vs radians vs
        # Nm vs the tiny ~0.1 manip index), leaving w_* as a PURE priority knob
        # you can read and compare directly. Retune priority via w_*, rescale a
        # term via ref_*.
        #
        # w_*/ref_* below are EMPIRICALLY calibrated (not hand-picked) from a
        # 63-pick /tmp/calib.csv session: ref_* = each term's median raw value
        # (analyze_calib.py Tahap 1); w_* = Spearman(term, traj_energy) evidence
        # (Tahap 4) -- J is meant as an ENERGY proxy here, so priority follows
        # what actually tracks the executed trajectory's mechanical energy
        # (Pinocchio rnea over the full 8-DOF gantry+arm chain), not intuition:
        #   arm   rho=0.308 (strongest, most stable)      -> w_arm=20 (leads)
        #   grot  rho=0.276 (real, stable)                -> w_gantry_rot=12
        #   dist  rho=0.18-0.24 (weak, UNSTABLE across n)  -> w_dist=3
        #   glin  rho=0.15 (weak; adding it LOWERS combined rho) -> w_gantry_lin=2
        #   manip rho=0.12 (weak, wrong-ish sign)          -> w_manip=1
        #   hold  rho=0.03 (~no signal)                    -> w_hold=1
        # CAVEAT: even the best achievable combo (arm alone) only reaches
        # rho~0.31 (weak). Root cause (verified): the URDF has NO <dynamics
        # damping/friction> on any gantry or arm joint, and t1_linear_joint's
        # axis (1,0,0) / t1_rotation_joint's axis (0,0,1) are both orthogonal to
        # gravity -- so this idealised rigid-body model can't capture the
        # friction/stiction that likely dominates the REAL gantry motors' energy
        # draw. Treat J as idealised mechanical energy (gravity+inertia only),
        # not real motor energy, when writing this up.
        # Override live with -p w_*:=... or -p ref_*:=... (re-run
        # analyze_calib.py after collecting new picks to re-derive these).
        #
        # The gantry's two DOFs are weighted SEPARATELY because their units and
        # cost differ: w_gantry_lin scores the linear/prismatic axis (metres of
        # heavy-carriage travel), w_gantry_rot the rotation axis (radians).
        self.declare_parameter('w_gantry_lin', 2.0)
        self.declare_parameter('w_gantry_rot', 12.0)
        self.declare_parameter('w_arm', 20.0)
        self.declare_parameter('w_manip', 1.0)
        # w_hold scores the static gravity load (||generalized gravity torque||,
        # Nm) of holding the candidate config -- a positive cost so J prefers
        # poses that fight gravity less (0 = ignore gravity). Requires maps built
        # with --config so `hold` is non-zero; hold=0 makes this term inert.
        self.declare_parameter('w_hold', 1.0)
        # w_dist scores the task-space gap (metres) between the candidate node
        # and the object: distance already gates the pool, this also makes a
        # closer seed cheaper inside J (0 = distance only gates, does not rank).
        self.declare_parameter('w_dist', 3.0)
        # Fixed normalisation references (representative full-scale of each term):
        # linear travel (m), gantry rotation (rad), summed arm-joint travel (rad),
        # ee->object gap (m), manipulability index. Constants (not pool-relative)
        # so J stays comparable across picks. Values = median raw term across the
        # 63-pick calibration session (analyze_calib.py Tahap 1).
        self.declare_parameter('ref_gantry_lin', 0.95)
        self.declare_parameter('ref_gantry_rot', 0.70)
        self.declare_parameter('ref_arm', 6.0)
        self.declare_parameter('ref_dist', 1.36)
        self.declare_parameter('ref_manip', 0.145)
        # representative full-scale gravity hold torque (Nm): median ||gravity
        # torque|| across the 63-pick calibration session (was 6.5, a rougher
        # estimate from the full GNG map rather than the actual in-pool
        # candidates that J evaluates).
        self.declare_parameter('ref_hold', 2.90)
        # Print the full ranked J table (every pooled candidate, ascending J,
        # with each term's weighted contribution) to the terminal on each pick.
        self.declare_parameter('log_j_table', True)
        self.declare_parameter('log_j_table_max', 20)   # rows to print (top-N)
        # --- IK / planning ---
        self.declare_parameter('ik_timeout', 0.05)        # inside the IK request
        self.declare_parameter('ik_wait', 2.0)            # service round-trip cap
        # Collision-aware IK. True (default) makes /compute_ik reject a solution
        # that collides -> returns -31. Set False to A/B test a -31: if IK then
        # succeeds, the -31 was a COLLISION (e.g. against the static collision
        # geometry / octomap), not a kinematically unreachable pose.
        self.declare_parameter('ik_avoid_collisions', True)
        self.declare_parameter('plan_time', 2.0)
        self.declare_parameter('plan_attempts', 5)
        self.declare_parameter('plan_wait', 12.0)
        # When execute=True the MoveGroup result future only completes AFTER the
        # whole trajectory has physically moved -- a slow gantry travel easily
        # exceeds the planning budget. Wait this long for the execution result so
        # a still-running (successful) motion is not mis-reported as a failure
        # (and so the loop does not fire the next candidate mid-execution, which
        # left a ghost/duplicate arm in RViz). A genuine planning failure still
        # returns fast with an error code, so this does not slow the fallback.
        self.declare_parameter('exec_wait', 120.0)
        # move_group's execute result can return before Isaac Sim's slower-than-
        # realtime physics finish settling, so SUCCESS is confirmed only once the
        # LIVE /joint_states converge to the goal within reach_tol (max per-joint
        # error, rad for arm / m for the gantry linear axis), waiting up to
        # exec_wait for that. This makes the SUCCESS message reflect the real arm.
        self.declare_parameter('reach_tol', 0.03)
        self.declare_parameter('vel_scale', 0.2)
        self.declare_parameter('acc_scale', 0.2)
        self.declare_parameter('joint_tolerance', 1e-3)
        self.declare_parameter('execute', False)          # plan-only by default
        self.declare_parameter('max_attempts', 8)
        # Head start for the allocation winner (overall lowest-J arm): try this
        # many of its nodes BEFORE any other arm gets a turn. 1 = strict
        # round-robin (old behaviour). >1 stops a single un-plannable winner node
        # (e.g. a goal-in-collision -2) from immediately handing the whole task to
        # another arm -- e.g. when the winner arm is already hovering over the
        # object, let it try its next node in place before a far arm drives the
        # shared gantry across the workcell. Capped below max_attempts so the
        # other arm is still guaranteed a shot.
        self.declare_parameter('winner_head_start', 2)
        # In-place re-grasp: when an arm's tool is already essentially over the
        # object (current ee_dist <= this, metres), add a candidate whose IK SEED
        # is the arm's CURRENT config -- so IK re-grasps from where the arm
        # already is (near-zero gantry travel) instead of only re-seeding from GNG
        # nodes whose stored configs can jump the shared gantry across the
        # workcell. 0 disables. Default 0.30 m cleanly separates "hovering after a
        # pick" (~0.17 m) from "moved away / far arm" (>0.8 m).
        self.declare_parameter('in_place_radius', 0.30)
        # Fix A: freeze the sensed scene (octomap + object CollisionObjects) for
        # the plan/execute window so a mid-plan scene-version bump doesn't drop the
        # plan (-3/-2). gate_settle lets the last in-flight update land first.
        self.declare_parameter('gate_perception', True)
        self.declare_parameter('gate_settle', 0.5)
        # --- wrist-camera look poses (grasp geometry comes from the WRIST
        # camera only; the ceiling RGBDs keep their localization/collision role
        # and are never used for grasp geometry) ---
        # Index-aligned with arm_names, same convention as arm_groups /
        # arm_ee_frames / gripper_links above; '' (or a short list) = that arm
        # has no wrist camera, so no look pose can be computed for it.
        self.declare_parameter('wrist_frames', ['wrist1_camera_optical'])
        # Index-aligned wrist depth_cloud topic per arm (empty = no wrist camera
        # / not captured for that arm). A SEPARATE depth_cloud instance from the
        # one env_gng/topo map uses (see gantry_pick.launch.py wrist_cloud:=true)
        # -- different min_depth/stride tuned for the D405's close-range sweet
        # spot, not the ceiling cameras'.
        self.declare_parameter('wrist_cloud_topics', ['/wrist1/depth_cloud'])
        # Camera-to-object distance at the look pose (m). 0.25 sits mid-range of
        # the planned RealSense D405's 0.07-0.5 m sweet spot.
        self.declare_parameter('look_distance', 0.25)
        # Roll about the view ray is free (see look_camera_pose): sample this
        # many, all giving the SAME view, so IK has several tool poses to try.
        self.declare_parameter('look_roll_samples', 8)
        # Stop-and-stare acquisition (session A2): after the arm reaches the
        # look pose, wait this long (settle, like grasp_settle_s) before
        # accepting any wrist cloud, then collect this many FRESH captures
        # (buffer cleared at settle-end, so no frame from mid-motion can sneak
        # in) within look_timeout seconds, cropped to look_roi metres around the
        # object for the drift/point-count report.
        self.declare_parameter('look_captures', 5)
        self.declare_parameter('look_settle_s', 0.5)
        self.declare_parameter('look_timeout', 5.0)
        self.declare_parameter('look_roi', 0.15)
        # Progress-based settle (replaces relying on look_settle_s alone):
        # after the minimum look_settle_s wait, keep probing wrist frames until
        # the ROI centroid stops moving between consecutive frames -- Isaac's
        # rendered cloud can lag physics by more than a fixed sleep covers (a
        # capture straight after a real move can come back with ZERO ROI
        # points even though the object is plainly in the RGB frame; see
        # memory look-settle-race-and-cube-occlusion). Only the OFFICIAL
        # look_captures window (started once settled) is reported/used.
        self.declare_parameter('look_settle_tol', 0.01)
        self.declare_parameter('look_settle_consec', 3)
        self.declare_parameter('look_settle_timeout', 3.0)
        # --- grasp / logging ---
        self.declare_parameter('auto_attach', False)
        self.declare_parameter('attach_object_id', '')
        # Let the grasp target's own CollisionObject touch the robot (ACM allow)
        # so IK + the plan can reach INTO the object being grasped; everything
        # else stays collision-checked. The target is the scene collision object
        # nearest the goal pose, within grasp_match_radius (m).
        self.declare_parameter('allow_target_collision', True)
        self.declare_parameter('grasp_match_radius', 0.12)
        self.declare_parameter('compute_traj_energy', False)
        self.declare_parameter('csv_log', '')
        # --- full pick-place cycle (opt-in; False = old approach+auto_attach-only
        # behaviour, unchanged) ---
        self.declare_parameter('do_grasp', False)
        # gripper_names/gripper_joints are index-aligned with arm_names, same
        # convention as arm_groups/arm_ee_frames/gripper_links above.
        self.declare_parameter('gripper_names', ['gripper_1', 'gripper_2'])
        self.declare_parameter('gripper_joints',
                               ['t1_a1_right_finger_bottom_joint',
                                't1_a2_right_finger_bottom_joint'])
        # Isaac-verified (isaac_sim/gripper.py, isaac_sim/workcell/grasp_verify.txt:
        # PASS, pad gap 0.085->0.020, drop 0.000m): increasing master joint OPENS
        # the gripper in THIS sim. CONFLICTS with run_take_bottle.py's real-Kortex-
        # hardware convention (degrees, increasing = CLOSES) -- do not reuse those
        # values here without re-verifying direction on the target platform.
        self.declare_parameter('gripper_open_pos', 0.96)
        self.declare_parameter('gripper_closed_pos', -0.09)
        self.declare_parameter('gripper_max_effort', 50.0)
        # How far (m, world -Z) to lower the EE from the reached pre-grasp pose
        # to actually enclose the object -- independent of box_clearance /
        # approach_offset so it is tunable without moving the pre-grasp aim point.
        self.declare_parameter('grasp_descend', 0.05)
        self.declare_parameter('lift_height', 0.15)   # m, world +Z rise after grasp
        self.declare_parameter('grasp_settle_s', 0.5)  # pause after close, before lift
        # Optional world (x, y, z) drop point. Empty = no place: hold the object
        # at lift height above the pick point (still a full grasp+lift cycle).
        self.declare_parameter('place_xyz', [0.0])
        self.declare_parameter('place_enabled', False)
        # --- session C: sampler-driven grasp cycle (~/grasp) ---
        # How far (m) to back off along the grasp pose's own approach axis
        # (local +Z, points at the fingertips) before descending open-loop --
        # the D405 is blind under ~0.07 m, matches approach_offset's role for
        # the old naive-descent cycle.
        self.declare_parameter('grasp_standoff', 0.10)
        # ~/sample_grasps call budget (<=0 in the request means "use the
        # node's own param" -- see reachability_gng_interfaces/srv/SampleGrasps).
        self.declare_parameter('sample_grasps_timeout', 15.0)
        # Achieved pad gap (from the gripper's closed joint position) vs the
        # sampler's predicted width[0] must agree within this margin (m) for
        # the grasp to count as success -- "closed on something" alone is not
        # enough (a stall a few cm off the real object would also report
        # closed). See isaac-grasping-wrist-only-plan Session C item 4.
        self.declare_parameter('grasp_width_tol', 0.015)
        # Tier-2/3 oblique escalation: when the top candidate's far contact
        # side was never actually observed (both_sides_observed False), take
        # another look from off-vertical and re-sample with the two views
        # FUSED (SampleGrasps.accumulate). Tier 3 is tier 2 swung 90 deg
        # around in azimuth, for objects where one oblique still is not
        # enough. Set grasp_oblique_tiers 0 to disable and keep the old
        # nadir-only behaviour.
        self.declare_parameter('grasp_oblique_tiers', 2)
        self.declare_parameter('grasp_oblique_tilt_deg', 37.0)
        self.declare_parameter('grasp_oblique_azimuth_deg', 0.0)

        g = self.get_parameter
        names = list(g('arm_names').value)
        models = list(g('arm_models').value)
        groups = list(g('arm_groups').value)
        ees = list(g('arm_ee_frames').value)
        grips = list(g('gripper_links').value)
        self.pin_configs = list(g('arm_pin_configs').value)
        self.task = g('task').value
        self.ori_weight = float(g('ori_weight').value)
        self.grasp_ori = [float(v) for v in g('grasp_orientation').value]
        self.grasp_yaw_samples = int(g('grasp_yaw_samples').value)
        self.grasp_tilt_samples = int(g('grasp_tilt_samples').value)
        self.grasp_tilt_max = float(g('grasp_tilt_max').value)
        self.grasp_tilt_azimuths = int(g('grasp_tilt_azimuths').value)
        self.approach_offset = float(g('approach_offset').value)
        self.box_clearance = float(g('box_clearance').value)
        # Let `ros2 param set` retune the stand-off live (no restart), since these
        # are the knobs iterated during tuning.
        self.add_on_set_parameters_callback(self._on_set_params)
        self.world_frame = g('world_frame').value
        self.pool_radius = float(g('pool_radius').value)
        self.pool_radius_factor = float(g('pool_radius_factor').value)
        self.max_candidates = int(g('max_candidates').value)
        self.n_gantry = int(g('n_gantry_dofs').value)
        self.w_gantry_lin = float(g('w_gantry_lin').value)
        self.w_gantry_rot = float(g('w_gantry_rot').value)
        self.w_arm = float(g('w_arm').value)
        self.w_manip = float(g('w_manip').value)
        self.w_hold = float(g('w_hold').value)
        self.w_dist = float(g('w_dist').value)
        self.ref_gantry_lin = float(g('ref_gantry_lin').value)
        self.ref_gantry_rot = float(g('ref_gantry_rot').value)
        self.ref_arm = float(g('ref_arm').value)
        self.ref_dist = float(g('ref_dist').value)
        self.ref_manip = float(g('ref_manip').value)
        self.ref_hold = float(g('ref_hold').value)
        self.log_j_table = bool(g('log_j_table').value)
        self.log_j_table_max = int(g('log_j_table_max').value)
        self.ik_timeout = float(g('ik_timeout').value)
        self.ik_wait = float(g('ik_wait').value)
        self.ik_avoid_collisions = bool(g('ik_avoid_collisions').value)
        self.plan_time = float(g('plan_time').value)
        self.exec_wait = float(g('exec_wait').value)
        self.reach_tol = float(g('reach_tol').value)
        self.plan_attempts = int(g('plan_attempts').value)
        self.plan_wait = float(g('plan_wait').value)
        self.vel_scale = float(g('vel_scale').value)
        self.acc_scale = float(g('acc_scale').value)
        self.joint_tol = float(g('joint_tolerance').value)
        self.execute = bool(g('execute').value)
        self.max_attempts = int(g('max_attempts').value)
        self.winner_head_start = max(1, int(g('winner_head_start').value))
        self.in_place_radius = float(g('in_place_radius').value)
        self.gate_perception = bool(g('gate_perception').value)
        self.gate_settle = float(g('gate_settle').value)
        self.auto_attach = bool(g('auto_attach').value)
        self.attach_object_id = g('attach_object_id').value
        self.allow_target_collision = bool(g('allow_target_collision').value)
        self.grasp_match_radius = float(g('grasp_match_radius').value)
        self.compute_traj_energy = bool(g('compute_traj_energy').value)
        self.csv_log = g('csv_log').value
        self.do_grasp = bool(g('do_grasp').value)
        self.gripper_names = list(g('gripper_names').value)
        self.gripper_joints = list(g('gripper_joints').value)
        self.gripper_open_pos = float(g('gripper_open_pos').value)
        self.gripper_closed_pos = float(g('gripper_closed_pos').value)
        self.gripper_max_effort = float(g('gripper_max_effort').value)
        self.grasp_descend = float(g('grasp_descend').value)
        self.lift_height = float(g('lift_height').value)
        self.grasp_settle_s = float(g('grasp_settle_s').value)
        self.wrist_frames = list(g('wrist_frames').value)
        self.wrist_cloud_topics = list(g('wrist_cloud_topics').value)
        self.look_distance = float(g('look_distance').value)
        self.look_roll_samples = int(g('look_roll_samples').value)
        self.look_captures = int(g('look_captures').value)
        self.look_settle_s = float(g('look_settle_s').value)
        self.look_timeout = float(g('look_timeout').value)
        self.look_roi = float(g('look_roi').value)
        self.look_settle_tol = float(g('look_settle_tol').value)
        self.grasp_standoff = float(g('grasp_standoff').value)
        self.sample_grasps_timeout = float(g('sample_grasps_timeout').value)
        self.grasp_width_tol = float(g('grasp_width_tol').value)
        self.grasp_oblique_tiers = int(g('grasp_oblique_tiers').value)
        self.grasp_oblique_tilt = np.radians(float(g('grasp_oblique_tilt_deg').value))
        self.grasp_oblique_azimuth = np.radians(
            float(g('grasp_oblique_azimuth_deg').value))
        self.look_settle_consec = int(g('look_settle_consec').value)
        self.look_settle_timeout = float(g('look_settle_timeout').value)
        self.place_enabled = bool(g('place_enabled').value)
        self.place_xyz = (np.array([float(v) for v in g('place_xyz').value])
                          if self.place_enabled else None)
        if self.place_enabled and (self.place_xyz is None or len(self.place_xyz) != 3):
            self.get_logger().error(
                'place_enabled=true but place_xyz is not a 3-vector -- place step '
                'will be skipped')
            self.place_enabled = False

        self.arms = []
        for nm, mp, grp, ee, gl in zip(names, models, groups, ees, grips):
            try:
                arm = ArmModel(nm, mp, grp, ee, gl)
                arm.eff_radius = (self.pool_radius if self.pool_radius > 0.0
                                  else self.pool_radius_factor * arm.spacing)
                idx = len(self.arms)
                arm.gripper_name = (self.gripper_names[idx]
                                    if idx < len(self.gripper_names) else None)
                arm.gripper_joint = (self.gripper_joints[idx]
                                     if idx < len(self.gripper_joints) else None)
                arm.wrist_frame = (self.wrist_frames[idx]
                                   if idx < len(self.wrist_frames) else '')
                arm.wrist_tf = None          # cached T_tool<-camera, see below
                arm.tool_from_gripper = None  # cached T_tool<-gripper_base_link
                arm.wrist_cloud_topic = (self.wrist_cloud_topics[idx]
                                         if idx < len(self.wrist_cloud_topics)
                                         else '')
                arm.wrist_lock = threading.Lock()
                arm.wrist_buf = []           # msgs collected during a capture
                arm.wrist_capturing = False
                arm.wrist_target_n = 0
                self.arms.append(arm)
                self.get_logger().info(
                    f'loaded {nm}: {mp} -> group {grp}, ee {ee} '
                    f'(spacing={arm.spacing:.3f} m, pool_radius='
                    f'{arm.eff_radius:.3f} m)')
            except (OSError, KeyError) as e:
                self.get_logger().error(f'could not load {nm} ({mp}): {e}')
        if not self.arms:
            raise SystemExit('no arm models loaded; nothing to do')
        if self.do_grasp:
            missing = [a.name for a in self.arms if not a.gripper_name]
            if missing:
                self.get_logger().error(
                    f'do_grasp=true but gripper_names has no entry for {missing} '
                    f'-- grasp will fail for those arms (check gripper_names/'
                    f'gripper_joints list length vs arm_names)')

        self._pin_cache = {}            # arm name -> (pin, model, data, order)
        self._latest_objects = None
        self._latest_target = None      # /target_object, the single grasp target
        self._target_boxes = []         # [(center3, size3)] fitted target box(es)
        self._labels = {}               # marker id -> label text (class/color name)
        self._target_label = ''         # latest /grasp_target (target-pick name)
        self._joints = {}               # joint name -> position
        self._acm_allowed = set()       # object ids already ACM-allowed to touch
        self._busy = threading.Lock()
        self._pick_active = False       # drives the perception-pause heartbeat

        # TF, to read each arm's CURRENT tool-frame position for the J `dist` term
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cb = ReentrantCallbackGroup()
        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1, callback_group=cb)
        self.create_subscription(PoseStamped, '/target_object',
                                 self._on_target, 1, callback_group=cb)
        # target's fitted 3D box(es) from object_collision -> EE stand-off height
        self.create_subscription(String, '/target_collision_boxes',
                                 self._on_target_boxes, latched_qos(),
                                 callback_group=cb)
        # object labels (class/color, e.g. 'yellow bottle') for terminal output
        self.create_subscription(MarkerArray, '/detected_objects/markers',
                                 self._on_markers, 1, callback_group=cb)
        self.create_subscription(String, '/grasp_target',
                                 self._on_grasp_target, 10, callback_group=cb)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joints, 10, callback_group=cb)
        self.create_subscription(String, '~/pick', self._on_pick, 1,
                                 callback_group=cb)
        # session A2: stop-and-stare wrist-camera acquisition, same arg format
        # as ~/pick (index into /detected_objects, or blank for /target_object).
        self.create_subscription(String, '~/look', self._on_look, 1,
                                 callback_group=cb)
        # session C: sampler-driven grasp cycle (look -> sample_grasps ->
        # stand-off -> open-loop descend -> close -> check -> lift), same arg
        # format as ~/pick / ~/look.
        self.create_subscription(String, '~/grasp', self._on_grasp, 1,
                                 callback_group=cb)
        for arm in self.arms:
            if not arm.wrist_cloud_topic:
                continue
            self.create_subscription(
                PointCloud2, arm.wrist_cloud_topic,
                lambda m, a=arm: self._on_wrist_cloud(a, m),
                qos_profile_sensor_data, callback_group=cb)
        self.attach_pub = self.create_publisher(
            String, '/object_collision/command', 1)
        self.pause_pub = self.create_publisher(Bool, PAUSE_TOPIC, latched_qos())
        # Heartbeat: while a pick runs, re-assert the pause every few seconds so
        # perception stays frozen for the WHOLE pick (even a slow multi-candidate
        # one) yet auto-resumes shortly after the executor stops/dies.
        self.create_timer(3.0, self._pause_heartbeat, callback_group=cb)
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik',
                                         callback_group=cb)
        self.get_scene_cli = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=cb)
        self.apply_scene_cli = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=cb)
        self.sample_grasps_cli = self.create_client(
            SampleGrasps, '/grasp_sampler/sample_grasps', callback_group=cb)
        self.move_cli = ActionClient(self, MoveGroup, 'move_action',
                                     callback_group=cb)
        # One GripperCommand client per arm that has a gripper_name, action name
        # convention confirmed against moveit_controllers.yaml + the existing
        # take_bottle_demo runner (workcell_description/scripts/run_take_bottle.py):
        # /<gripper_name>_controller/gripper_cmd.
        self.gripper_clients = {
            a.name: ActionClient(self, GripperCommand,
                                 f'/{a.gripper_name}_controller/gripper_cmd',
                                 callback_group=cb)
            for a in self.arms if a.gripper_name}

        if self.csv_log:
            self._init_csv()
        self.get_logger().info(
            f'gantry_reach_executor up; arms={[a.name for a in self.arms]}, '
            f'execute={self.execute}, pool_radius={self.pool_radius} m. '
            f'Trigger: ros2 topic pub --once {self.get_name()}/pick '
            f'std_msgs/String "{{data: \'0\'}}"')

    # ---- subscribers --------------------------------------------------------
    def _on_objects(self, msg):
        self._latest_objects = msg

    def _on_target(self, msg):
        self._latest_target = msg

    def _on_target_boxes(self, msg):
        """Latest fitted target box(es) as [(center3, size3)] for the EE stand-off."""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._target_boxes = [
            (np.asarray(b['center'], float), np.asarray(b['size'], float))
            for b in data if 'center' in b and 'size' in b]

    def _on_markers(self, msg):
        for m in msg.markers:
            if m.ns == 'labels':
                self._labels[m.id] = m.text

    def _on_grasp_target(self, msg):
        self._target_label = msg.data.strip()

    def _obj_name(self, idx):
        """Human label for logs: the grasp-target label for a /target_object pick,
        else the marker label at that index, falling back to 'obj<idx>'."""
        if idx == 'target':
            return self._target_label or 'target'
        return self._labels.get(idx) or f'obj{idx}'

    def _on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self._joints[n] = p

    def _on_wrist_cloud(self, arm, msg):
        """Append to `arm`'s capture buffer ONLY while a `~/look` stop-and-stare
        capture is active (see _do_look) -- a no-op the rest of the time, so
        this camera's normal ~10 Hz stream costs one lock+bool check per frame
        when idle. Runs on whichever MultiThreadedExecutor thread rclpy assigns
        this callback (ReentrantCallbackGroup, same as every other subscription
        on this node), concurrently with _do_look's blocking poll -- that
        concurrency is exactly why the capture is buffered here instead of
        _do_look reaching into a subscription itself."""
        with arm.wrist_lock:
            if arm.wrist_capturing and len(arm.wrist_buf) < arm.wrist_target_n:
                arm.wrist_buf.append(msg)

    def _on_pick(self, msg):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('pick already in progress; ignoring')
            return
        try:
            self._do_pick(msg.data.strip())
        except Exception as e:                       # never die on one pick
            self.get_logger().error(f'pick failed: {e}')
        finally:
            self._busy.release()

    def _on_look(self, msg):
        # Shares _busy with ~/pick: both drive the arm through _move_to_pose /
        # _wait_until_reached, which poll the same live joint-state state --
        # letting a pick and a look race would be a live-arm collision hazard,
        # not just a logic bug.
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('pick/look already in progress; ignoring')
            return
        try:
            self._do_look(msg.data.strip())
        except Exception as e:                       # never die on one look
            self.get_logger().error(f'look failed: {e}')
        finally:
            self._busy.release()

    def _on_grasp(self, msg):
        # Shares _busy with ~/pick and ~/look for the same reason: all three
        # drive the arm through _move_to_pose, and a collision hazard on a
        # live arm is not something to risk on a logic race.
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('pick/look/grasp already in progress; ignoring')
            return
        try:
            self._do_grasp(msg.data.strip())
        except Exception as e:                       # never die on one grasp
            self.get_logger().error(f'grasp failed: {e}')
        finally:
            self._busy.release()

    def _on_set_params(self, params):
        """Apply live `ros2 param set` for the tunable stand-off knobs so they
        take effect on the NEXT pick without a restart."""
        # attr name is the param name for the J weight/reference knobs, so they
        # can be retuned live (next pick) while reading the J table.
        live = ('w_gantry_lin', 'w_gantry_rot', 'w_arm', 'w_manip', 'w_hold',
                'w_dist', 'ref_gantry_lin', 'ref_gantry_rot', 'ref_arm',
                'ref_dist', 'ref_manip', 'ref_hold')
        for p in params:
            if p.name == 'box_clearance':
                self.box_clearance = float(p.value)
            elif p.name == 'approach_offset':
                self.approach_offset = float(p.value)
            elif p.name == 'grasp_width_tol':
                self.grasp_width_tol = float(p.value)
            elif p.name in live:
                setattr(self, p.name, float(p.value))
        return SetParametersResult(successful=True)

    # ---- helpers ------------------------------------------------------------
    def _pregrasp_point(self, obj_xyz):
        """Pre-grasp (x, y, z) above the object plus whether a fitted box was used.

        If object_collision has a target box near obj_xyz, stand the EE off
        box_clearance ABOVE the box top (x, y at the box centre) so a tall
        object's body is not entered. Otherwise fall back to the object centroid
        + approach_offset (legacy)."""
        best, bestd = None, self.grasp_match_radius
        for center, size in self._target_boxes:
            d = float(np.linalg.norm(center - obj_xyz))
            if d < bestd:
                best, bestd = (center, size), d
        if best is not None:
            center, size = best
            return (float(center[0]), float(center[1]),
                    float(center[2] + size[2] / 2.0 + self.box_clearance), True)
        return (float(obj_xyz[0]), float(obj_xyz[1]),
                float(obj_xyz[2] + self.approach_offset), False)

    def _task_vec(self, pose):
        p = pose.position
        if self.task == 'pos':
            return np.array([p.x, p.y, p.z])
        o = pose.orientation
        return np.array([p.x, p.y, p.z,
                         o.x * self.ori_weight, o.y * self.ori_weight,
                         o.z * self.ori_weight, o.w * self.ori_weight])

    def _current_q(self, arm):
        """Current positions of `arm`'s joints, or None if not all known yet."""
        if not arm.joint_names:
            return None
        try:
            return np.array([self._joints[n] for n in arm.joint_names])
        except KeyError:
            return None

    def _current_ee(self, arm):
        """Arm's CURRENT tool-frame position in the world frame (via TF), or None
        if the transform is not available yet."""
        try:
            t = self._tf_buffer.lookup_transform(
                self.world_frame, arm.ee_frame, Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(
                f'{arm.name}: TF {self.world_frame}->{arm.ee_frame} '
                f'unavailable ({e}); dist term = 0')
            return None
        tr = t.transform.translation
        return np.array([tr.x, tr.y, tr.z])

    def _wait_until_reached(self, arm, goal_q, tol, timeout):
        """Block until arm's LIVE joints converge to goal_q (the real Isaac Sim
        state from /joint_states), so SUCCESS reflects the physical arm rather
        than move_group's action result (which returns before slow sim physics
        settle). Returns (reached, max_err). _on_joints runs on another executor
        thread, so the polled state updates while we sleep here."""
        deadline = time.time() + timeout
        err = float('inf')
        while time.time() < deadline:
            cur = self._current_q(arm)
            if cur is not None:
                err = float(np.max(np.abs(goal_q[:len(cur)] - cur)))
                if err <= tol:
                    return True, err
            time.sleep(0.1)
        return False, err

    def _wait(self, future, timeout):
        """Block until an async future completes (MultiThreadedExecutor spins it
        on another thread) without nesting executor spins."""
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None
        return future.result()

    def _score(self, q, cur):
        # gantry DOF order is [linear, rotation, ...]: split so each is scored by
        # its own weight (metres vs radians -- summing them was unit-inconsistent).
        if cur is not None:
            dq = np.abs(q[:len(cur)] - cur)
            d_gantry_lin = float(dq[0]) if self.n_gantry >= 1 else 0.0
            d_gantry_rot = float(dq[1:self.n_gantry].sum())
            d_arm = float(dq[self.n_gantry:].sum())
        else:
            d_gantry_lin = d_gantry_rot = d_arm = 0.0
        return d_gantry_lin, d_gantry_rot, d_arm

    @staticmethod
    def _interleave_by_arm(by_J, head_start=1):
        """Round-robin the J-sorted candidates across arms, giving the allocation
        winner (overall lowest-J arm, whose bucket is first) a `head_start`: its
        first `head_start` nodes are tried BEFORE any other arm gets a turn, then
        the remaining candidates round-robin across arms with the OTHER arms ahead
        of the winner's tail so every arm is still guaranteed a shot within
        max_attempts. Ascending-J order is preserved within each arm's bucket.
        head_start=1 reproduces strict round-robin (best-A, best-B, 2nd-A, ...).
        """
        buckets = {}
        for c in by_J:                       # by_J is already ascending-J
            buckets.setdefault(c['arm'].name, []).append(c)
        lists = list(buckets.values())
        winner = lists[0]
        order = list(winner[:head_start])
        # remaining rounds: other arms first, then the winner's tail, so after the
        # head start no arm monopolises the rest.
        rest = list(lists[1:]) + [winner[head_start:]]
        for i in range(max((len(b) for b in rest), default=0)):
            for b in rest:
                if i < len(b):
                    order.append(b[i])
        return order

    def _log_j_table(self, name, by_J, dist_rank):
        """Print the full ranked J table -- every pooled candidate in ascending
        J, with each term's weighted contribution (weight*value), so the J
        calculation is fully visible in the terminal. `*` marks the winner."""
        rows = ['', f'>>> {name}: J ranking ({len(by_J)} candidates, '
                    f'weights glin={self.w_gantry_lin:g} grot={self.w_gantry_rot:g} '
                    f'arm={self.w_arm:g} dist={self.w_dist:g} '
                    f'hold={self.w_hold:g} manip={self.w_manip:g}; '
                    f'refs glin={self.ref_gantry_lin:g} grot={self.ref_gantry_rot:g} '
                    f'arm={self.ref_arm:g} dist={self.ref_dist:g} '
                    f'hold={self.ref_hold:g} manip={self.ref_manip:g})',
                '  rank arm     node      J | glin  grot   arm eedist  hold  manip '
                '(rank_dist)']
        for r, c in enumerate(by_J[:self.log_j_table_max]):
            glin = self.w_gantry_lin * c['d_gantry_lin'] / self.ref_gantry_lin
            grot = self.w_gantry_rot * c['d_gantry_rot'] / self.ref_gantry_rot
            arm = self.w_arm * c['d_arm'] / self.ref_arm
            eedist = self.w_dist * c['ee_dist'] / self.ref_dist
            hold = self.w_hold * c['hold'] / self.ref_hold
            manip = -self.w_manip * c['manip'] / self.ref_manip
            mark = '*' if r == 0 else ' '
            rows.append(
                f'{mark} {r:>3} {c["arm"].name:<7} {c["node"]:>4} '
                f'{c["J"]:>7.3f} | {glin:>5.2f} {grot:>5.2f} {arm:>5.2f} '
                f'{eedist:>5.2f} {hold:>5.2f} {manip:>6.2f}  '
                f'({dist_rank[id(c)]})')
        if len(by_J) > self.log_j_table_max:
            rows.append(f'  ... {len(by_J) - self.log_j_table_max} more '
                        f'(raise log_j_table_max to see all)')
        self.get_logger().info('\n'.join(rows))

    # ---- target resolution ---------------------------------------------------
    def _resolve_target(self, arg, verb='pick'):
        """Resolve a `~/pick` / `~/look` string arg to (target Pose, idx, name),
        or (None, None, None) on failure (already logged). Shared so `~/look`
        (session A2) picks the SAME object `~/pick` would for the same arg.

        Prefer the explicit index the caller sent against the LIVE
        /detected_objects -- that is exactly what pick_cli listed and the user
        just selected, so each request hits the object chosen. Only fall back to
        the /target_object topic when no usable index was given (e.g. a bare
        `ros2 topic pub` in target_label mode): that topic is refreshed on
        object_localizer's timer and lags a fresh selection, so preferring it
        would re-select the PREVIOUS object (the bug this fixes)."""
        objs = self._latest_objects
        target, idx = None, None
        if arg.isdigit() and objs is not None and objs.poses:
            i = int(arg)
            if 0 <= i < len(objs.poses):
                target, idx = objs.poses[i], i
            else:
                self.get_logger().warn(
                    f'object index {i} out of range (have {len(objs.poses)})')
                return None, None, None
        if target is None:
            if self._latest_target is not None:
                target, idx = self._latest_target.pose, 'target'
            else:
                self.get_logger().warn(
                    f'{verb} arg "{arg}": no matching /detected_objects index '
                    f'and no /target_object yet')
                return None, None, None
        return target, idx, self._obj_name(idx)

    # ---- main pick ----------------------------------------------------------
    def _do_pick(self, arg):
        target, idx, name = self._resolve_target(arg)
        if target is None:
            return
        p = target.position
        self.get_logger().info(
            f'>>> NEW TARGET {name}: pick requested at '
            f'x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f} (world) -- allocating arm')
        tvec = self._task_vec(target)
        cur_by_arm = {a.name: self._current_q(a) for a in self.arms}
        # `dist` in J is the gap from each arm's CURRENT end-effector to the
        # object -- one value per arm (same for all its nodes), so it biases the
        # allocation toward the arm whose tool is already nearer the object.
        obj_xyz = np.array([target.position.x, target.position.y,
                            target.position.z])
        ee_dist_by_arm = {}
        for a in self.arms:
            ee = self._current_ee(a)
            ee_dist_by_arm[a.name] = (float(np.linalg.norm(ee - obj_xyz))
                                      if ee is not None else 0.0)

        # 1) pool every arm's in-radius candidates, scored by J
        cands = []
        for arm in self.arms:
            ee_dist = ee_dist_by_arm[arm.name]
            for node_idx, q, dist, manip, hold in arm.candidates(
                    tvec, arm.eff_radius, self.max_candidates):
                d_gantry_lin, d_gantry_rot, d_arm = self._score(
                    q, cur_by_arm[arm.name])
                J = (self.w_gantry_lin * d_gantry_lin / self.ref_gantry_lin
                     + self.w_gantry_rot * d_gantry_rot / self.ref_gantry_rot
                     + self.w_arm * d_arm / self.ref_arm
                     + self.w_dist * ee_dist / self.ref_dist
                     + self.w_hold * hold / self.ref_hold
                     - self.w_manip * manip / self.ref_manip)
                cands.append(dict(arm=arm, node=node_idx, q=q, dist=dist,
                                  ee_dist=ee_dist, manip=manip, hold=hold,
                                  d_gantry_lin=d_gantry_lin,
                                  d_gantry_rot=d_gantry_rot,
                                  d_arm=d_arm, J=J))

        # 1b) in-place seed for any arm already over the object: IK seeded from
        # its CURRENT config, scored with ZERO joint travel (it IS the current
        # state, manip unknown -> 0), so J = w_dist*ee_dist leads that arm's
        # bucket and is tried first -- re-grasp in place instead of a far arm
        # driving the shared gantry to a target this arm is already hovering over.
        if self.in_place_radius > 0.0:
            for arm in self.arms:
                q_cur = cur_by_arm[arm.name]
                ee_dist = ee_dist_by_arm[arm.name]
                if q_cur is None or ee_dist > self.in_place_radius:
                    continue
                cands.append(dict(arm=arm, node=-1, q=q_cur.copy(), dist=ee_dist,
                                  ee_dist=ee_dist, manip=0.0, hold=0.0,
                                  d_gantry_lin=0.0, d_gantry_rot=0.0, d_arm=0.0,
                                  J=self.w_dist * ee_dist / self.ref_dist))
                self.get_logger().info(
                    f'{name}: {arm.name} already over target '
                    f'(ee_dist={ee_dist:.3f} <= {self.in_place_radius:.2f} m) '
                    f'-- added in-place re-grasp candidate (node -1)')

        if not cands:
            self.get_logger().error(
                f'>>> {name}: FAILED -- no reachable arm candidates in pool '
                f'(object out of every arm\'s GNG reach map)')
            return

        by_J = sorted(cands, key=lambda c: c['J'])
        # rank-by-distance, to later show energy may pick a non-nearest seed
        dist_rank = {id(c): r for r, c in
                     enumerate(sorted(cands, key=lambda c: c['dist']))}
        # Fallback order: round-robin ACROSS arms (best-per-arm first), keeping
        # ascending-J WITHIN each arm. Otherwise the lowest-J arm's cluster of
        # near-identical nodes fills all max_attempts and the other arm is never
        # tried -- a whole arm's worth of -31s/collisions is spent before the
        # alternative ever gets a single shot.
        attempt_order = self._interleave_by_arm(by_J, self.winner_head_start)

        if self.log_j_table:
            self._log_j_table(name, by_J, dist_rank)

        # Announce the arm the energy allocation chose (lowest-J candidate). This
        # is the arm that WILL do the task; if its plan fails the executor falls
        # through to the next-best candidate (possibly the other arm) below.
        best = by_J[0]
        by_arm_bestJ = {}
        for c in by_J:
            by_arm_bestJ.setdefault(c['arm'].name, c['J'])
        summary = ', '.join(f'{n} J={j:.3f}' for n, j in by_arm_bestJ.items())
        self.get_logger().info(
            f'>>> {name}: allocation -> {best["arm"].name} will do the task '
            f'(best J={best["J"]:.3f} [gantry_lin={best["d_gantry_lin"]:.3f} '
            f'gantry_rot={best["d_gantry_rot"]:.3f} arm={best["d_arm"]:.3f} '
            f'eedist={best["ee_dist"]:.3f} '
            f'manip={best["manip"]:.3f}]; per-arm best: {summary})')

        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        # pre-grasp point: stand the EE off ABOVE the fitted box top (safe EE<->box
        # clearance) when a target box is known, else above the object centroid.
        px, py, pz, used_box = self._pregrasp_point(obj_xyz)
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        self.get_logger().info(
            f'{name}: pre-grasp z={pz:.3f} '
            + (f'(box top + {self.box_clearance:.3f} m clearance)' if used_box
               else f'(centroid + {self.approach_offset:.3f} m; no target box yet)'))
        if self.task == 'pose':
            o = target.orientation
            ori_list = [(o.x, o.y, o.z, o.w)]              # real detected ori
        else:
            # Centroid gives no usable orientation -> sample yaw about the fixed
            # down-approach so IK has several reachable targets (fixes -31 when a
            # single orientation is unreachable for the redundant arm+gantry).
            ori_list = self._grasp_ori_candidates()

        # allow the grasp target's own box to touch the robot (so IK + plan can
        # reach into it); everything else stays collision-checked
        if self.allow_target_collision:
            self._allow_target_collision(target)

        # Fix A: freeze the sensed scene so a mid-plan version bump can't drop the
        # plan; settle lets the last in-flight update land, then resume in finally.
        self._set_perception_pause(True)
        try:
            # Try candidates round-robin across arms. For each: IK, then a
            # plan-only CHECK (fast, no motion) to skip un-plannable ones without
            # ever moving the arm; the FIRST that plans is executed and confirmed
            # against the LIVE arm. If execution fails (e.g. env change -3), we
            # KEEP GOING to the next candidate instead of giving up -- so a
            # transient abort doesn't end the pick. Because each execution is
            # awaited to completion before the next, a slow gantry can't cause the
            # old mid-motion mis-fire (-4 / IK timeouts).
            fail = {'ik': [], 'plan': 0, 'exec': 0}   # why each candidate lost
            for attempt, c in enumerate(attempt_order[:self.max_attempts]):
                arm = c['arm']
                t0 = time.perf_counter()
                ok, js, ikerr = self._solve_ik(arm, ps, c['q'], ori_list)
                ik_ms = (time.perf_counter() - t0) * 1e3
                if not ok:
                    fail['ik'].append(ikerr)
                    self.get_logger().info(
                        f'{name} cand#{attempt} {arm.name} J={c["J"]:.3f}: '
                        f'IK failed (err={ikerr})')
                    continue
                goal_q = self._extract(js, arm.joint_names)
                if goal_q is None:
                    continue
                t1 = time.perf_counter()
                planned, traj, perr, ptime = self._plan(
                    arm, goal_q, do_execute=False)
                plan_ms = (time.perf_counter() - t1) * 1e3
                if not planned:
                    fail['plan'] += 1
                    self.get_logger().info(
                        f'{name} cand#{attempt} {arm.name} J={c["J"]:.3f}: '
                        f'plan failed (err={perr})')
                    continue

                energy = (self._traj_energy(arm, traj)
                          if self.compute_traj_energy else float('nan'))
                self.get_logger().info(
                    f'{name}: PICKED {arm.name} via cand#{attempt} '
                    f'(rank-by-dist {dist_rank[id(c)]}) J={c["J"]:.3f} '
                    f'[gantry_lin={c["d_gantry_lin"]:.3f} '
                    f'gantry_rot={c["d_gantry_rot"]:.3f} arm={c["d_arm"]:.3f} '
                    f'eedist={c["ee_dist"]:.3f} '
                    f'manip={c["manip"]:.3f}] '
                    f'gantry_goal=({goal_q[0]:.3f},{goal_q[1]:.3f}) '
                    f'ik={ik_ms:.0f}ms plan={plan_ms:.0f}ms '
                    f'{"executing" if self.execute else "plan-only"}')

                if not self.execute:                     # plan-only: first wins
                    self._log_csv(idx, attempt, dist_rank[id(c)], c, goal_q,
                                  ik_ms, ptime, plan_ms, energy)
                    return

                # Execute this candidate, then confirm against the LIVE arm
                # (/joint_states), since move_group's result can beat Isaac's
                # slower physics.
                ex_ok, _, eperr, _ = self._plan(arm, goal_q, do_execute=True)
                reached, err = (
                    self._wait_until_reached(
                        arm, goal_q, self.reach_tol, self.exec_wait)
                    if ex_ok else (False, float('inf')))
                grasp_ok, grasp_fail_reason = True, ''
                if reached and self.do_grasp:
                    # ps.pose.orientation is still the orientation _solve_ik just
                    # solved (it returns immediately on the first hit, so nothing
                    # has mutated ps since) -- reuse it rather than re-deriving.
                    quat = (ps.pose.orientation.x, ps.pose.orientation.y,
                           ps.pose.orientation.z, ps.pose.orientation.w)
                    grasp_ok = self._grasp_sequence(
                        arm, (px, py, pz), quat, goal_q, self.attach_object_id)
                    grasp_fail_reason = ' (approach OK, grasp sequence failed)'
                elif reached and self.auto_attach and self.attach_object_id:
                    self._attach(arm, self.attach_object_id)
                if reached and grasp_ok:
                    self.get_logger().info(
                        f'>>> {name}: SUCCESS -- {arm.name} reached the '
                        f'target (max joint err={err:.3f} <= {self.reach_tol})'
                        + (' and completed the full grasp cycle'
                           if self.do_grasp else ''))
                    self._log_csv(idx, attempt, dist_rank[id(c)], c, goal_q,
                                  ik_ms, ptime, plan_ms, energy)
                    return
                # Execution (or the grasp sequence) failed: fall through and try
                # the next candidate so the arm keeps trying to reach the target.
                fail['exec'] += 1
                self.get_logger().warn(
                    f'{name} cand#{attempt} {arm.name}: did NOT reach '
                    f'(exec err={eperr}, joint err={err:.3f}){grasp_fail_reason} '
                    f'-- trying next candidate')

            # Classify WHY every candidate lost so the terminal says it plainly.
            n_tried = min(len(by_J), self.max_attempts)
            n_ik31 = sum(1 for e in fail['ik'] if e == -31)
            if n_tried and n_ik31 == n_tried:
                why = ('UNREACHABLE -- no IK solution for any candidate; the '
                       'target is outside the arm+gantry workspace (or every '
                       'grasp pose collides). If it should be reachable, '
                       're-run with ik_avoid_collisions:=false to tell '
                       'kinematics from collision.')
            elif fail['exec']:
                why = (f'planned but execution aborted on {fail["exec"]} '
                       f'candidate(s) (env change / controller) -- target may '
                       f'still be reachable, retry.')
            elif fail['plan']:
                why = (f'{fail["plan"]} candidate(s) had IK but NO '
                       f'collision-free plan -- target blocked by an '
                       f'obstacle/octomap on the approach.')
            else:
                why = 'no viable candidate (IK/plan failed).'
            self.get_logger().error(
                f'>>> {name}: FAILED -- {why} '
                f'[{n_tried} candidates: {n_ik31} no-IK, '
                f'{fail["plan"]} no-plan, {fail["exec"]} exec-abort]')
            self._log_csv(idx, -1, -1, None, None, float('nan'),
                          float('nan'), float('nan'), float('nan'))
        finally:
            self._set_perception_pause(False)

    # ---- wrist-camera look poses --------------------------------------------
    def _tool_from_camera(self, arm):
        """T_tool<-camera as (R, p), read from TF -- deliberately NOT hardcoded.

        The mount geometry already lives in exactly one place: the static TF
        <gripper_base_link> -> <wrist>_camera_optical published by
        launch_workcell.sh (itself derived from ros2_bridge_gui.py's
        _add_wrist_camera()). The IK goal, however, is the arm's tool_frame,
        which is a DIFFERENT link -- so the numbers measured against
        gripper_base_link cannot be used here as-is. Composing the hop through
        TF keeps this in sync with the mount automatically instead of
        duplicating measured constants that would silently rot.

        Both hops are rigid, so the lookup is cached after the first success.
        Returns None (and logs) when the arm has no wrist camera or the TF is
        not up yet -- no fallback guess (Rule 12).
        """
        if arm.wrist_tf is not None:
            return arm.wrist_tf
        if not arm.wrist_frame:
            self.get_logger().error(
                f'{arm.name}: no wrist camera frame configured (wrist_frames) '
                '-- cannot compute a look pose for this arm')
            return None
        try:
            t = self._tf_buffer.lookup_transform(arm.ee_frame, arm.wrist_frame,
                                                 Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().error(
                f'{arm.name}: TF {arm.ee_frame}->{arm.wrist_frame} unavailable '
                f'({e}) -- is the wrist camera static TF publisher running?')
            return None
        tr, q = t.transform.translation, t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        p = np.array([tr.x, tr.y, tr.z])
        arm.wrist_tf = (R, p)
        self.get_logger().info(
            f'{arm.name}: T_{arm.ee_frame}<-{arm.wrist_frame} '
            f'p={np.round(p, 4)} q={np.round([q.x, q.y, q.z, q.w], 4)} (cached)')
        return arm.wrist_tf

    def _tool_from_gripper(self, arm):
        """T_tool<-gripper_base_link as (R, p), read from TF -- same reasoning
        as _tool_from_camera. ~/sample_grasps poses are for `arm.gripper_link`
        (e.g. t1_a1_gripper_base_link, see reachability_gng_interfaces/srv/
        SampleGrasps), but IK targets `arm.ee_frame` (tool_frame), a DIFFERENT
        link -- using a sampler pose as the _move_to_pose target directly
        would silently offset every grasp by the tool<->gripper mount
        distance (~0.116 m, measured in isaac-grasping-wrist-only-plan).
        Cached after the first success (rigid transform)."""
        if arm.tool_from_gripper is not None:
            return arm.tool_from_gripper
        try:
            t = self._tf_buffer.lookup_transform(arm.ee_frame, arm.gripper_link,
                                                 Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().error(
                f'{arm.name}: TF {arm.ee_frame}->{arm.gripper_link} unavailable '
                f'({e}) -- cannot convert a sampler grasp pose to an IK target')
            return None
        tr, q = t.transform.translation, t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        p = np.array([tr.x, tr.y, tr.z])
        arm.tool_from_gripper = (R, p)
        self.get_logger().info(
            f'{arm.name}: T_{arm.ee_frame}<-{arm.gripper_link} '
            f'p={np.round(p, 4)} q={np.round([q.x, q.y, q.z, q.w], 4)} (cached)')
        return arm.tool_from_gripper

    def _gripper_pose_to_tool(self, arm, p_wg, R_wg):
        """World-frame (gripper_base_link position, rotation) -> world-frame
        (tool_frame xyz, quat) for _move_to_pose, via T_world<-tool =
        T_world<-gripper o (T_tool<-gripper)^-1 -- same composition
        _look_poses uses for the wrist camera. Returns (None, None) if the
        tool<-gripper TF isn't available (already logged)."""
        tg = self._tool_from_gripper(arm)
        if tg is None:
            return None, None
        R_tg, p_tg = tg
        R_wt = R_wg @ R_tg.T
        p_wt = p_wg - R_wt @ p_tg
        return p_wt, R_to_quat(R_wt)

    def _look_poses(self, arm, obj_xyz, tilt=0.0, azimuth=0.0):
        """Ordered tool-frame goals [(xyz (3,), quat (x,y,z,w))] that put this
        arm's WRIST CAMERA `look_distance` m from `obj_xyz`, staring at it:

            T_world<-tool = T_world<-camera o (T_tool<-camera)^-1

        One entry per sampled roll about the view ray. Every entry gives the
        SAME camera view (only the image rotates), so the caller may simply take
        the first one that IK solves -- roll is a redundancy handle, not a
        preference order. Empty list = no wrist camera / TF missing.

        tilt/azimuth stay 0 for now: only the nadir tier is wired up. They are
        arguments rather than constants because the oblique tiers 2/3 differ
        from this ONLY in those two numbers.
        """
        tc = self._tool_from_camera(arm)
        if tc is None:
            return []
        R_tc, p_tc = tc
        out = []
        n = max(1, self.look_roll_samples)
        for i in range(n):
            roll = 2.0 * np.pi * i / n
            p_wc, R_wc = look_camera_pose(obj_xyz, self.look_distance, roll,
                                          tilt, azimuth)
            R_wt = R_wc @ R_tc.T
            p_wt = p_wc - R_wt @ p_tc
            out.append((p_wt, R_to_quat(R_wt)))
        return out

    def _finger_link_names(self, arm):
        """This arm's 4 finger-tip/proximal link names, derived from its
        configured gripper_link (e.g. 't1_a1_gripper_base_link' ->
        't1_a1_left_finger_dist_link', ...) -- same suffixes measured in
        query_gripper_frame.txt. Diagnostic-only (see _do_look): whether the
        wrist cloud's own gripper contaminates the capture is an explicitly
        open question, not yet answered either way."""
        prefix = arm.gripper_link.rsplit('gripper_base_link', 1)[0]
        return [prefix + s for s in
               ('left_finger_prox_link', 'left_finger_dist_link',
                'right_finger_prox_link', 'right_finger_dist_link')]

    def _reach_look_and_settle(self, obj_xyz, name, tilt=0.0, azimuth=0.0,
                               only_arm=None):
        """Move a wrist-camera arm to a look pose for `obj_xyz`, then
        block until the wrist cloud's ROI centroid stabilizes (progress-based
        settle: joints reaching goal_q does not mean the RENDERED cloud has
        caught up -- Isaac's render can lag physics under load). Probes
        frames until the ROI centroid stops moving between consecutive
        frames, instead of trusting a fixed sleep. See memory
        look-settle-race-and-cube-occlusion (a real capture came back 0/5
        empty right after a move, clean on immediate retry with no motion).

        Shared by `~/look` (session A2, which then does its own diagnostic
        capture window) and `~/grasp` (session C, which then calls
        ~/sample_grasps trusting the cloud is actually stable). Deliberately
        NOT going through the J/pool arm-selection machinery _do_pick uses --
        this just tries every arm that HAS a wrist camera, in arm_names
        order, and stops at the first that reaches a look pose. Returns the
        arm on success, or None (already logged) on failure.

        `tilt`/`azimuth` (rad) select the tier: 0/0 is the nadir tier 1, a
        non-zero tilt is the oblique tier 2/3 used to observe a contact side
        the nadir view could not see. `only_arm` restricts the search to one
        arm -- an oblique re-look must come from the SAME arm that took the
        nadir view, since the accumulated clouds are fused together.
        """
        cams = [a for a in self.arms if a.wrist_frame and a.wrist_cloud_topic]
        if only_arm is not None:
            cams = [a for a in cams if a is only_arm]
        if not cams:
            self.get_logger().error(
                'no arm has both wrist_frame and wrist_cloud_topic configured '
                '-- nothing can look')
            return None

        arm = reached = None
        for cand in cams:
            seed_q = self._current_q(cand)
            cand_poses = self._look_poses(cand, obj_xyz, tilt, azimuth)
            if not cand_poses:
                continue                      # _tool_from_camera already logged
            for xyz, quat in cand_poses:
                ok, _, _ = self._move_to_pose(cand, xyz, quat, seed_q)
                if ok:
                    arm, reached = cand, True
                    break
            if reached:
                break
        if not reached:
            tier = ('nadir' if tilt == 0.0
                    else f'oblique (tilt={np.degrees(tilt):.0f} deg, '
                         f'azimuth={np.degrees(azimuth):.0f} deg)')
            self.get_logger().error(
                f'{name}: no arm reached a {tier} look pose '
                f'({self.look_roll_samples} roll(s) tried per candidate arm)')
            return None

        self.get_logger().info(
            f'{arm.name}: look pose reached, waiting {self.look_settle_s:.2f}s '
            f'min then polling for a settled cloud (tol={self.look_settle_tol:.3f} m, '
            f'{self.look_settle_consec} consecutive, timeout='
            f'{self.look_settle_timeout:.1f}s)')
        if self.look_settle_s > 0:
            time.sleep(self.look_settle_s)

        probe_cap = self.look_captures + self.look_settle_consec + 20
        with arm.wrist_lock:
            arm.wrist_buf = []
            arm.wrist_target_n = probe_cap
            arm.wrist_capturing = True
        settle_deadline = time.time() + self.look_settle_timeout
        settled, last_centroid, stable, seen = False, None, 0, 0
        while time.time() < settle_deadline:
            with arm.wrist_lock:
                probe_msgs = list(arm.wrist_buf)
            for m in probe_msgs[seen:]:
                pts = _cloud_xyz(m)
                d = np.linalg.norm(pts - obj_xyz, axis=1)
                roi = pts[d <= self.look_roi]
                c = roi.mean(axis=0) if len(roi) else None
                if c is not None and last_centroid is not None and \
                        np.linalg.norm(c - last_centroid) <= self.look_settle_tol:
                    stable += 1
                else:
                    stable = 0
                last_centroid = c
                if stable >= self.look_settle_consec:
                    settled = True
                    break
            seen = len(probe_msgs)
            if settled:
                break
            time.sleep(0.05)
        with arm.wrist_lock:
            arm.wrist_capturing = False
        if not settled:
            self.get_logger().warn(
                f'{arm.name}: cloud never settled within '
                f'{self.look_settle_timeout:.1f}s (tol={self.look_settle_tol:.3f} m) '
                '-- proceeding anyway, results may reflect a still-moving scene')
        return arm

    def _do_look(self, arg):
        """Session A2: move `arm`'s wrist camera to the nadir look pose for the
        resolved target, settle (see _reach_look_and_settle), THEN collect
        `look_captures` fresh wrist-cloud frames (the capture buffer is
        cleared right before this, so nothing from the settle probe can be
        counted) and report per-capture point count / centroid plus the
        across-capture drift, so a live Isaac run can be checked against the
        pass bar (drift, point count, wrist-vs-ceiling agreement) rather than
        eyeballed.
        """
        target, idx, name = self._resolve_target(arg, verb='look')
        if target is None:
            return
        obj_xyz = np.array([target.position.x, target.position.y,
                            target.position.z])
        self.get_logger().info(
            f'>>> LOOK {name}: x={obj_xyz[0]:+.3f} y={obj_xyz[1]:+.3f} '
            f'z={obj_xyz[2]:+.3f} (world)')
        arm = self._reach_look_and_settle(obj_xyz, name)
        if arm is None:
            return

        with arm.wrist_lock:
            arm.wrist_buf = []
            arm.wrist_target_n = self.look_captures
            arm.wrist_capturing = True
        deadline = time.time() + self.look_timeout
        while time.time() < deadline:
            with arm.wrist_lock:
                n = len(arm.wrist_buf)
            if n >= self.look_captures:
                break
            time.sleep(0.05)
        with arm.wrist_lock:
            arm.wrist_capturing = False
            msgs = list(arm.wrist_buf)
        if len(msgs) < self.look_captures:
            self.get_logger().error(
                f'{arm.name}: only {len(msgs)}/{self.look_captures} wrist '
                f'captures arrived within {self.look_timeout:.1f}s '
                f'(topic {arm.wrist_cloud_topic}) -- reporting what came in')
        if not msgs:
            return

        finger_pts = []
        for link in self._finger_link_names(arm):
            try:
                t = self._tf_buffer.lookup_transform(
                    self.world_frame, link, Time())
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                continue
            tr = t.transform.translation
            finger_pts.append([tr.x, tr.y, tr.z])
        finger_pts = np.array(finger_pts) if finger_pts else np.zeros((0, 3))

        centroids, counts = [], []
        for i, msg in enumerate(msgs):
            pts = _cloud_xyz(msg)
            d = np.linalg.norm(pts - obj_xyz, axis=1)
            roi = pts[d <= self.look_roi]
            n_roi = len(roi)
            counts.append(n_roi)
            centroid = roi.mean(axis=0) if n_roi else None
            centroids.append(centroid)
            n_finger = 0
            if n_roi and len(finger_pts):
                fd = np.linalg.norm(
                    roi[:, None, :] - finger_pts[None, :, :], axis=2)
                n_finger = int((fd.min(axis=1) <= 0.03).sum())
            stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            off = (f'{np.round(centroid - obj_xyz, 4)}'
                  if centroid is not None else 'n/a')
            self.get_logger().info(
                f'{arm.name}: capture {i}: stamp={stamp} n_roi={n_roi} '
                f'centroid_offset={off} n_within_3cm_of_finger={n_finger}')

        valid = [c for c in centroids if c is not None]
        if len(valid) >= 2:
            V = np.array(valid)
            drift = np.linalg.norm(
                V[:, None, :] - V[None, :, :], axis=2).max()
        else:
            drift = float('nan')
        wrist_vs_ceiling = (float(np.linalg.norm(np.mean(valid, axis=0)
                                                  - obj_xyz))
                           if valid else float('nan'))
        self.get_logger().info(
            f'>>> {arm.name}: LOOK {name} summary -- {len(valid)}/'
            f'{len(msgs)} captures with points in ROI, max pairwise centroid '
            f'drift={drift:.4f} m, mean-centroid-vs-ceiling={wrist_vs_ceiling:.4f} m, '
            f'n_roi min/max={min(counts) if counts else 0}/'
            f'{max(counts) if counts else 0}')

    def _finger_roi_count(self, arm, obj_xyz, n_captures=3, timeout=2.0):
        """Fresh wrist-cloud probe: median (across n_captures frames) count of
        points near the object's ROI that sit within 3cm of a finger link --
        i.e. is something still between the fingers. Post-grasp visual check:
        Gen3 Lite has no torque sensor (see gen3-lite-no-torque-sensor), so
        this plus the pad-gap check is the closest thing to a force signal
        available. Returns 0 (with a warning, not a silent pass) if finger TF
        is unavailable."""
        finger_pts = []
        for link in self._finger_link_names(arm):
            try:
                t = self._tf_buffer.lookup_transform(
                    self.world_frame, link, Time())
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                continue
            tr = t.transform.translation
            finger_pts.append([tr.x, tr.y, tr.z])
        finger_pts = np.array(finger_pts) if finger_pts else np.zeros((0, 3))
        if not len(finger_pts):
            self.get_logger().warn(
                f'{arm.name}: no finger TF available -- post-grasp visual '
                'check cannot run (reporting 0, do NOT treat as a pass)')
            return 0

        with arm.wrist_lock:
            arm.wrist_buf = []
            arm.wrist_target_n = n_captures
            arm.wrist_capturing = True
        deadline = time.time() + timeout
        while time.time() < deadline:
            with arm.wrist_lock:
                n = len(arm.wrist_buf)
            if n >= n_captures:
                break
            time.sleep(0.05)
        with arm.wrist_lock:
            arm.wrist_capturing = False
            msgs = list(arm.wrist_buf)
        if not msgs:
            self.get_logger().warn(
                f'{arm.name}: no wrist frames arrived for the post-grasp '
                'check within {timeout:.1f}s')
            return 0
        counts = []
        for msg in msgs:
            pts = _cloud_xyz(msg)
            d = np.linalg.norm(pts - obj_xyz, axis=1)
            roi = pts[d <= self.look_roi]
            if not len(roi):
                counts.append(0)
                continue
            fd = np.linalg.norm(roi[:, None, :] - finger_pts[None, :, :], axis=2)
            counts.append(int((fd.min(axis=1) <= 0.03).sum()))
        return int(np.median(counts))

    def _do_grasp(self, arg):
        """Run one grasp cycle and ALWAYS end with a single, uniform verdict
        line -- `GRASP SUCCESS` or `GRASP FAILED -- <reason>`.

        The verdict is emitted here rather than at each early return inside
        _grasp_cycle because several of those paths used to exit with only
        their own specific message (e.g. 'stand-off unreachable'), so anything
        watching the log for a result could wait forever on a cycle that had
        in fact already finished. One exit, one marker (Rule 12: a failure
        that leaves no uniform trace is a silent failure).
        """
        why = self._grasp_cycle(arg)
        if why is not None:
            self.get_logger().error(f'>>> {arg}: GRASP FAILED -- {why}')
        return why

    def _grasp_cycle(self, arg):
        """Session C: sampler-driven grasp cycle. Returns None on success, or
        a short reason string on failure (the caller logs the verdict).

        look -> ~/sample_grasps for
        REAL geometry (not the fixed-descend _grasp_sequence's naive
        approach) -> back off to a stand-off along the grasp pose's own
        approach axis -> descend open-loop (D405 is blind under ~0.07 m, no
        visual servoing to contact) -> close -> verify against BOTH the
        sampler's predicted width and a post-grasp wrist-cloud check -> lift.
        Every stage failure is reported plainly (Rule 12) -- no silent
        partial success, and a both_sides_observed=False top candidate is
        logged as a documented risk (tier-2 oblique look not implemented
        yet), not silently ignored.
        """
        self._set_perception_pause(True)
        name = arg
        try:
            # Perception is paused BEFORE resolving the target, so the index
            # stays valid for the whole cycle (object_localizer freezes while
            # paused) -- see memory detected-objects-index-unstable.
            target, idx, name = self._resolve_target(arg, verb='grasp')
            if target is None:
                return 'target could not be resolved'
            obj_xyz = np.array([target.position.x, target.position.y,
                                target.position.z])
            self.get_logger().info(
                f'>>> GRASP {name}: x={obj_xyz[0]:+.3f} y={obj_xyz[1]:+.3f} '
                f'z={obj_xyz[2]:+.3f} (world)')

            if not self.sample_grasps_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    f'{name}: /grasp_sampler/sample_grasps not available')
                return 'sample_grasps service not available'

            # Tiered look: nadir first, then oblique re-looks FUSED onto it
            # (accumulate) while the top candidate's far contact side is still
            # unobserved. Escalation stops on both_sides_observed, never on a
            # score threshold -- an unobserved far side is missing evidence,
            # not a low score, and the two are not interchangeable.
            arm = resp = None
            n_tiers = 1 + max(0, self.grasp_oblique_tiers)
            for tier in range(n_tiers):
                if tier == 0:
                    tilt, azim = 0.0, 0.0
                else:
                    tilt = self.grasp_oblique_tilt
                    # OPPOSITE azimuths (0, 180, ...), not +90 steps. The flag
                    # being chased is both_sides_observed: a grasp's two
                    # contacts sit on OPPOSING faces along the closing axis, so
                    # the evidence needed is a view from the other side of that
                    # axis. A +90 step only shows an adjacent face and leaves
                    # the far contact just as unobserved -- verified in
                    # test/verify_grasp_sampler.py, whose passing two-view case
                    # is az 0/180 (antipodality 0.998, both_sides True), and
                    # live, where a 0/90 pair left both_sides False on all
                    # three tiers.
                    azim = self.grasp_oblique_azimuth + (tier - 1) * np.pi
                    self.get_logger().info(
                        f'{name}: escalating to oblique tier {tier} '
                        f'(tilt={np.degrees(tilt):.0f} deg, '
                        f'azimuth={np.degrees(azim):.0f} deg) and fusing views')
                look_arm = self._reach_look_and_settle(
                    obj_xyz, name, tilt, azim, only_arm=arm)
                if look_arm is None:
                    if tier == 0:
                        return 'no arm could reach the nadir look pose'
                    self.get_logger().warn(
                        f'{name}: oblique tier {tier} pose unreachable -- '
                        'keeping the best candidate seen so far')
                    break
                arm = look_arm

                req = SampleGrasps.Request()
                req.target = str(idx) if isinstance(idx, int) else ''
                req.roi = 0.0
                req.captures = 0
                req.max_candidates = 0
                req.accumulate = tier > 0
                r = self._wait(self.sample_grasps_cli.call_async(req),
                               self.sample_grasps_timeout)
                if r is None:
                    self.get_logger().error(f'{name}: sample_grasps timed out')
                    if resp is None:
                        return 'sample_grasps timed out'
                    break
                if not r.success or not r.grasps.poses:
                    self.get_logger().error(
                        f'{name}: sample_grasps failed: {r.message}')
                    if resp is None:
                        return f'sample_grasps found no candidate ({r.message})'
                    break
                resp = r
                self.get_logger().info(
                    f'{arm.name}: tier {tier} top candidate '
                    f'score={resp.scores[0]:.3f} '
                    f'width={resp.widths[0] * 1000:.1f}mm '
                    f'antipodal={resp.antipodal[0]:.3f} '
                    f'gravity={resp.gravity[0]:.3f} '
                    f'both_sides_observed={resp.both_sides_observed[0]} '
                    f'({resp.message})')
                if resp.both_sides_observed[0]:
                    break

            gp = resp.grasps.poses[0]
            width = resp.widths[0]
            both_sides = resp.both_sides_observed[0]
            if not both_sides:
                self.get_logger().warn(
                    f'{arm.name}: top candidate STILL has an unobserved far '
                    f'contact side after {n_tiers} tier(s) -- proceeding on '
                    'the inferred far side as a DOCUMENTED risk, not a silent '
                    'skip')
            # Gen3 Lite pad gap calibration (isaac_sim/workcell/grasp_verify.txt):
            # 0.085 m open, 0.020 m closed.
            gripper_open_gap, gripper_closed_gap = 0.085, 0.020
            if width > gripper_open_gap:
                self.get_logger().error(
                    f'{arm.name}: candidate width {width * 1000:.1f}mm exceeds '
                    f"the gripper's open gap {gripper_open_gap * 1000:.0f}mm "
                    '-- cannot be grasped, aborting')
                return (f'candidate width {width * 1000:.1f}mm exceeds the '
                        f'gripper open gap')

            gq = gp.orientation
            R_wg = quat_to_R(gq.x, gq.y, gq.z, gq.w)
            grasp_xyz_gripper = np.array(
                [gp.position.x, gp.position.y, gp.position.z])
            # local +Z of the GRIPPER points at the fingertips (see the srv
            # definition) -- back off along -Z to retreat from the object.
            approach_dir = R_wg[:, 2]
            standoff_xyz_gripper = grasp_xyz_gripper - approach_dir * self.grasp_standoff

            standoff_xyz, standoff_quat = self._gripper_pose_to_tool(
                arm, standoff_xyz_gripper, R_wg)
            grasp_xyz, grasp_quat = self._gripper_pose_to_tool(
                arm, grasp_xyz_gripper, R_wg)
            if standoff_xyz is None or grasp_xyz is None:
                return 'no tool<-gripper TF; cannot convert the grasp pose'

            seed_q = self._current_q(arm)
            self.get_logger().info(
                f'{arm.name}: moving to stand-off {np.round(standoff_xyz, 3)} '
                f'({self.grasp_standoff:.3f} m back along the approach axis, '
                f'gripper-frame grasp point {np.round(grasp_xyz_gripper, 3)})')
            ok, goal_q, _ = self._move_to_pose(
                arm, standoff_xyz, standoff_quat, seed_q)
            if not ok:
                return 'stand-off unreachable'

            ok, _ = self._move_gripper(arm, self.gripper_open_pos)
            if not ok:
                self.get_logger().error(
                    f'{name}: gripper failed to open before descent')
                return 'gripper failed to open before descent'

            self.get_logger().info(
                f'{arm.name}: descending open-loop to grasp pose '
                f'{np.round(grasp_xyz, 3)} (no visual feedback below ~0.07 m '
                '-- D405 blind range)')
            ok, goal_q, _ = self._move_to_pose(
                arm, grasp_xyz, grasp_quat, goal_q)
            if not ok:
                self.get_logger().error(
                    f'{name}: open-loop descent failed to reach the grasp pose')
                return 'open-loop descent did not reach the grasp pose'

            self.get_logger().info(f'{arm.name}: closing gripper')
            ok, end_pos = self._move_gripper(arm, self.gripper_closed_pos)
            if not ok or end_pos is None:
                return 'gripper did not actuate/stall on anything'
            span_pos = self.gripper_open_pos - self.gripper_closed_pos
            frac = ((end_pos - self.gripper_closed_pos) / span_pos
                    if span_pos else 0.0)
            achieved_gap = (gripper_closed_gap
                           + frac * (gripper_open_gap - gripper_closed_gap))
            width_err = achieved_gap - width
            width_ok = abs(width_err) <= self.grasp_width_tol
            self.get_logger().info(
                f'{arm.name}: gripper closed at pos={end_pos:.3f} -> achieved '
                f'pad gap={achieved_gap * 1000:.1f}mm vs predicted '
                f'width={width * 1000:.1f}mm (err={width_err * 1000:+.1f}mm, '
                f'tol={self.grasp_width_tol * 1000:.0f}mm) -> '
                f'{"PASS" if width_ok else "FAIL"}')
            if self.grasp_settle_s > 0:
                time.sleep(self.grasp_settle_s)

            n_finger = self._finger_roi_count(arm, obj_xyz)
            visual_ok = n_finger > 0
            self.get_logger().info(
                f'{arm.name}: post-grasp wrist-cloud check -- {n_finger} '
                f'point(s) within 3cm of a finger link -> '
                f'{"PASS" if visual_ok else "FAIL"}')

            if not (width_ok and visual_ok):
                return (f'verification failed (width_check={width_ok}, '
                        f'visual_check={visual_ok}) -- not lifting')

            lift_xyz = grasp_xyz_gripper + np.array([0.0, 0.0, self.lift_height])
            lift_tool_xyz, lift_quat = self._gripper_pose_to_tool(
                arm, lift_xyz, R_wg)
            self.get_logger().info(f'{arm.name}: lifting {self.lift_height:.3f} m')
            ok, goal_q, _ = self._move_to_pose(
                arm, lift_tool_xyz, lift_quat, goal_q)
            if not ok:
                return 'grasp closed and verified, but the LIFT failed'

            self.get_logger().info(
                f'>>> {name}: GRASP SUCCESS -- {arm.name} closed on the '
                f'object (pad gap err={width_err * 1000:+.1f}mm) and lifted '
                f'{self.lift_height:.3f} m')
            return None
        finally:
            self._set_perception_pause(False)

    # ---- IK / planning ------------------------------------------------------
    def _grasp_ori_candidates(self):
        """Ordered list of grasp orientations (xyzw) to try, best-first: the
        vertical (grasp_orientation) approach with each yaw FIRST, then -- as a
        fallback for spots the ceiling arm can only reach at an angle -- the same
        yaws swung off vertical by grasp_tilt_samples magnitudes around
        grasp_tilt_azimuths compass directions. IK takes the first that solves, so
        a clean top-down grasp is always preferred and tilts only kick in when
        vertical is unreachable (-31)."""
        base = tuple(self.grasp_ori)
        yaws = max(1, self.grasp_yaw_samples)

        def with_yaws(ori):
            out = []
            for i in range(yaws):
                yaw = 2.0 * np.pi * i / yaws
                qz = (0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0))  # Rz(yaw)
                out.append(quat_mul(qz, ori))
            return out

        cands = with_yaws(base)                     # vertical first (preferred)
        if self.grasp_tilt_samples > 0 and self.grasp_tilt_max > 0:
            n_az = max(1, self.grasp_tilt_azimuths)
            for k in range(1, self.grasp_tilt_samples + 1):
                tilt = np.radians(self.grasp_tilt_max * k / self.grasp_tilt_samples)
                st, ct = np.sin(tilt / 2.0), np.cos(tilt / 2.0)
                for a in range(n_az):
                    phi = 2.0 * np.pi * a / n_az
                    # tilt the down-approach by `tilt` toward compass dir `phi`
                    # (rotation about the horizontal axis (cos phi, sin phi, 0)).
                    # One orientation per (tilt, azimuth) -- azimuth already gives
                    # the reach variety; no full yaw sweep here (keeps IK bounded).
                    q_tilt = (np.cos(phi) * st, np.sin(phi) * st, 0.0, ct)
                    cands.append(quat_mul(q_tilt, base))
        return cands

    def _solve_ik(self, arm, pose_stamped, seed_q, ori_list):
        """Try each grasp orientation in ori_list; return the first IK solution.
        Freeing the grasp yaw turns many -31 (NO_IK_SOLUTION) into a reachable
        target for the redundant arm+gantry chain."""
        if seed_q is None:
            # _current_q(arm) returns None until /joint_states has all of this
            # arm's joints -- e.g. right after a bringup restart where
            # ros2_control_node/robot_state_publisher haven't come up yet.
            # build_ik_request's `[float(v) for v in seed_pos]` would otherwise
            # crash hard on None ("'NoneType' object is not iterable") instead
            # of failing loud with a clear reason.
            self.get_logger().error(
                f'{arm.name}: no seed joint state available yet (arm not '
                'reporting on /joint_states) -- cannot solve IK')
            return False, None, None
        if not self.ik_cli.service_is_ready():
            self.ik_cli.wait_for_service(timeout_sec=2.0)
        last_err = None
        for ox, oy, oz, ow in ori_list:
            o = pose_stamped.pose.orientation
            o.x, o.y, o.z, o.w = ox, oy, oz, ow
            req = build_ik_request(arm.group, arm.ee_frame, pose_stamped,
                                   arm.joint_names, seed_q, self.ik_timeout,
                                   avoid_collisions=self.ik_avoid_collisions)
            res = self._wait(self.ik_cli.call_async(req), self.ik_wait)
            if res is None:
                continue
            if res.error_code.val == 1:
                return True, res.solution.joint_state, 1
            last_err = res.error_code.val
        return False, None, last_err

    @staticmethod
    def _extract(js, joint_names):
        """Pull `joint_names` positions out of an IK solution JointState."""
        pos = dict(zip(js.name, js.position))
        try:
            return np.array([pos[n] for n in joint_names])
        except KeyError:
            return None

    def _plan(self, arm, goal_q, do_execute):
        """MoveGroup to a joint-space goal; plan-only when do_execute is False,
        plan-AND-execute when True (the caller searches plan-only, then executes
        only the chosen candidate once).

        Returns (success, planned_trajectory, error_code, planning_time)."""
        if not self.move_cli.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('move_action server unavailable')
            return False, None, None, float('nan')
        req = MotionPlanRequest()
        req.group_name = arm.group
        con = Constraints()
        for n, v in zip(arm.joint_names, goal_q):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = self.joint_tol
            jc.tolerance_below = self.joint_tol
            jc.weight = 1.0
            con.joint_constraints.append(jc)
        req.goal_constraints = [con]
        req.num_planning_attempts = self.plan_attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = not do_execute

        gh = self._wait(self.move_cli.send_goal_async(goal), self.plan_wait)
        if gh is None or not gh.accepted:
            return False, None, None, float('nan')
        # plan-only returns as soon as planning finishes; execute only completes
        # once the trajectory has physically moved -> allow exec_wait for that.
        result_wait = self.plan_wait + self.plan_time + (
            self.exec_wait if do_execute else 5.0)
        rr = self._wait(gh.get_result_async(), result_wait)
        if rr is None:
            return False, None, None, float('nan')
        result = rr.result
        ok = result.error_code.val == 1
        return ok, result.planned_trajectory, result.error_code.val, \
            float(result.planning_time)

    # ---- optional trajectory energy (lazy Pinocchio) ------------------------
    def _pin_model(self, arm):
        if arm.name in self._pin_cache:
            return self._pin_cache[arm.name]
        try:
            cfg_path = self.pin_configs[self.arms.index(arm)]
        except (IndexError, ValueError):
            cfg_path = ''
        if not cfg_path:
            self._pin_cache[arm.name] = None
            return None
        import yaml
        from reachability_gng.eval import build_model
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        pin, model, data, ee_id, order, lo, hi = build_model(cfg)
        self._pin_cache[arm.name] = (pin, model, data, order)
        return self._pin_cache[arm.name]

    def _traj_energy(self, arm, traj):
        """Mechanical energy E = sum |tau . qdot| dt over the planned trajectory
        via inverse dynamics (Pinocchio). Returns NaN if no pin config / failure."""
        try:
            pm = self._pin_model(arm)
            if pm is None or traj is None:
                return float('nan')
            pin, model, data, order = pm
            jt = traj.joint_trajectory
            col = [jt.joint_names.index(n) for n in order]
            pts = jt.points
            E = 0.0
            for k in range(1, len(pts)):
                a, b = pts[k - 1], pts[k]
                dt = (b.time_from_start.sec + b.time_from_start.nanosec * 1e-9) \
                    - (a.time_from_start.sec + a.time_from_start.nanosec * 1e-9)
                if dt <= 0:
                    continue
                q = np.array([b.positions[c] for c in col])
                v = np.array([b.velocities[c] for c in col]) if b.velocities \
                    else np.zeros(len(order))
                acc = np.array([b.accelerations[c] for c in col]) \
                    if b.accelerations else np.zeros(len(order))
                tau = pin.rnea(model, data, q, v, acc)
                E += abs(float(tau @ v)) * dt
            return E
        except Exception as e:                       # energy is optional
            self.get_logger().warn(f'traj energy failed: {e}')
            return float('nan')

    # ---- grasp collision allowance ------------------------------------------
    def _allow_target_collision(self, target):
        """Add the grasp target's own CollisionObject to the ACM default-allow so
        IK + the plan can reach INTO the object being grasped. The target is the
        scene collision object nearest `target` (within grasp_match_radius);
        everything else (table, octomap, other objects) stays collision-checked.
        Applied once per object id; no-op if no box is at the target yet."""
        if not self.get_scene_cli.service_is_ready():
            self.get_scene_cli.wait_for_service(timeout_sec=2.0)
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        res = self._wait(self.get_scene_cli.call_async(req), self.plan_wait)
        if res is None:
            self.get_logger().warn('allow_target_collision: planning-scene fetch '
                                   'timed out; grasp IK may collide with target')
            return None
        tp = np.array([target.position.x, target.position.y, target.position.z])
        best, bestd = None, self.grasp_match_radius
        for co in res.scene.world.collision_objects:
            # MoveIt may return the centre in co.pose with primitives relative to
            # it, OR co.pose at origin with the centre in primitive_poses -- match
            # against both so we find the box regardless of convention.
            positions = [co.pose.position] + [pp.position for pp in co.primitive_poses]
            for c in positions:
                d = float(np.linalg.norm(np.array([c.x, c.y, c.z]) - tp))
                if d < bestd:
                    best, bestd = co.id, d
        if best is None:
            self.get_logger().warn(
                f'allow_target_collision: no collision object within '
                f'{self.grasp_match_radius:.2f} m of the grasp goal '
                f'(scene has {len(res.scene.world.collision_objects)} objects)')
            return None
        if best in self._acm_allowed:
            return best
        acm = res.scene.allowed_collision_matrix
        acm.default_entry_names.append(best)
        acm.default_entry_values.append(True)
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        areq = ApplyPlanningScene.Request()
        areq.scene = scene
        if self._wait(self.apply_scene_cli.call_async(areq), self.plan_wait) \
                is not None:
            self._acm_allowed.add(best)
            self.get_logger().info(
                f"grasp target '{best}' ACM-allowed to touch the robot "
                f"(IK/plan may reach into it; {bestd:.3f} m from goal)")
        return best

    # ---- grasp + logging ----------------------------------------------------
    def _set_perception_pause(self, paused):
        """Freeze/resume the sensed scene for the pick (Fix A). On pause, settle so
        the last in-flight octomap/CollisionObject update lands before planning."""
        if not self.gate_perception:
            return
        self._pick_active = bool(paused)
        msg = Bool()
        msg.data = bool(paused)
        self.pause_pub.publish(msg)
        if paused:
            time.sleep(self.gate_settle)
        self.get_logger().info(
            f'perception {"paused" if paused else "resumed"} for pick')

    def _pause_heartbeat(self):
        if self._pick_active and self.gate_perception:
            msg = Bool()
            msg.data = True
            self.pause_pub.publish(msg)

    def _attach(self, arm, object_id):
        msg = String()
        msg.data = f'attach {object_id} {arm.gripper_link}'
        self.attach_pub.publish(msg)
        self.get_logger().info(f'sent: {msg.data}')

    def _detach(self, object_id):
        msg = String()
        msg.data = f'detach {object_id}'
        self.attach_pub.publish(msg)
        self.get_logger().info(f'sent: {msg.data}')

    def _move_gripper(self, arm, position, settle_timeout=90.0, settle_tol=0.02,
                      poll_dt=0.2, stall_consec=5, min_settle_before_stall=30.0):
        """Send `position` (rad, master finger joint) to arm's gripper, then
        confirm against the LIVE joint state rather than trusting the action
        result alone -- GripperActionController's `reached_goal`/`stalled`
        can report before Isaac's physics finishes the move, the same race
        already found for arm motion (_wait_until_reached) and ~/look (see
        memory look-settle-race-and-cube-occlusion): a live grasp attempt
        got `stalled=True, moved~=0` immediately after commanding OPEN, while
        the live joint had actually reached ~0.96 (fully open) moments later.

        `min_settle_before_stall` guards against a second, distinct failure
        mode seen live: under heavy system load (observed load average 53 on
        this shared server) Isaac can have several seconds of dead time
        before a commanded joint even STARTS moving -- the naive stall check
        misread "hasn't started yet" as "already stalled" and gave up after
        <1.5s while the close command was still going to complete correctly
        given enough time. No stall exit is allowed before this many seconds
        have elapsed, regardless of stable-reading count.

        True if the LIVE joint converges to `position` OR stalls (after this
        grace period) having moved (an object between the fingers stops it
        short of `position` -- that IS the grasp, not a failure), mirroring
        run_take_bottle.py's move_gripper confirmation heuristic. False on a
        genuine no-motion stall (nothing between the fingers / hardware not
        actuating)."""
        client = self.gripper_clients.get(arm.name)
        if client is None:
            self.get_logger().error(f'{arm.name}: no gripper client (gripper_name '
                                    'missing for this arm)')
            return False, None
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{arm.name}: gripper action server not ready')
            return False, None
        start_pos = self._joints.get(arm.gripper_joint)
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_max_effort
        gh = self._wait(client.send_goal_async(goal), 15.0)
        if gh is None or not gh.accepted:
            self.get_logger().error(f'{arm.name}: gripper goal rejected')
            return False, None
        result = self._wait(gh.get_result_async(), 15.0)
        if result is None:
            self.get_logger().error(f'{arm.name}: gripper action timed out')
            return False, None

        t0 = time.time()
        deadline = t0 + settle_timeout
        last, stable = self._joints.get(arm.gripper_joint), 0
        while time.time() < deadline:
            cur = self._joints.get(arm.gripper_joint)
            if cur is not None:
                if abs(cur - position) <= settle_tol:
                    break
                if last is not None and abs(cur - last) <= settle_tol * 0.25:
                    stable += 1
                    if (stable >= stall_consec
                            and time.time() - t0 >= min_settle_before_stall):
                        break
                else:
                    stable = 0
                last = cur
            time.sleep(poll_dt)
        end_pos = self._joints.get(arm.gripper_joint)
        if end_pos is None:
            self.get_logger().error(
                f'{arm.name}: no live gripper joint state available to '
                'confirm the move')
            return False, None
        if abs(end_pos - position) <= settle_tol:
            return True, end_pos
        moved = start_pos is not None and abs(end_pos - start_pos) > 0.02
        if moved:
            self.get_logger().info(
                f'{arm.name}: gripper stalled on object (pos={end_pos:.3f}) '
                '-- treating as grasped')
            return True, end_pos
        self.get_logger().error(
            f'{arm.name}: gripper did not actuate (start={start_pos}, '
            f'end={end_pos:.4f}, target={position:.4f})')
        return False, end_pos

    def _move_to_pose(self, arm, xyz, quat, seed_q):
        """IK (seeded, single orientation -- a small local correction, not a
        fresh search) -> plan+execute -> wait for the live arm to converge.
        Returns (ok, goal_q, err)."""
        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2]))
        ok, js, ikerr = self._solve_ik(arm, ps, seed_q, [tuple(quat)])
        if not ok:
            self.get_logger().error(f'{arm.name}: no IK to {np.round(xyz, 3)} '
                                    f'(err={ikerr})')
            return False, None, float('inf')
        goal_q = self._extract(js, arm.joint_names)
        if goal_q is None:
            return False, None, float('inf')
        ex_ok, _, eperr, _ = self._plan(arm, goal_q, do_execute=True)
        if not ex_ok:
            self.get_logger().error(f'{arm.name}: plan/execute to '
                                    f'{np.round(xyz, 3)} failed (err={eperr})')
            return False, goal_q, float('inf')
        reached, err = self._wait_until_reached(
            arm, goal_q, self.reach_tol, self.exec_wait)
        if not reached:
            self.get_logger().error(f'{arm.name}: did not converge to '
                                    f'{np.round(xyz, 3)} (joint err={err:.3f})')
        return reached, goal_q, err

    def _grasp_sequence(self, arm, pregrasp_xyz, quat, seed_q, object_id):
        """Full cycle from the reached pre-grasp pose: descend, close, attach,
        lift, optional transport+place, retreat. Returns True only if every
        stage that was attempted succeeded; logs plainly which stage failed
        otherwise (Rule 12 -- no silent partial "success").

        `object_id` may be '' (no scene CollisionObject to attach/detach --
        still runs the physical grasp, just without MoveIt attach bookkeeping).
        """
        px, py, pz = pregrasp_xyz
        grasp_xyz = (px, py, pz - self.grasp_descend)
        self.get_logger().info(f'{arm.name}: descending {self.grasp_descend:.3f} m '
                               'to grasp height')
        ok, goal_q, _ = self._move_to_pose(arm, grasp_xyz, quat, seed_q)
        if not ok:
            return False

        self.get_logger().info(f'{arm.name}: closing gripper')
        ok, _ = self._move_gripper(arm, self.gripper_closed_pos)
        if not ok:
            return False
        if self.grasp_settle_s > 0:
            time.sleep(self.grasp_settle_s)
        if object_id:
            self._attach(arm, object_id)

        lift_xyz = (px, py, pz + self.lift_height)
        self.get_logger().info(f'{arm.name}: lifting {self.lift_height:.3f} m')
        ok, goal_q, _ = self._move_to_pose(arm, lift_xyz, quat, goal_q)
        if not ok:
            return False

        if self.place_enabled:
            place_above = (self.place_xyz[0], self.place_xyz[1],
                           self.place_xyz[2] + self.lift_height)
            self.get_logger().info(f'{arm.name}: transporting to '
                                   f'{np.round(self.place_xyz, 3)}')
            ok, goal_q, _ = self._move_to_pose(arm, place_above, quat, goal_q)
            if not ok:
                return False
            ok, goal_q, _ = self._move_to_pose(arm, tuple(self.place_xyz), quat, goal_q)
            if not ok:
                return False
            self.get_logger().info(f'{arm.name}: opening gripper (place)')
            ok, _ = self._move_gripper(arm, self.gripper_open_pos)
            if not ok:
                return False
            if object_id:
                self._detach(object_id)
            ok, goal_q, _ = self._move_to_pose(arm, place_above, quat, goal_q)
            if not ok:
                return False
        return True

    def _init_csv(self):
        with open(self.csv_log, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['t', 'obj', 'arm', 'attempt', 'rank_J', 'rank_dist', 'node',
                 'dist', 'ee_dist', 'd_gantry_lin', 'd_gantry_rot', 'd_arm',
                 'hold', 'manip', 'J', 'gantry_lin', 'gantry_rot', 'ik_ms',
                 'plan_time_s', 'plan_ms', 'traj_energy', 'success'])

    def _log_csv(self, obj, attempt, rank_dist, c, goal_q, ik_ms, ptime,
                 plan_ms, energy):
        if not self.csv_log:
            return
        if c is None:
            row = [time.time(), obj, '', -1, -1, -1, -1, '', '', '', '', '',
                   '', '', '', '', '', ik_ms, ptime, plan_ms, energy, 0]
        else:
            row = [time.time(), obj, c['arm'].name, attempt, attempt, rank_dist,
                   c['node'], c['dist'], c['ee_dist'], c['d_gantry_lin'],
                   c['d_gantry_rot'], c['d_arm'], c['hold'], c['manip'], c['J'],
                   float(goal_q[0]), float(goal_q[1]),
                   ik_ms, ptime, plan_ms, energy, 1]
        with open(self.csv_log, 'a', newline='') as f:
            csv.writer(f).writerow(row)


def main():
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = GantryReachExecutor()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
