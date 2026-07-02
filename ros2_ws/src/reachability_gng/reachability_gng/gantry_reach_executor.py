"""Energy-aware redundancy resolution + arm selection for the gantry workcell.

Given a detected object, this node decides WHICH arm (arm_1 or arm_2, both
riding the shared gantry_1) reaches it and at WHICH 8-DOF configuration -- by
ENERGY, not by nearest seed -- then hands MoveIt a joint-space goal so MoveIt
does the actual collision-aware planning (and optionally execution) against the
live octomap.

Pipeline per pick (`~/pick`):
  1. Take the target object pose from /detected_objects.
  2. From EACH arm's GNG map, gather the POOL of nodes within `pool_radius`
     (task-space) of the object -- the reachability filter (hard).
  3. Score every pooled candidate by
        J = w_gantry_lin*d_gantry_lin + w_gantry_rot*d_gantry_rot
            + w_arm*d_arm + w_dist*ee_dist - w_manip*manip
     where d_* are joint travel from the current state. The gantry's linear
     (prismatic, metres) and rotation (radians) axes are weighted separately
     because their units and cost differ; gantry travel is the dominant,
     expensive term. `ee_dist` is the arm's CURRENT tool-frame distance (TF,
     metres) to the object -- one value per arm, so this term biases the
     allocation toward whichever arm's end-effector is already nearer. (The
     per-node task-space `dist` still gates the pool and is logged for the
     rank-by-distance diagnostic; the gravity `hold` cost is CSV-only.) `manip`
     is the node manipulability. J is the OBJECTIVE; arm selection emerges from
     J.
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
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.gng import GNG
from reachability_gng.pause_gate import PAUSE_TOPIC, latched_qos
from reachability_gng.seed_ik import build_ik_request


def quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions -> (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


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
        # Pre-grasp: aim this many metres ABOVE the object centroid (world +Z) so
        # the gripper stops over the object instead of plunging to the centroid and
        # hitting the table/environment octomap (goal-pose collision -> -2).
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('world_frame', 'world')
        # pool_radius <= 0 -> density-adaptive (pool_radius_factor * node
        # spacing), so the pool size is independent of the GNG `lam`; > 0 ->
        # absolute task-space metres (mirrors reachability_check's convention).
        self.declare_parameter('pool_radius', 0.0)
        self.declare_parameter('pool_radius_factor', 2.5)
        self.declare_parameter('max_candidates', 20)
        self.declare_parameter('n_gantry_dofs', 2)
        # --- energy weights (J) --- single source of truth for the calibrated
        # defaults; the launch no longer overrides these. Calibrated so each
        # term's influence (weight x its pool spread) matches the intended
        # balance: gantry dominant (heavy platform), arm minor (cheap joints),
        # manip x30 because its spread (~0.08) is tiny.
        # Override live with -p w_*:=...
        # The gantry's two DOFs are weighted SEPARATELY because their units and
        # cost differ: w_gantry_lin scores the linear/prismatic axis (metres of
        # heavy-carriage travel), w_gantry_rot the rotation axis (radians).
        self.declare_parameter('w_gantry_lin', 1.0)
        self.declare_parameter('w_gantry_rot', 1.0)
        self.declare_parameter('w_arm', 2.0)        #1.2
        self.declare_parameter('w_manip', 30.0)      #30
        # w_dist scores the task-space gap (metres) between the candidate node
        # and the object: distance already gates the pool, this also makes a
        # closer seed cheaper inside J (0 = distance only gates, does not rank).
        self.declare_parameter('w_dist', 10.0)
        # Print the full ranked J table (every pooled candidate, ascending J,
        # with each term's weighted contribution) to the terminal on each pick.
        self.declare_parameter('log_j_table', True)
        self.declare_parameter('log_j_table_max', 20)   # rows to print (top-N)
        # --- IK / planning ---
        self.declare_parameter('ik_timeout', 0.05)        # inside the IK request
        self.declare_parameter('ik_wait', 2.0)            # service round-trip cap
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
        # Fix A: freeze the sensed scene (octomap + object CollisionObjects) for
        # the plan/execute window so a mid-plan scene-version bump doesn't drop the
        # plan (-3/-2). gate_settle lets the last in-flight update land first.
        self.declare_parameter('gate_perception', True)
        self.declare_parameter('gate_settle', 0.5)
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
        self.approach_offset = float(g('approach_offset').value)
        self.world_frame = g('world_frame').value
        self.pool_radius = float(g('pool_radius').value)
        self.pool_radius_factor = float(g('pool_radius_factor').value)
        self.max_candidates = int(g('max_candidates').value)
        self.n_gantry = int(g('n_gantry_dofs').value)
        self.w_gantry_lin = float(g('w_gantry_lin').value)
        self.w_gantry_rot = float(g('w_gantry_rot').value)
        self.w_arm = float(g('w_arm').value)
        self.w_manip = float(g('w_manip').value)
        self.w_dist = float(g('w_dist').value)
        self.log_j_table = bool(g('log_j_table').value)
        self.log_j_table_max = int(g('log_j_table_max').value)
        self.ik_timeout = float(g('ik_timeout').value)
        self.ik_wait = float(g('ik_wait').value)
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
        self.gate_perception = bool(g('gate_perception').value)
        self.gate_settle = float(g('gate_settle').value)
        self.auto_attach = bool(g('auto_attach').value)
        self.attach_object_id = g('attach_object_id').value
        self.allow_target_collision = bool(g('allow_target_collision').value)
        self.grasp_match_radius = float(g('grasp_match_radius').value)
        self.compute_traj_energy = bool(g('compute_traj_energy').value)
        self.csv_log = g('csv_log').value

        self.arms = []
        for nm, mp, grp, ee, gl in zip(names, models, groups, ees, grips):
            try:
                arm = ArmModel(nm, mp, grp, ee, gl)
                arm.eff_radius = (self.pool_radius if self.pool_radius > 0.0
                                  else self.pool_radius_factor * arm.spacing)
                self.arms.append(arm)
                self.get_logger().info(
                    f'loaded {nm}: {mp} -> group {grp}, ee {ee} '
                    f'(spacing={arm.spacing:.3f} m, pool_radius='
                    f'{arm.eff_radius:.3f} m)')
            except (OSError, KeyError) as e:
                self.get_logger().error(f'could not load {nm} ({mp}): {e}')
        if not self.arms:
            raise SystemExit('no arm models loaded; nothing to do')

        self._pin_cache = {}            # arm name -> (pin, model, data, order)
        self._latest_objects = None
        self._latest_target = None      # /target_object, the single grasp target
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
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joints, 10, callback_group=cb)
        self.create_subscription(String, '~/pick', self._on_pick, 1,
                                 callback_group=cb)
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
        self.move_cli = ActionClient(self, MoveGroup, 'move_action',
                                     callback_group=cb)

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

    def _on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self._joints[n] = p

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

    # ---- helpers ------------------------------------------------------------
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

    def _log_j_table(self, idx, by_J, dist_rank):
        """Print the full ranked J table -- every pooled candidate in ascending
        J, with each term's weighted contribution (weight*value), so the J
        calculation is fully visible in the terminal. `*` marks the winner."""
        rows = ['', f'>>> obj{idx}: J ranking ({len(by_J)} candidates, '
                    f'weights glin={self.w_gantry_lin:g} grot={self.w_gantry_rot:g} '
                    f'arm={self.w_arm:g} dist={self.w_dist:g} '
                    f'manip={self.w_manip:g})',
                '  rank arm     node      J | glin  grot   arm eedist  manip '
                '(rank_dist)']
        for r, c in enumerate(by_J[:self.log_j_table_max]):
            glin = self.w_gantry_lin * c['d_gantry_lin']
            grot = self.w_gantry_rot * c['d_gantry_rot']
            arm = self.w_arm * c['d_arm']
            eedist = self.w_dist * c['ee_dist']
            manip = -self.w_manip * c['manip']
            mark = '*' if r == 0 else ' '
            rows.append(
                f'{mark} {r:>3} {c["arm"].name:<7} {c["node"]:>4} '
                f'{c["J"]:>7.3f} | {glin:>5.2f} {grot:>5.2f} {arm:>5.2f} '
                f'{eedist:>5.2f} {manip:>6.2f}  '
                f'({dist_rank[id(c)]})')
        if len(by_J) > self.log_j_table_max:
            rows.append(f'  ... {len(by_J) - self.log_j_table_max} more '
                        f'(raise log_j_table_max to see all)')
        self.get_logger().info('\n'.join(rows))

    # ---- main pick ----------------------------------------------------------
    def _do_pick(self, arg):
        # Resolve the grasp target. Prefer the explicit index the caller sent
        # against the LIVE /detected_objects -- that is exactly what pick_cli
        # listed and the user just selected, so each pick hits the object chosen.
        # Only fall back to the /target_object topic when no usable index was
        # given (e.g. a bare `ros2 topic pub` in target_label mode): that topic
        # is refreshed on object_localizer's timer and lags a fresh selection, so
        # preferring it would re-pick the PREVIOUS object (the bug this fixes).
        objs = self._latest_objects
        target, idx = None, None
        if arg.isdigit() and objs is not None and objs.poses:
            i = int(arg)
            if 0 <= i < len(objs.poses):
                target, idx = objs.poses[i], i
            else:
                self.get_logger().warn(
                    f'object index {i} out of range (have {len(objs.poses)})')
                return
        if target is None:
            if self._latest_target is not None:
                target, idx = self._latest_target.pose, 'target'
            else:
                self.get_logger().warn(
                    f'pick arg "{arg}": no matching /detected_objects index and '
                    f'no /target_object yet')
                return
        p = target.position
        self.get_logger().info(
            f'>>> NEW TARGET obj{idx}: pick requested at '
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
                J = (self.w_gantry_lin * d_gantry_lin
                     + self.w_gantry_rot * d_gantry_rot
                     + self.w_arm * d_arm
                     + self.w_dist * ee_dist
                     - self.w_manip * manip)
                cands.append(dict(arm=arm, node=node_idx, q=q, dist=dist,
                                  ee_dist=ee_dist, manip=manip, hold=hold,
                                  d_gantry_lin=d_gantry_lin,
                                  d_gantry_rot=d_gantry_rot,
                                  d_arm=d_arm, J=J))
        if not cands:
            self.get_logger().error(
                f'>>> obj{idx}: FAILED -- no reachable arm candidates in pool '
                f'(object out of every arm\'s GNG reach map)')
            return

        by_J = sorted(cands, key=lambda c: c['J'])
        # rank-by-distance, to later show energy may pick a non-nearest seed
        dist_rank = {id(c): r for r, c in
                     enumerate(sorted(cands, key=lambda c: c['dist']))}

        if self.log_j_table:
            self._log_j_table(idx, by_J, dist_rank)

        # Announce the arm the energy allocation chose (lowest-J candidate). This
        # is the arm that WILL do the task; if its plan fails the executor falls
        # through to the next-best candidate (possibly the other arm) below.
        best = by_J[0]
        by_arm_bestJ = {}
        for c in by_J:
            by_arm_bestJ.setdefault(c['arm'].name, c['J'])
        summary = ', '.join(f'{n} J={j:.3f}' for n, j in by_arm_bestJ.items())
        self.get_logger().info(
            f'>>> obj{idx}: allocation -> {best["arm"].name} will do the task '
            f'(best J={best["J"]:.3f} [gantry_lin={best["d_gantry_lin"]:.3f} '
            f'gantry_rot={best["d_gantry_rot"]:.3f} arm={best["d_arm"]:.3f} '
            f'eedist={best["ee_dist"]:.3f} '
            f'manip={best["manip"]:.3f}]; per-arm best: {summary})')

        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        # pre-grasp point: above the object centroid (don't share target.position)
        ps.pose.position.x = target.position.x
        ps.pose.position.y = target.position.y
        ps.pose.position.z = target.position.z + self.approach_offset
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
            # 2) try candidates in ascending-J order
            for attempt, c in enumerate(by_J[:self.max_attempts]):
                arm = c['arm']
                t0 = time.perf_counter()
                ok, js, ikerr = self._solve_ik(arm, ps, c['q'], ori_list)
                ik_ms = (time.perf_counter() - t0) * 1e3
                if not ok:
                    self.get_logger().info(
                        f'obj{idx} cand#{attempt} {arm.name} J={c["J"]:.3f}: '
                        f'IK failed (err={ikerr})')
                    continue
                goal_q = self._extract(js, arm.joint_names)
                if goal_q is None:
                    continue
                t1 = time.perf_counter()
                planned, traj, perr, ptime = self._plan(arm, goal_q)
                plan_ms = (time.perf_counter() - t1) * 1e3
                if not planned:
                    self.get_logger().info(
                        f'obj{idx} cand#{attempt} {arm.name} J={c["J"]:.3f}: '
                        f'plan failed (err={perr})')
                    continue

                energy = (self._traj_energy(arm, traj)
                          if self.compute_traj_energy else float('nan'))
                self.get_logger().info(
                    f'obj{idx}: PICKED {arm.name} via cand#{attempt} '
                    f'(rank-by-dist {dist_rank[id(c)]}) J={c["J"]:.3f} '
                    f'[gantry_lin={c["d_gantry_lin"]:.3f} '
                    f'gantry_rot={c["d_gantry_rot"]:.3f} arm={c["d_arm"]:.3f} '
                    f'eedist={c["ee_dist"]:.3f} '
                    f'manip={c["manip"]:.3f}] '
                    f'gantry_goal=({goal_q[0]:.3f},{goal_q[1]:.3f}) '
                    f'ik={ik_ms:.0f}ms plan={plan_ms:.0f}ms '
                    f'{"executed" if self.execute else "plan-only"}')
                if self.execute:
                    # Confirm against the LIVE arm: move_group's result can beat
                    # Isaac Sim's physics, so wait for /joint_states to converge.
                    reached, err = self._wait_until_reached(
                        arm, goal_q, self.reach_tol, self.exec_wait)
                    if reached:
                        if self.auto_attach and self.attach_object_id:
                            self._attach(arm, self.attach_object_id)
                        self.get_logger().info(
                            f'>>> obj{idx}: SUCCESS -- {arm.name} reached the '
                            f'target (max joint err={err:.3f} <= {self.reach_tol})')
                    else:
                        self.get_logger().error(
                            f'>>> obj{idx}: {arm.name} motion did NOT settle at '
                            f'the goal within {self.exec_wait:.0f}s '
                            f'(max joint err={err:.3f} > {self.reach_tol})')
                self._log_csv(idx, attempt, dist_rank[id(c)], c, goal_q,
                              ik_ms, ptime, plan_ms, energy)
                return

            n_tried = min(len(by_J), self.max_attempts)
            self.get_logger().error(
                f'>>> obj{idx}: FAILED -- no arm could reach the target '
                f'(no collision-free plan over {n_tried} candidates)')
            self._log_csv(idx, -1, -1, None, None, float('nan'),
                          float('nan'), float('nan'), float('nan'))
        finally:
            self._set_perception_pause(False)

    # ---- IK / planning ------------------------------------------------------
    def _grasp_ori_candidates(self):
        """grasp_orientation plus yaw rotations about world Z (list of xyzw)."""
        base = tuple(self.grasp_ori)
        n = max(1, self.grasp_yaw_samples)
        cands = []
        for i in range(n):
            yaw = 2.0 * np.pi * i / n
            qz = (0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0))  # Rz(yaw) xyzw
            cands.append(quat_mul(qz, base))
        return cands

    def _solve_ik(self, arm, pose_stamped, seed_q, ori_list):
        """Try each grasp orientation in ori_list; return the first IK solution.
        Freeing the grasp yaw turns many -31 (NO_IK_SOLUTION) into a reachable
        target for the redundant arm+gantry chain."""
        if not self.ik_cli.service_is_ready():
            self.ik_cli.wait_for_service(timeout_sec=2.0)
        last_err = None
        for ox, oy, oz, ow in ori_list:
            o = pose_stamped.pose.orientation
            o.x, o.y, o.z, o.w = ox, oy, oz, ow
            req = build_ik_request(arm.group, arm.ee_frame, pose_stamped,
                                   arm.joint_names, seed_q, self.ik_timeout,
                                   avoid_collisions=True)
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

    def _plan(self, arm, goal_q):
        """MoveGroup to a joint-space goal; plan_only unless self.execute.

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
        goal.planning_options.plan_only = not self.execute

        gh = self._wait(self.move_cli.send_goal_async(goal), self.plan_wait)
        if gh is None or not gh.accepted:
            return False, None, None, float('nan')
        # plan-only returns as soon as planning finishes; execute only completes
        # once the trajectory has physically moved -> allow exec_wait for that.
        result_wait = self.plan_wait + self.plan_time + (
            self.exec_wait if self.execute else 5.0)
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
