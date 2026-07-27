"""Simultaneous 4-arm / 2-gantry co-manipulation: lift ONE rigid object with all
four arms gripping its top-face corners at once.

This is DELIBERATELY SEPARATE from gantry_reach_executor (single-winner: picks
the ONE cheapest arm for an object). A shared rigid object needs the opposite
discipline -- every stage below is a BARRIER: all participants are commanded
concurrently and the executor waits for ALL of them before the next stage, so
the object is never held by fewer arms than the plan calls for.

Two arms on ONE gantry share that gantry's rail/rotation joints, so they cannot
be driven by two independent MoveGroup goals at once (both would fight over the
same actuator). The sequence is therefore:

  1. decide      -- offline, numpy only: which gantry pose (rail, theta) x2 and
                     which arm takes which corner. Nearest-FK-sample search over
                     the SAME per-arm reach maps comanip_map.py/comanip_env.py
                     use (gantry-local frame), not a live service call.
  2. gantries    -- move gantry_1 AND gantry_2 to their chosen poses, CONCURRENT
                     (they share no joints with each other).
  3. approach x4 -- each arm (arm-ONLY group, gantry now fixed) to its corner's
                     pre-grasp pose, CONCURRENT, barrier.
  4. descend x4  -- lower each EE onto the corner, CONCURRENT, barrier.
  5. grip x4     -- close each gripper, CONCURRENT, barrier. Any arm that does
                     NOT confirm a grasp aborts the lift (Rule 12: a partial
                     grip must not attempt to lift -- that is how the object
                     gets dropped or knocked over).
  6. lift x4     -- raise every arm by lift_height, CONCURRENT, barrier.

No transport/place in this version -- the object is held at lift height above
its pick point. (Minimal-scope decision, 2026-07-27: coordinated TRANSPORT
needs both gantries translating together while the object stays rigid between
the arms, which is materially harder and out of scope for this pass.)

Prerequisites: move_group (my_workcell.launch.py) + this workcell's 4 GNG maps
(data/maps/arm{1..4}_model.npz, arm{1..4}_dataset.npz).

    ros2 launch reachability_gng gantry_pick_coalition.launch.py execute:=true
    ros2 topic pub --once /coalition_reach_executor/pick std_msgs/String \\
        "{data: '0.45,0.60,1.23,0.0'}"   # x,y,z,yaw of the box CENTRE (world)
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Point
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from reachability_gng.gantry_reach_executor import ArmModel, quat_mul
from reachability_gng.seed_ik import build_ik_request

# Mirrors scripts/comanip_map.py's gantry-local geometry (KEEP IN SYNC if the
# URDF mounts ever move): gantry base in world, the -Z rotation-joint origin
# offset, which two arms ride which gantry, and each arm's shoulder mount in
# the gantry-local frame (for logging only here, unlike comanip_map.py this
# node does not need the arm-crossing proxy -- shown non-binding, see
# arm-crossing-cannot-bind-pi-symmetry memory).
GANTRY_BASE = {0: np.array([0.0, 0.36, 2.05]), 1: np.array([0.0, -0.36, 2.05])}
Z_OFFSET = np.array([0.0, 0.0, -0.04])
GANTRY_ARMS = {0: ['arm_1', 'arm_2'], 1: ['arm_3', 'arm_4']}
GANTRY_GROUPS = {0: 'gantry_1', 1: 'gantry_2'}
GANTRY_JOINTS = {0: ('t1_linear_joint', 't1_rotation_joint'),
                 1: ('t2_linear_joint', 't2_rotation_joint')}
PARTITIONS = [((0, 1), (2, 3)), ((1, 2), (3, 0))]   # opposite top-face edges


def to_local(p_world, base, rail, theta):
    v = p_world - base - np.array([rail, 0.0, 0.0]) - Z_OFFSET
    ang = -(np.pi / 2.0 + theta)
    c, s = np.cos(ang), np.sin(ang)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def handle_corners(centre, yaw, size):
    """4 world corners of a `size` square, TOP face (size/2 above centre),
    ordered around the face -- same convention as comanip_map.handle_corners."""
    h = size / 2.0
    c, s = np.cos(yaw), np.sin(yaw)
    out = []
    for sx, sy in ((+1, +1), (-1, +1), (-1, -1), (+1, -1)):
        dx, dy = sx * h, sy * h
        out.append(centre + np.array([c * dx - s * dy, s * dx + c * dy, h]))
    return np.stack(out)


class CoalitionReachExecutor(Node):
    def __init__(self):
        super().__init__('coalition_reach_executor')
        p = self.declare_parameter
        p('arm_names', ['arm_1', 'arm_2', 'arm_3', 'arm_4'])
        p('arm_models', ['/srv/data/users/raditya/arm_WS/ceiling_arm/data/maps/arm1_model.npz',
                         '/srv/data/users/raditya/arm_WS/ceiling_arm/data/maps/arm2_model.npz',
                         '/srv/data/users/raditya/arm_WS/ceiling_arm/data/maps/arm3_model.npz',
                         '/srv/data/users/raditya/arm_WS/ceiling_arm/data/maps/arm4_model.npz'])
        # Arm-ONLY groups (6 joints, no gantry) -- the gantry is fixed by stage 2
        # before any arm moves, so arm_i/arm_j on the SAME gantry never race.
        p('arm_groups', ['arm_1', 'arm_2', 'arm_3', 'arm_4'])
        p('arm_ee_frames', ['t1_a1_tool_frame', 't1_a2_tool_frame',
                            't2_a1_tool_frame', 't2_a2_tool_frame'])
        p('gripper_links', ['t1_a1_gripper_base_link', 't1_a2_gripper_base_link',
                            't2_a1_gripper_base_link', 't2_a2_gripper_base_link'])
        p('gripper_names', ['gripper_1', 'gripper_2', 'gripper_3', 'gripper_4'])
        p('gripper_joints', ['t1_a1_right_finger_bottom_joint',
                             't1_a2_right_finger_bottom_joint',
                             't2_a1_right_finger_bottom_joint',
                             't2_a2_right_finger_bottom_joint'])
        p('world_frame', 'world')
        p('box_size', 0.25)                    # m, top-face edge
        p('reach_radius', 0.05)                # m, FK-sample match tolerance
        p('rail_step', 0.1); p('rot_step', 0.2)  # decision-sweep resolution
        # Isaac-verified (see gantry_reach_executor.py's do_grasp params for the
        # real-hardware-convention conflict this does NOT re-litigate here).
        p('gripper_open_pos', 0.96); p('gripper_closed_pos', -0.09)
        p('gripper_max_effort', 50.0)
        p('grasp_orientation', [1.0, 0.0, 0.0, 0.0])   # top-down (preferred)
        # LIVE-VERIFIED 2026-07-27 against this exact box pose (0.5,0,1.5): a
        # pure top-down grasp at arm_1's corner FAILS IK at every yaw (the box
        # top sits near the ceiling/gantry, z=1.725 -- the position-only reach
        # map flags the xyz reachable but a straight-down wrist has no room).
        # tilt=15deg still fails; tilt=30/45deg solve at every azimuth tested at
        # a generous 1.0s IK timeout, but only ~half the azimuths solve at the
        # real 0.3s ik_timeout (KDL random-restart variance) -- so this sweeps
        # yaw first (preferred, often works elsewhere) then BOTH tilt
        # magnitudes across all azimuths, same redundancy idea as
        # gantry_reach_executor's grasp_yaw_samples/grasp_tilt_*.
        p('grasp_yaw_samples', 8)
        p('grasp_tilt_samples', 2); p('grasp_tilt_max', 45.0)
        p('grasp_tilt_azimuths', 4)
        p('approach_offset', 0.10)             # m above each corner, pre-grasp
        p('grasp_descend', 0.05)
        p('lift_height', 0.20)
        p('ik_timeout', 0.3); p('ik_avoid_collisions', False)
        p('plan_time', 8.0); p('plan_attempts', 20)
        p('vel_scale', 0.1); p('acc_scale', 0.1); p('joint_tol', 0.01)
        p('stage_timeout', 60.0)               # s, per barrier stage
        p('execute', False)

        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        self.box_size = float(g('box_size'))
        self.reach_radius = float(g('reach_radius'))
        self.rail_step = float(g('rail_step')); self.rot_step = float(g('rot_step'))
        self.gripper_open = float(g('gripper_open_pos'))
        self.gripper_closed = float(g('gripper_closed_pos'))
        self.gripper_effort = float(g('gripper_max_effort'))
        self.grasp_ori = tuple(float(v) for v in g('grasp_orientation'))
        self.yaw_samples = int(g('grasp_yaw_samples'))
        self.tilt_samples = int(g('grasp_tilt_samples'))
        self.tilt_max = float(g('grasp_tilt_max'))
        self.tilt_azimuths = int(g('grasp_tilt_azimuths'))
        self.approach_offset = float(g('approach_offset'))
        self.grasp_descend = float(g('grasp_descend'))
        self.lift_height = float(g('lift_height'))
        self.ik_timeout = float(g('ik_timeout'))
        self.ik_avoid = bool(g('ik_avoid_collisions'))
        self.plan_time = float(g('plan_time'))
        self.plan_attempts = int(g('plan_attempts'))
        self.vel_scale = float(g('vel_scale')); self.acc_scale = float(g('acc_scale'))
        self.joint_tol = float(g('joint_tol'))
        self.stage_timeout = float(g('stage_timeout'))
        self.execute = bool(g('execute'))

        names = list(g('arm_names'))
        models = list(g('arm_models'))
        groups = list(g('arm_groups'))
        ees = list(g('arm_ee_frames'))
        grips = list(g('gripper_links'))
        gnames = list(g('gripper_names'))
        gjoints = list(g('gripper_joints'))
        self.arms = {}          # name -> ArmModel (+ .gripper_name/.gripper_joint)
        for i, (nm, mp, grp, ee, gl) in enumerate(zip(names, models, groups, ees, grips)):
            try:
                arm = ArmModel(nm, mp, grp, ee, gl)
            except (OSError, KeyError) as e:
                self.get_logger().error(f'could not load {nm} ({mp}): {e}')
                continue
            arm.gripper_name = gnames[i] if i < len(gnames) else None
            arm.gripper_joint = gjoints[i] if i < len(gjoints) else None
            self.arms[nm] = arm
        if len(self.arms) != 4:
            raise SystemExit(f'need exactly 4 arms loaded, got {len(self.arms)} '
                             '-- this node only does the 4-arm coalition case')
        # gantry index -> its 2 ArmModels, in GANTRY_ARMS order
        self._garms = {g_: [self.arms[n] for n in GANTRY_ARMS[g_]] for g_ in (0, 1)}
        self._trees = {}       # arm name -> (cKDTree(local FK cloud), q_all)
        for g_ in (0, 1):
            for nm in GANTRY_ARMS[g_]:
                dataset_path = models[names.index(nm)].replace(
                    '_model.npz', '_dataset.npz')
                d = np.load(dataset_path)
                pose = d['pose'][:, :3].astype(np.float64)
                q = d['q'].astype(np.float64)
                base = GANTRY_BASE[g_]
                v = pose - base - Z_OFFSET
                v[:, 0] -= q[:, 0]
                ang = -(np.pi / 2.0 + q[:, 1])
                c, s = np.cos(ang), np.sin(ang)
                loc = np.column_stack([c * v[:, 0] - s * v[:, 1],
                                       s * v[:, 0] + c * v[:, 1], v[:, 2]])
                self._trees[nm] = (cKDTree(loc), q)

        self._joints = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        cb = ReentrantCallbackGroup()
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.move_cli = ActionClient(self, MoveGroup, 'move_action', callback_group=cb)
        self.gripper_clients = {
            nm: ActionClient(self, GripperCommand,
                             f'/{arm.gripper_name}_controller/gripper_cmd',
                             callback_group=cb)
            for nm, arm in self.arms.items() if arm.gripper_name}
        self.create_subscription(String, '~/pick', self._on_pick, 1, callback_group=cb)
        self._busy = threading.Lock()
        self.get_logger().info(
            f'coalition_reach_executor up; arms={list(self.arms)}, '
            f'execute={self.execute}, box_size={self.box_size} m. Trigger: '
            f"ros2 topic pub --once {self.get_name()}/pick std_msgs/String "
            '"{data: \'x,y,z,yaw\'}"')

    # ---- state ----------------------------------------------------------
    def _on_joints(self, msg):
        for n, v in zip(msg.name, msg.position):
            self._joints[n] = v

    def _current_q(self, joint_names):
        try:
            return np.array([self._joints[n] for n in joint_names])
        except KeyError:
            return None

    # ---- decision (offline, numpy) ---------------------------------------
    def _nearest(self, arm_name, xyz, gantry, rail, theta):
        tree, q_all = self._trees[arm_name]
        loc = to_local(xyz, GANTRY_BASE[gantry], rail, theta)
        d, i = tree.query(loc)
        return float(d), q_all[int(i)]

    def _gantry_pair_plan(self, gantry, H, pair, cur_rail, cur_theta):
        """Cheapest (rail, theta, arm-order) for `gantry`'s 2 arms to reach
        H[pair[0]] and H[pair[1]] at ONE shared pose. Ranked by combined
        reach-map distance + a light gantry-travel tie-break (not full energy
        J -- this is the deliberately minimal decision step)."""
        a, b = GANTRY_ARMS[gantry]
        rails = np.arange(0.0, 2.0 + 1e-9, self.rail_step)
        rots = np.arange(-np.pi, np.pi, self.rot_step)
        best = None
        for order in ((a, b), (b, a)):
            for rail in rails:
                for theta in rots:
                    d0, q0 = self._nearest(order[0], H[pair[0]], gantry, rail, theta)
                    if d0 >= self.reach_radius:
                        continue
                    d1, q1 = self._nearest(order[1], H[pair[1]], gantry, rail, theta)
                    if d1 >= self.reach_radius:
                        continue
                    travel = abs(rail - cur_rail) + abs(theta - cur_theta)
                    cost = d0 + d1 + 0.05 * travel
                    if best is None or cost < best[0]:
                        best = (cost, rail, theta, order, (q0, q1))
        if best is None:
            return None
        cost, rail, theta, order, (q0, q1) = best
        # corner_idx[i] is the corner ARMS[i] takes -- positional, so
        # zip(arms, corner_idx) in the caller pairs them correctly regardless
        # of which order (a,b) vs (b,a) won.
        return dict(rail=rail, theta=theta, arms=order, corner_idx=pair,
                   q={order[0]: q0, order[1]: q1}, cost=cost)

    def _plan_coalition(self, box_xyz, yaw):
        """Cheapest feasible (gantry pose, arm, corner) assignment for all 4
        arms, or None. H = the 4 world corner handles for this box pose."""
        H = handle_corners(box_xyz, yaw, self.box_size)
        cur = {g_: (self._joints.get(GANTRY_JOINTS[g_][0], 0.0),
                    self._joints.get(GANTRY_JOINTS[g_][1], 0.0)) for g_ in (0, 1)}
        best, best_cost = None, None
        for pa, pb in PARTITIONS:
            for g_pairs in ((pa, pb), (pb, pa)):
                per_gantry, ok, total = {}, True, 0.0
                for g_ in (0, 1):
                    sub = self._gantry_pair_plan(g_, H, g_pairs[g_], *cur[g_])
                    if sub is None:
                        ok = False
                        break
                    per_gantry[g_] = sub
                    total += sub['cost']
                if ok and (best is None or total < best_cost):
                    best, best_cost = per_gantry, total
        if best is None:
            self.get_logger().error(
                'no feasible 4-arm assignment for this box pose/position '
                '(no gantry-pose pair puts both its arms within reach_radius '
                'of their corners)')
            return None
        return H, best

    # ---- concurrency helpers ----------------------------------------------
    @staticmethod
    def _send_all(senders):
        """Fire every sender NOW (no blocking between them) -- true
        concurrency. Returns the futures in the same order."""
        return [s() for s in senders]

    def _wait_all(self, futures, timeout):
        events = [threading.Event() for _ in futures]
        for f, ev in zip(futures, events):
            f.add_done_callback(lambda _f, ev=ev: ev.set())
        deadline = time.monotonic() + timeout
        for ev in events:
            ev.wait(max(0.0, deadline - time.monotonic()))
        return [f.result() if f.done() else None for f in futures]

    # ---- IK / plan / execute (single group, returns a not-yet-awaited call) -
    def _ori_candidates(self):
        """Grasp orientations to try, best-first: vertical (grasp_ori) at each
        yaw, then -- LIVE-VERIFIED NECESSARY at this box's corner height, see
        the params above -- the same yaws tilted off vertical, several
        magnitudes x azimuths. Mirrors gantry_reach_executor's
        _grasp_ori_candidates (kept separate: different self.* names)."""
        base = self.grasp_ori
        yaws = max(1, self.yaw_samples)
        out = []
        for i in range(yaws):
            yaw = 2.0 * np.pi * i / yaws
            out.append(quat_mul((0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)), base))
        if self.tilt_samples > 0 and self.tilt_max > 0:
            n_az = max(1, self.tilt_azimuths)
            for k in range(1, self.tilt_samples + 1):
                tilt = np.radians(self.tilt_max * k / self.tilt_samples)
                st, ct = np.sin(tilt / 2.0), np.cos(tilt / 2.0)
                for a in range(n_az):
                    phi = 2.0 * np.pi * a / n_az
                    q_tilt = (np.cos(phi) * st, np.sin(phi) * st, 0.0, ct)
                    out.append(quat_mul(q_tilt, base))
        return out

    def _ik(self, group, ee_frame, xyz, ori_list, seed_names, seed_q):
        """Try each orientation in ori_list (best-first); return the first
        solution, or None if every one fails."""
        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        ps.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        for quat in ori_list:
            ox, oy, oz, ow = quat
            ps.pose.orientation.x, ps.pose.orientation.y = float(ox), float(oy)
            ps.pose.orientation.z, ps.pose.orientation.w = float(oz), float(ow)
            req = build_ik_request(group, ee_frame, ps, seed_names, seed_q,
                                   timeout_s=self.ik_timeout, avoid_collisions=self.ik_avoid)
            fut = self.ik_cli.call_async(req)
            res = self._wait_all([fut], self.stage_timeout)[0]
            if res is not None and res.error_code.val == 1:
                return res.solution.joint_state
        return None

    def _joint_goal_sender(self, group, joint_names, positions):
        """Returns a zero-arg callable that, when called, FIRES the MoveGroup
        goal and returns its send_goal_async future (for _send_all)."""
        def send():
            req = MotionPlanRequest(group_name=group)
            con = Constraints()
            for n, v in zip(joint_names, positions):
                con.joint_constraints.append(JointConstraint(
                    joint_name=n, position=float(v), tolerance_above=self.joint_tol,
                    tolerance_below=self.joint_tol, weight=1.0))
            con_req = req
            con_req.goal_constraints = [con]
            con_req.num_planning_attempts = self.plan_attempts
            con_req.allowed_planning_time = self.plan_time
            con_req.max_velocity_scaling_factor = self.vel_scale
            con_req.max_acceleration_scaling_factor = self.acc_scale
            goal = MoveGroup.Goal(request=con_req,
                                  planning_options=PlanningOptions(plan_only=not self.execute))
            return self.move_cli.send_goal_async(goal)
        return send

    def _run_move_barrier(self, label, sends):
        """Send N MoveGroup goals CONCURRENTLY, wait for all to be accepted,
        then wait for all N results. Returns list of error_code.val (or None
        for a timed-out/rejected one). This IS the barrier."""
        goal_futs = self._send_all(sends)
        handles = self._wait_all(goal_futs, self.stage_timeout)
        if any(h is None or not h.accepted for h in handles):
            rejected = [i for i, h in enumerate(handles) if h is None or not h.accepted]
            self.get_logger().error(f'{label}: goal(s) {rejected} rejected/timed out')
            return [None] * len(sends)
        result_futs = [h.get_result_async() for h in handles]
        results = self._wait_all(result_futs, self.stage_timeout + self.plan_time)
        codes = [r.result.error_code.val if r is not None else None for r in results]
        self.get_logger().info(f'{label}: codes={codes}')
        return codes

    def _move_gripper_sender(self, arm_name, position):
        def send():
            client = self.gripper_clients[arm_name]
            client.wait_for_server(timeout_sec=5.0)
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = self.gripper_effort
            return client.send_goal_async(goal)
        return send

    def _run_gripper_barrier(self, position):
        arms = list(self.arms)
        start_pos = {nm: self._joints.get(self.arms[nm].gripper_joint) for nm in arms}
        goal_futs = self._send_all([self._move_gripper_sender(nm, position) for nm in arms])
        handles = self._wait_all(goal_futs, self.stage_timeout)
        if any(h is None or not h.accepted for h in handles):
            self.get_logger().error('gripper: goal(s) rejected/timed out')
            return {nm: False for nm in arms}
        result_futs = [h.get_result_async() for h in handles]
        results = self._wait_all(result_futs, self.stage_timeout)
        ok = {}
        for nm, r in zip(arms, results):
            if r is None:
                ok[nm] = False
                continue
            rr = r.result
            moved = (start_pos[nm] is not None
                    and abs(float(rr.position) - start_pos[nm]) > 0.02)
            ok[nm] = bool(rr.reached_goal or (rr.stalled and moved))
        self.get_logger().info(f'gripper: {ok}')
        return ok

    # ---- main sequence ------------------------------------------------------
    def _on_pick(self, msg):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('coalition pick already in progress; ignoring')
            return
        try:
            self._do_pick(msg.data)
        except Exception as e:  # noqa: BLE001 -- never die on one pick
            self.get_logger().error(f'coalition pick failed: {e}')
        finally:
            self._busy.release()

    def _do_pick(self, arg):
        try:
            x, y, z, yaw = (float(v) for v in arg.split(','))
        except ValueError:
            self.get_logger().error(f"bad pick arg '{arg}', want 'x,y,z,yaw'")
            return
        box_xyz = np.array([x, y, z])
        plan = self._plan_coalition(box_xyz, yaw)
        if plan is None:
            return
        H, per_gantry = plan
        self.get_logger().info(f'=== coalition pick: box@{box_xyz.round(3)} '
                               f'yaw={np.degrees(yaw):.0f}deg ===')
        for g_, sub in per_gantry.items():
            self.get_logger().info(f'  gantry_{g_+1}: rail={sub["rail"]:.2f} '
                                   f'theta={np.degrees(sub["theta"]):+.0f}deg '
                                   f'arms={sub["arms"]}')

        # Stage 1: position both gantries CONCURRENTLY.
        sends = [self._joint_goal_sender(GANTRY_GROUPS[g_], GANTRY_JOINTS[g_],
                                         (sub['rail'], sub['theta']))
                for g_, sub in per_gantry.items()]
        codes = self._run_move_barrier('stage1 gantries', sends)
        if any(c != 1 for c in codes):
            self.get_logger().error('=== COALITION FAILED: gantries did not '
                                    f'reach their poses (codes={codes}) ===')
            return

        self._grasp_stage(H, per_gantry)

    def _grasp_stage(self, H, per_gantry):
        # Flatten to arm_name -> (corner_idx, seed_q). sub['q'][arm_name] is the
        # GNG node's 8-DOF seed (gantry+arm); [2:] keeps only the 6 ARM joints,
        # since every group here is the arm-ONLY group (gantry is already fixed
        # by stage 1) -- arm.joint_names is likewise the arm's 6-DOF list
        # (ArmModel.joint_names[2:]), never the full 8, throughout this method.
        flat = {}       # arm_name -> (corner_idx, seed_q[6])
        for g_, sub in per_gantry.items():
            for arm_name, corner_idx in zip(sub['arms'], sub['corner_idx']):
                flat[arm_name] = (corner_idx, sub['q'][arm_name][2:])
        # Same orientation candidates for every arm/stage (position-independent):
        # a pure top-down grasp near the ceiling can need a tilt, LIVE-VERIFIED
        # for this box height -- see _ori_candidates / the grasp_tilt_* params.
        ori_list = self._ori_candidates()

        def solve_stage(label, target_of, seed_of):
            """IK every arm to its own target (seeded from `seed_of`), then run
            one move barrier for all of them. Returns the new goal_q dict, or
            None (already logged) if any arm has no IK / any code != 1."""
            goal_q = {}
            for nm, (ci, _) in flat.items():
                arm = self.arms[nm]
                arm_joints = arm.joint_names[2:]
                js = self._ik(arm.group, arm.ee_frame, target_of(nm, ci),
                              ori_list, arm_joints, seed_of(nm))
                if js is None:
                    self.get_logger().error(
                        f'=== COALITION FAILED: {nm} no IK for {label} '
                        f'(corner {ci}) ===')
                    return None
                pos = dict(zip(js.name, js.position))
                goal_q[nm] = np.array([pos[n] for n in arm_joints])
            codes = self._run_move_barrier(
                label, [self._joint_goal_sender(
                    self.arms[nm].group, self.arms[nm].joint_names[2:], goal_q[nm])
                    for nm in flat])
            if any(c != 1 for c in codes):
                self.get_logger().error(f'=== COALITION FAILED: {label} codes={codes} ===')
                return None
            return goal_q

        approach_q = solve_stage(
            'stage2 approach',
            lambda nm, ci: H[ci] + np.array([0.0, 0.0, self.approach_offset]),
            lambda nm: flat[nm][1])
        if approach_q is None:
            return

        descend_q = solve_stage(
            'stage3 descend',
            lambda nm, ci: H[ci] + np.array(
                [0.0, 0.0, self.approach_offset - self.grasp_descend]),
            lambda nm: approach_q[nm])
        if descend_q is None:
            return

        grip_ok = self._run_gripper_barrier(self.gripper_closed)
        if not all(grip_ok.values()):
            self.get_logger().error(
                f'=== COALITION FAILED: not all grippers confirmed a grasp '
                f'({grip_ok}) -- refusing to lift a partial grip ===')
            return

        lift_q = solve_stage(
            'stage4 lift',
            lambda nm, ci: H[ci] + np.array(
                [0.0, 0.0, self.approach_offset + self.lift_height]),
            lambda nm: descend_q[nm])
        if lift_q is None:
            return

        self.get_logger().info(
            f'=== COALITION SUCCESS: all 4 arms lifted the {self.box_size:.2f} m '
            f'box {self.lift_height:.2f} m ===')


def main():
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = CoalitionReachExecutor()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
