#!/usr/bin/env python3
"""One-shot runner that returns the whole workcell to home.

Order (chosen deliberately, see below):
  1. open all four grippers
  2. all four arms -> home, IN PARALLEL
  3. both tables -> home (0 mm / 0 deg), in parallel

Why arms before tables: the arms must be retracted before a gantry translates,
otherwise an extended arm is dragged through the surrounding structure.

Why grippers first: a gripper still holding an object is released at the current
pose, so the object drops where it is rather than being carried to home.

!! PARALLEL ARM MOTION — READ THIS !!
MoveIt's `move_action` server accepts one goal at a time (a second goal preempts
the first), so parallel arm motion cannot go through it. Instead each arm is
planned via the `plan_kinematic_path` service (planning only, no motion) and the
four resulting trajectories are then sent straight to the per-arm
`follow_joint_trajectory` controllers at the same time.

The consequence: every plan is collision-checked against the scene as it is
BEFORE anything moves, but the arms are NOT checked against each other while
they execute together. arm_1/arm_2 (and arm_3/arm_4) share a gantry and overlap
in workspace. Run with sequential_arms:=true to home them one at a time through
MoveIt instead if the starting poses put two arms of one gantry near each other.

Assumes MoveIt + per-arm controllers are already running (start_single_rviz.sh)
and a dual_table_controller owns the serial ports.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from control_msgs.action import GripperCommand, FollowJointTrajectory

from moving_table_interfaces.srv import MovingTable

# operation_type: move to an ABSOLUTE table position. The controller reads the
# motor encoder server-side and computes the move itself, so the sequence works
# from any starting position without homing and without depending on /joint_states.
OP_GOTO_ABS = 97


# Joint-name templates per planning group (from trailer_workcell.srdf)
ARM_JOINTS = {
    "arm_1": [f"t1_a1_joint_{i}" for i in range(1, 7)],
    "arm_2": [f"t1_a2_joint_{i}" for i in range(1, 7)],
    "arm_3": [f"t2_a1_joint_{i}" for i in range(1, 7)],
    "arm_4": [f"t2_a2_joint_{i}" for i in range(1, 7)],
}
# Planning group -> the FollowJointTrajectory controller that executes it
# (config/moveit_controllers_per_arm.yaml, spawned by single_rviz_workcell.launch.py).
ARM_CONTROLLER = {
    "arm_1": "arm_1_controller",
    "arm_2": "arm_2_controller",
    "arm_3": "arm_3_controller",
    "arm_4": "arm_4_controller",
}
GRIPPER_JOINT = {
    "gripper_1": "t1_a1_right_finger_bottom_joint",
    "gripper_2": "t1_a2_right_finger_bottom_joint",
    "gripper_3": "t2_a1_right_finger_bottom_joint",
    "gripper_4": "t2_a2_right_finger_bottom_joint",
}
TABLE_JOINTS = {
    "table1": ("t1_linear_joint", "t1_rotation_joint"),
    "table2": ("t2_linear_joint", "t2_rotation_joint"),
}

# Home joint angles (deg), same values the take-bag sequence homes to.
# arm_1/arm_3 and arm_2/arm_4 share mounting rpy, so all four arms share the
# same home pose (see run_take_bag.py's "arm1/2/3/4 home (parallel)" step —
# [90, -150, -150, ...] for arm_3/arm_4 was a choreography leftover, not a
# kinematic requirement).
ARM_HOME = {
    "arm_1": [0, 150, 150, 0, 0, 0],
    "arm_2": [0, 150, 150, 0, 0, 0],
    "arm_3": [0, 150, 150, 0, 0, 0],
    "arm_4": [0, 150, 150, 0, 0, 0],
}


class SequenceRunner(Node):
    def __init__(self):
        super().__init__("go_home_runner")

        # Final outcome for real_robot_task_server. `ros2 launch` does not
        # propagate a child's exit code, so sys.exit() alone never reaches the
        # task server -- this topic is what tells it the sequence failed.
        self._result_pub = self.create_publisher(
            Bool,
            "/task/result",
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # --- Parameters ---
        self.gripper_open_deg = self.declare_parameter("gripper_open_deg", 0.0).value
        self.linear_speed = self.declare_parameter("linear_speed", 3000).value
        self.rotate_speed = self.declare_parameter("rotate_speed", 1000).value
        self.planning_time = self.declare_parameter("planning_time", 10.0).value
        self.vel_scale = self.declare_parameter("vel_scale", 0.1).value
        self.acc_scale = self.declare_parameter("acc_scale", 0.1).value
        self.table_timeout_s = self.declare_parameter("table_timeout_s", 120.0).value
        self.table_tol_mm = self.declare_parameter("table_tol_mm", 5.0).value
        self.table_tol_deg = self.declare_parameter("table_tol_deg", 2.0).value
        self.table_stable_samples = self.declare_parameter("table_stable_samples", 6).value
        self.startup_delay_s = self.declare_parameter("startup_delay_s", 3.0).value
        self.motor_settle_s = self.declare_parameter("motor_settle_s", 1.0).value
        self.gripper_max_effort = self.declare_parameter("gripper_max_effort", 50.0).value
        self.skip_grippers = self.declare_parameter("skip_grippers", False).value
        self.arm_settle_s = self.declare_parameter("arm_settle_s", 0.5).value
        self.gripper_pre_delay_s = self.declare_parameter("gripper_pre_delay_s", 1.5).value
        # Escape hatch: home the arms one at a time through MoveIt instead of all
        # four at once. Slower, but arm-vs-arm collisions are then impossible.
        self.sequential_arms = self.declare_parameter("sequential_arms", False).value
        self.arm_exec_timeout_s = self.declare_parameter("arm_exec_timeout_s", 120.0).value

        # Latest /joint_states sample (name -> position)
        self._joint_state = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        # MoveIt action client (used for sequential_arms mode)
        self._move_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server (move_action)...")
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("MoveGroup action server not available.")

        # MoveIt planning-only service (used for the parallel path)
        self._plan_client = self.create_client(GetMotionPlan, "plan_kinematic_path")
        self.get_logger().info("Waiting for plan_kinematic_path service...")
        if not self._plan_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("plan_kinematic_path service not available.")

        # Table service client
        self._table_client = self.create_client(MovingTable, "move_dual_table")
        self.get_logger().info("Waiting for move_dual_table service...")
        if not self._table_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("move_dual_table service not available.")

        # Per-arm trajectory controllers, for parallel execution.
        self._traj_clients = {
            group: ActionClient(
                self, FollowJointTrajectory, f"/{ctrl}/follow_joint_trajectory"
            )
            for group, ctrl in ARM_CONTROLLER.items()
        }

        # Gripper GripperCommand action clients (one per gripper controller).
        self._gripper_clients = {
            g: ActionClient(self, GripperCommand, f"/{g}_controller/gripper_cmd")
            for g in GRIPPER_JOINT
        }

        self.get_logger().info("Connected. Go-home runner ready.")

    # ------------------------------------------------------------------
    # Callbacks / helpers
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_state[name] = pos

    def _spin_until(self, future, timeout_sec=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

    def _joint_goal_constraints(self, joint_names, degrees_list) -> Constraints:
        constraints = Constraints()
        constraints.name = "goal"
        for name, deg in zip(joint_names, degrees_list):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = math.radians(deg)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    # ------------------------------------------------------------------
    # Arm / gripper motion via MoveIt joint-space goal
    # ------------------------------------------------------------------
    def move_joints(self, group_name, joint_names, degrees_list) -> bool:
        if len(joint_names) != len(degrees_list):
            self.get_logger().error(
                f"{group_name}: joint/value count mismatch "
                f"({len(joint_names)} vs {len(degrees_list)})."
            )
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = float(self.planning_time)
        # Cap speed: a 0.0 scaling factor is treated as full speed by MoveIt,
        # which trips the Kinova protective stop on large swings.
        goal.request.max_velocity_scaling_factor = float(self.vel_scale)
        goal.request.max_acceleration_scaling_factor = float(self.acc_scale)
        goal.request.goal_constraints.append(
            self._joint_goal_constraints(joint_names, degrees_list)
        )

        self.get_logger().info(f"→ {group_name}: {degrees_list}")
        # Brief settle so joint states are stable after any prior move
        if self.arm_settle_s > 0:
            time.sleep(self.arm_settle_s)
        goal_handle = self._spin_until(self._move_client.send_goal_async(goal))
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{group_name}: goal rejected by move_action.")
            return False

        result = self._spin_until(goal_handle.get_result_async())
        if result is None:
            self.get_logger().error(f"{group_name}: no result returned.")
            return False
        code = result.result.error_code.val
        if code != 1:
            self.get_logger().error(f"{group_name}: planning/execution failed (code {code}).")
            return False
        return True

    def move_arms_parallel(self, targets) -> bool:
        """Move several arms to joint targets at the SAME time.

        `targets` is a list of (group_name, joint_names, degrees_list).

        Phase 1 plans every arm through `plan_kinematic_path` (no motion). If any
        plan fails we abort before anything has moved. Phase 2 sends the planned
        trajectories to the per-arm controllers back-to-back, then waits for all
        of them, so the total time is the slowest arm rather than the sum.

        Collision caveat: each plan is checked against the scene as it is BEFORE
        the batch starts. Arms executing concurrently are not checked against
        each other -- see the module docstring.
        """
        # Phase 1: plan everything first.
        plans = []
        for group_name, joint_names, degrees_list in targets:
            if len(joint_names) != len(degrees_list):
                self.get_logger().error(
                    f"{group_name}: joint/value count mismatch "
                    f"({len(joint_names)} vs {len(degrees_list)})."
                )
                return False

            req = GetMotionPlan.Request()
            mpr = MotionPlanRequest()
            mpr.group_name = group_name
            mpr.num_planning_attempts = 5
            mpr.allowed_planning_time = float(self.planning_time)
            mpr.max_velocity_scaling_factor = float(self.vel_scale)
            mpr.max_acceleration_scaling_factor = float(self.acc_scale)
            # is_diff with no joint values = "start from the current state".
            mpr.start_state.is_diff = True
            mpr.goal_constraints.append(
                self._joint_goal_constraints(joint_names, degrees_list)
            )
            req.motion_plan_request = mpr

            self.get_logger().info(f"→ planning {group_name}: {degrees_list}")
            result = self._spin_until(
                self._plan_client.call_async(req),
                timeout_sec=float(self.planning_time) + 15.0,
            )
            if result is None:
                self.get_logger().error(f"{group_name}: planning call timed out.")
                return False
            code = result.motion_plan_response.error_code.val
            if code != 1:
                self.get_logger().error(f"{group_name}: planning failed (code {code}).")
                return False
            traj = result.motion_plan_response.trajectory.joint_trajectory
            if not traj.points:
                self.get_logger().error(f"{group_name}: planner returned an empty trajectory.")
                return False
            plans.append((group_name, traj))

        # Phase 2a: fire every trajectory. Controllers are independent
        # (one per arm), so these run concurrently.
        if self.arm_settle_s > 0:
            time.sleep(self.arm_settle_s)
        handles = []
        for group_name, traj in plans:
            client = self._traj_clients[group_name]
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error(
                    f"{group_name}: {ARM_CONTROLLER[group_name]} action server not available."
                )
                return False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            self.get_logger().info(f"→ executing {group_name} (parallel)")
            goal_handle = self._spin_until(client.send_goal_async(goal))
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{group_name}: trajectory goal rejected.")
                # Arms already sent are still moving; report loudly rather than
                # pretending the batch is clean.
                self.get_logger().error(
                    "Some arms may still be executing — issue a stop if needed."
                )
                return False
            handles.append((group_name, goal_handle))

        # Phase 2b: wait for all of them. They are already moving, so waiting
        # one after another still costs only the slowest arm.
        ok = True
        for group_name, goal_handle in handles:
            result = self._spin_until(
                goal_handle.get_result_async(), timeout_sec=self.arm_exec_timeout_s
            )
            if result is None:
                self.get_logger().error(f"{group_name}: execution timed out.")
                ok = False
                continue
            err = result.result.error_code
            if err != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"{group_name}: execution failed (error_code {err}, "
                    f"{result.result.error_string})."
                )
                ok = False
        return ok

    def move_gripper(self, gripper_group, degrees) -> bool:
        """Drive a gripper via its GripperCommand action.

        Success = reached the goal, OR stalled after actually moving. A stall
        with no movement means the gripper hardware did not actuate — retried
        once (a single transient comm/stall hiccup shouldn't abort the whole
        go-home sequence), then reported as failure. If skip_grippers is set,
        the step is a no-op."""
        if self.skip_grippers:
            self.get_logger().warn(f"{gripper_group}: skip_grippers set — skipping.")
            return True

        joint = GRIPPER_JOINT[gripper_group]

        # If already at target (within ~3°), skip — sending the goal would stall
        # immediately with no movement, which the driver reports as failure.
        start_pos = self._joint_state.get(joint)
        if start_pos is not None and abs(start_pos - math.radians(degrees)) <= 0.05:
            self.get_logger().info(f"{gripper_group}: already at {degrees}°, skipping.")
            return True

        client = self._gripper_clients[gripper_group]
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f"{gripper_group}: gripper action server not available.")
            return False

        for attempt in (1, 2):
            start_pos = self._joint_state.get(joint)

            goal = GripperCommand.Goal()
            goal.command.position = math.radians(degrees)
            goal.command.max_effort = float(self.gripper_max_effort)

            self.get_logger().info(f"→ {gripper_group}: {degrees}° (attempt {attempt}/2)")
            if self.gripper_pre_delay_s > 0:
                time.sleep(self.gripper_pre_delay_s)
            goal_handle = self._spin_until(client.send_goal_async(goal))
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{gripper_group}: goal rejected.")
                return False

            result = self._spin_until(goal_handle.get_result_async())
            if result is None:
                self.get_logger().error(f"{gripper_group}: no result returned.")
                return False

            r = result.result
            if r.reached_goal:
                return True
            # Opening onto nothing should reach the goal. A stall that still moved
            # the finger means it opened as far as it could — good enough for a
            # release.
            end_pos = r.position
            moved = (start_pos is not None and abs(end_pos - start_pos) > 0.02)
            if r.stalled and moved:
                self.get_logger().info(f"{gripper_group}: stalled after moving (pos={end_pos:.3f}).")
                return True
            if attempt == 1:
                self.get_logger().warn(
                    f"{gripper_group}: did not actuate on attempt 1 "
                    f"(stalled={r.stalled}, start={start_pos}, end={end_pos:.4f}) — retrying once."
                )
                continue
            self.get_logger().error(
                f"{gripper_group}: did not actuate after retry (stalled={r.stalled}, "
                f"reached={r.reached_goal}, start={start_pos}, end={end_pos:.4f}). "
                f"Gripper is not responding to commands — check for a mechanical jam or a "
                f"gripper-level fault on this arm, or run with skip_grippers:=true."
            )
            return False

    # ------------------------------------------------------------------
    # Table motion via service (absolute target + wait for completion)
    # ------------------------------------------------------------------
    def _wait_for_table(self, table_id, target_mm, target_deg) -> bool:
        """Block until the table is STABLY at the target.

        Returns True only after the joints read within tolerance for
        `table_stable_samples` CONSECUTIVE /joint_states samples. A single
        in-tolerance reading is not enough — the table can pass through the
        target while still moving (or a stale Modbus read can momentarily land
        in tolerance).
        """
        lin_joint, rot_joint = TABLE_JOINTS[table_id]
        target_m = target_mm / 1000.0
        target_rad = math.radians(target_deg)
        tol_m = self.table_tol_mm / 1000.0
        tol_rad = math.radians(self.table_tol_deg)

        deadline = time.time() + self.table_timeout_s
        stable = 0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            lin = self._joint_state.get(lin_joint)
            rot = self._joint_state.get(rot_joint)
            if lin is None or rot is None:
                continue
            if abs(lin - target_m) <= tol_m and abs(rot - target_rad) <= tol_rad:
                stable += 1
                if stable >= self.table_stable_samples:
                    if self.motor_settle_s > 0:
                        time.sleep(self.motor_settle_s)
                    return True
            else:
                stable = 0  # left tolerance -> still moving; restart the count
        self.get_logger().error(
            f"{table_id}: timed out waiting to settle at target "
            f"(last lin={self._joint_state.get(lin_joint)}, rot={self._joint_state.get(rot_joint)}, "
            f"stable={stable}/{self.table_stable_samples})."
        )
        return False

    def move_tables_parallel(self, targets) -> bool:
        """Command several tables to absolute targets concurrently, then wait for
        all of them to settle. `targets` is a list of (table_id, target_mm,
        target_deg). The move_dual_table service is non-blocking server-side and
        each table has its own serial port, so firing all requests first lets the
        tables move at the same time; we then wait for each to reach tolerance
        (total wait ~= the slowest table, not the sum)."""
        # Phase 1: fire every service request (server-side moves are non-blocking).
        for table_id, target_mm, target_deg in targets:
            req = MovingTable.Request()
            req.table_id = table_id
            req.distance_mm = float(target_mm)
            req.angle_deg = float(target_deg)
            req.linear_speed = int(self.linear_speed)
            req.rotate_speed = int(self.rotate_speed)
            req.operation_type = OP_GOTO_ABS
            self.get_logger().info(
                f"→ {table_id}: absolute target {target_mm} mm / {target_deg} deg (parallel)"
            )
            result = self._spin_until(self._table_client.call_async(req), timeout_sec=10.0)
            if result is None:
                self.get_logger().error(f"{table_id}: service call timed out.")
                return False
            if not result.success:
                self.get_logger().error(f"{table_id}: service rejected — {result.message}")
                return False

        # Phase 2: wait for every table to settle (they are already moving).
        ok = True
        for table_id, target_mm, target_deg in targets:
            if not self._wait_for_table(table_id, target_mm, target_deg):
                ok = False
        return ok

    # ------------------------------------------------------------------
    # The choreography
    # ------------------------------------------------------------------
    def _home_arms(self) -> bool:
        if self.sequential_arms:
            for group in ("arm_1", "arm_2", "arm_3", "arm_4"):
                if not self.move_joints(group, ARM_JOINTS[group], ARM_HOME[group]):
                    return False
            return True
        return self.move_arms_parallel(
            [(g, ARM_JOINTS[g], ARM_HOME[g]) for g in ("arm_1", "arm_2", "arm_3", "arm_4")]
        )

    def run_sequence(self) -> bool:
        opn = self.gripper_open_deg

        # (label, callable) — executed in order; abort on first failure.
        steps = [
            ("01 gripper1 open",  lambda: self.move_gripper("gripper_1", opn)),
            ("02 gripper2 open",  lambda: self.move_gripper("gripper_2", opn)),
            ("03 gripper3 open",  lambda: self.move_gripper("gripper_3", opn)),
            ("04 gripper4 open",  lambda: self.move_gripper("gripper_4", opn)),
            ("05 arms -> home",   self._home_arms),
            ("06 tables -> home", lambda: self.move_tables_parallel(
                                      [("table1", 0.0, 0.0), ("table2", 0.0, 0.0)])),
        ]

        for label, action in steps:
            self.get_logger().info(f"=== Step {label} ===")
            if not action():
                self.get_logger().error(f"ABORTING: step '{label}' failed.")
                return False
            self.get_logger().info(f"Step {label} OK")
        return True



    def publish_result(self, ok: bool) -> None:
        """Publish the sequence outcome, then spin briefly so it goes out.

        The process exits right after this; without the short spin the sample
        can be dropped before the task server ever sees it.
        """
        msg = Bool()
        msg.data = bool(ok)
        # Wait for the task server to actually match this publisher first. The
        # process exits right after publishing, and a sample sent before the
        # subscriber has discovered us is simply dropped -- TRANSIENT_LOCAL
        # only replays to subscribers that find us while we are still alive.
        end = time.time() + 2.0
        while time.time() < end and self._result_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._result_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscriber on /task/result; outcome may not reach the task server."
            )
        self._result_pub.publish(msg)
        end = time.time() + 0.5
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

def main(args=None):
    rclpy.init(args=args)
    node = None
    ok = False
    try:
        node = SequenceRunner()
        if node.startup_delay_s > 0:
            node.get_logger().info(f"Startup delay {node.startup_delay_s}s...")
            end = time.time() + node.startup_delay_s
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.run_sequence()
        if ok:
            node.get_logger().info("✅ Sequence complete.")
        else:
            node.get_logger().error("❌ Sequence did not complete.")
    except Exception as e:
        if node is not None:
            node.get_logger().fatal(f"Fatal error: {e}")
        else:
            print(f"Fatal error before node init: {e}")
    finally:
        if node is not None:
            try:
                node.publish_result(ok)
            except Exception as e:  # never mask the real outcome
                node.get_logger().error(f"Failed to publish result: {e}")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
