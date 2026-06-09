#!/usr/bin/env python3
"""One-shot runner for the take-the-bag demo choreography (Sequence_ceilingArm_demo.pdf).

Drives the workcell through a fixed sequence:
  - tables via the `move_dual_table` service (moving_table_interfaces/srv/MovingTable)
  - arms + grippers via MoveIt's `move_action` (joint-space goals)

Assumes `my_workcell.launch.py` is already running (MoveIt, controllers, table node).
Runs the steps in order, aborting if any step fails, then exits.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import GripperCommand

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


class SequenceRunner(Node):
    def __init__(self):
        super().__init__("take_bag_runner")

        # --- Parameters ---
        self.gripper_grip_deg = self.declare_parameter("gripper_grip_deg", 45.0).value
        self.gripper_open_deg = self.declare_parameter("gripper_open_deg", 0.0).value
        self.linear_speed = self.declare_parameter("linear_speed", 3000).value
        self.rotate_speed = self.declare_parameter("rotate_speed", 1000).value
        self.planning_time = self.declare_parameter("planning_time", 10.0).value
        self.vel_scale = self.declare_parameter("vel_scale", 0.1).value
        self.acc_scale = self.declare_parameter("acc_scale", 0.1).value
        self.table_timeout_s = self.declare_parameter("table_timeout_s", 120.0).value
        self.table_tol_mm = self.declare_parameter("table_tol_mm", 5.0).value
        self.table_tol_deg = self.declare_parameter("table_tol_deg", 2.0).value
        # Number of CONSECUTIVE in-tolerance /joint_states samples required before
        # a table is treated as "in position". One sample is not enough: the table
        # can pass through the target while still moving, or a stale Modbus read
        # can momentarily land in tolerance. ~6 samples @ ~0.2s ≈ 1.2s settled.
        self.table_stable_samples = self.declare_parameter("table_stable_samples", 6).value
        self.startup_delay_s = self.declare_parameter("startup_delay_s", 3.0).value
        self.motor_settle_s = self.declare_parameter("motor_settle_s", 1.0).value
        self.gripper_max_effort = self.declare_parameter("gripper_max_effort", 50.0).value
        self.skip_grippers = self.declare_parameter("skip_grippers", False).value
        self.arm_settle_s = self.declare_parameter("arm_settle_s", 0.5).value
        # Extra delay inside move_gripper before sending the goal. After an arm
        # trajectory the Kortex hardware stays in high-level servoing mode; the
        # gripper controller uses low-level BaseCyclic API and gets
        # WRONG_SERVOING_MODE until the hardware transitions. 1.5 s is enough
        # in practice; lower if grippers feel sluggish.
        self.gripper_pre_delay_s = self.declare_parameter("gripper_pre_delay_s", 1.5).value

        # Latest /joint_states sample (name -> position)
        self._joint_state = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        # MoveIt action client
        self._move_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server (move_action)...")
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("MoveGroup action server not available.")

        # Table service client
        self._table_client = self.create_client(MovingTable, "move_dual_table")
        self.get_logger().info("Waiting for move_dual_table service...")
        if not self._table_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("move_dual_table service not available.")

        # Gripper GripperCommand action clients (one per gripper controller).
        # MoveGroup planning reports CONTROL_FAILED when a Kinova gripper stalls
        # on an object, so we drive grippers through their action directly.
        self._gripper_clients = {
            g: ActionClient(self, GripperCommand, f"/{g}_controller/gripper_cmd")
            for g in GRIPPER_JOINT
        }

        self.get_logger().info("Connected. Demo runner ready.")

    # ------------------------------------------------------------------
    # Callbacks / helpers
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_state[name] = pos

    def _spin_until(self, future, timeout_sec=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

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
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f"→ {group_name}: {degrees_list}")
        # Brief settle so joint states are stable after any prior move
        if self.arm_settle_s > 0:
            time.sleep(self.arm_settle_s)
        send_future = self._move_client.send_goal_async(goal)
        goal_handle = self._spin_until(send_future)
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

    def move_gripper(self, gripper_group, degrees) -> bool:
        """Drive a gripper via its GripperCommand action.

        Success = reached the goal, OR stalled after actually moving (gripped an
        object). A stall with no movement means the gripper hardware did not
        actuate — reported as failure. If skip_grippers is set, the step is a
        no-op (lets the arm/table choreography run while the gripper hardware is
        being debugged)."""
        if self.skip_grippers:
            self.get_logger().warn(f"{gripper_group}: skip_grippers set — skipping.")
            return True

        joint = GRIPPER_JOINT[gripper_group]
        start_pos = self._joint_state.get(joint)

        # If already at target (within ~3°), skip — sending the goal would stall
        # immediately with no movement, which the driver reports as failure.
        if start_pos is not None and abs(start_pos - math.radians(degrees)) <= 0.05:
            self.get_logger().info(f"{gripper_group}: already at {degrees}°, skipping.")
            return True

        client = self._gripper_clients[gripper_group]
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f"{gripper_group}: gripper action server not available.")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = math.radians(degrees)
        goal.command.max_effort = float(self.gripper_max_effort)

        self.get_logger().info(f"→ {gripper_group}: {degrees}°")
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
        # Stalled is only a real grip if the finger actually moved from its start.
        # Use r.position (authoritative from the driver) instead of joint_states,
        # which can be stale when the action result arrives.
        end_pos = r.position
        moved = (start_pos is not None and abs(end_pos - start_pos) > 0.02)
        if r.stalled and moved:
            self.get_logger().info(f"{gripper_group}: stalled on object (pos={end_pos:.3f}).")
            return True
        self.get_logger().error(
            f"{gripper_group}: did not actuate (stalled={r.stalled}, "
            f"reached={r.reached_goal}, start={start_pos}, end={end_pos:.4f}). "
            f"Grip angle must be ≤46° (kortex max 0.81 rad). "
            f"Check gripper hardware/driver, or run with skip_grippers:=true."
        )
        return False

    # ------------------------------------------------------------------
    # Table motion via service (absolute target + wait for completion)
    # ------------------------------------------------------------------
    def move_table(self, table_id, target_mm, target_deg) -> bool:
        # Send the ABSOLUTE target. The controller reads the motor encoder
        # server-side and computes the move — no /joint_states delta on the client,
        # so this is correct from any starting position.
        req = MovingTable.Request()
        req.table_id = table_id
        req.distance_mm = float(target_mm)
        req.angle_deg = float(target_deg)
        req.linear_speed = int(self.linear_speed)
        req.rotate_speed = int(self.rotate_speed)
        req.operation_type = OP_GOTO_ABS

        self.get_logger().info(f"→ {table_id}: absolute target {target_mm} mm / {target_deg} deg")
        result = self._spin_until(self._table_client.call_async(req), timeout_sec=10.0)
        if result is None:
            self.get_logger().error(f"{table_id}: service call timed out.")
            return False
        if not result.success:
            self.get_logger().error(f"{table_id}: service rejected — {result.message}")
            return False

        # Service is non-blocking; wait for the joints to settle at the target.
        return self._wait_for_table(table_id, target_mm, target_deg)

    def _wait_for_table(self, table_id, target_mm, target_deg) -> bool:
        """Block until the table is STABLY at the target.

        Returns True only after the joints read within tolerance for
        `table_stable_samples` CONSECUTIVE /joint_states samples. A single
        in-tolerance reading is not enough — the table can pass through the
        target while still moving (or a stale Modbus read can momentarily land
        in tolerance), which previously let the next arm step start before the
        table was actually in position.
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

    def require_table_home(self, table_id) -> bool:
        """Block until `table_id` is confirmed at home (0 mm / 0 deg) before the
        handoff arm is allowed to move. Reuses _wait_for_table's tolerance
        (table_tol_mm / table_tol_deg) and timeout (table_timeout_s); aborts on
        timeout rather than letting the arm move into a mis-positioned table.

        NOTE: this reads /joint_states, the same source step 11 already waited on.
        If the table's absolute encoder origin is not set (ppreset / 'H' in
        table_keyboard.py), /joint_states can report home while the table is not
        physically home — this guard cannot catch that case. Zero the encoder first.
        """
        self.get_logger().info(
            f"GUARD: blocking until {table_id} is home (0 mm / 0 deg) before arm2 moves..."
        )
        return self._wait_for_table(table_id, 0.0, 0.0)

    # ------------------------------------------------------------------
    # The choreography
    # ------------------------------------------------------------------
    def run_sequence(self) -> bool:
        grip = self.gripper_grip_deg
        opn = self.gripper_open_deg

        # (label, callable) — executed in order; abort on first failure.
        steps = [
            ("01 tables -> staging (t1 0/0, t2 650/90)",
                                           lambda: self.move_tables_parallel(
                                               [("table1", 0.0, 0.0), ("table2", 650.0, 90.0)])),
            ("02 gripper1 open",           lambda: self.move_gripper("gripper_1", opn)),
            ("03 gripper2 open",           lambda: self.move_gripper("gripper_2", opn)),
            ("04 gripper3 open",           lambda: self.move_gripper("gripper_3", opn)),
            ("05 gripper4 open",           lambda: self.move_gripper("gripper_4", opn)),
            ("06 arm4 approach 1",         lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -43, -56, -90, 76, 0])),
            ("07 arm4 approach 2",         lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -10, -63, -90, 35, 0])),
            ("08 gripper4 grip",           lambda: self.move_gripper("gripper_4", grip)),
            ("09 arm4 reach",              lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -75, -130, -90, 30, 2])),
            ("10 arm4 lift",               lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [0, -60, -115, -90, 30, 2])),
            ("11 arm4 carry",              lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [0, 78, -5, -90, -86, 0])),
            ("12 arm3 pre-approach",       lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [0, -55, -95, 90, -44, 90])),
            ("13 arm3 approach",           lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [4, -20, -81, 101, -29, 79])),
            ("14 gripper3 grip",           lambda: self.move_gripper("gripper_3", grip)),
            ("15 gripper4 release",        lambda: self.move_gripper("gripper_4", 20.0)),
            ("16 arm4 retreat",            lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [0, 86, -10, -90, -99, 0])),
            ("17 arm3 move",               lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [88, 94, -14, -93, -95, 24])),
            ("18 table1 -> home",          lambda: self.move_table("table1", 0.0, 0.0)),
            ("19 GUARD table1 home",       lambda: self.require_table_home("table1")),
            ("20 arm2 approach 1",         lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [28, -47, -129, 90, -9, -91])),
            ("21 arm2 approach 2",         lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [28, 21, -86, 90, 16, -90])),
            ("22 gripper2 grip",           lambda: self.move_gripper("gripper_2", grip)),
            ("23 gripper3 loosen",         lambda: self.move_gripper("gripper_3", opn)),
            ("24 arm2 place",              lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [0, -45, -99, 90, 55, -90])),
            ("25 gripper2 release",        lambda: self.move_gripper("gripper_2", opn)),
            ("26 arm2 home",               lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [0, 150, 150, 0, 0, 0])),
            ("27 arm3 home",               lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [90, -150, -150, 0, 0, 0])),
            ("28 arm4 home",               lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -150, -150, 0, 0, 0])),
            ("29 arm1 home",               lambda: self.move_joints("arm_1", ARM_JOINTS["arm_1"], [0, 150, 150, 0, 0, 0])),
            ("30 table1 -> home",          lambda: self.move_table("table1", 0.0, 0.0)),
            ("31 table2 -> home",          lambda: self.move_table("table2", 0.0, 0.0)),
        ]

        for label, action in steps:
            self.get_logger().info(f"=== Step {label} ===")
            if not action():
                self.get_logger().error(f"ABORTING: step '{label}' failed.")
                return False
            self.get_logger().info(f"Step {label} OK")
        return True


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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    main()
