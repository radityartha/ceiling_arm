#!/usr/bin/env python3
"""One-shot runner for the open-curtain choreography.

Sequence:
  1  gripper1 open
  2  gripper2 open
  3  table1 → 1268 mm / 0 deg
  4  arm1 pre-approach
  5  arm1 approach
  6  gripper1 grip (40 deg)
  7  arm1 open-curtain swing
  8  table1 → 1352 mm
  9  gripper1 open
  10 arm1 retreat + table1 → 1268 mm (parallel)
  11 arm1 home
  12 table1 → 1268 mm / 0 deg
  13 arm2 pre-approach
  14 arm2 approach
  15 gripper2 grip (40 deg)
  16 arm2 open-curtain swing
  17 table1 → 1170 mm
  18 gripper2 open
  19 arm2 joint1 clear (rotate to +120 deg)
  20 arm2 lift
  21 arm2 home
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import SwitchController

from rclpy.duration import Duration
from moving_table_interfaces.srv import MovingTable

# operation_type: move to an ABSOLUTE table position. The controller reads the
# motor encoder server-side and computes the move itself, so the sequence works
# from any starting position without homing and without depending on /joint_states.
OP_GOTO_ABS = 97


ARM_JOINTS = {
    "arm_1": [f"t1_a1_joint_{i}" for i in range(1, 7)],
    "arm_2": [f"t1_a2_joint_{i}" for i in range(1, 7)],
}
GRIPPER_JOINT = {
    "gripper_1": "t1_a1_right_finger_bottom_joint",
    "gripper_2": "t1_a2_right_finger_bottom_joint",
}
TABLE_JOINTS = {
    "table1": ("t1_linear_joint", "t1_rotation_joint"),
}


class OpenCurtainRunner(Node):
    def __init__(self):
        super().__init__("open_curtain_runner")

        self.gripper_grip_deg = self.declare_parameter("gripper_grip_deg", 40.0).value
        self.gripper_open_deg = self.declare_parameter("gripper_open_deg", 0.0).value
        self.linear_speed = self.declare_parameter("linear_speed", 3000).value
        self.rotate_speed = self.declare_parameter("rotate_speed", 1000).value
        self.planning_time = self.declare_parameter("planning_time", 10.0).value
        self.vel_scale = self.declare_parameter("vel_scale", 0.1).value
        self.acc_scale = self.declare_parameter("acc_scale", 0.1).value
        self.table_timeout_s = self.declare_parameter("table_timeout_s", 120.0).value
        self.table_tol_mm = self.declare_parameter("table_tol_mm", 5.0).value
        self.table_tol_deg = self.declare_parameter("table_tol_deg", 2.0).value
        self.startup_delay_s = self.declare_parameter("startup_delay_s", 3.0).value
        self.motor_settle_s = self.declare_parameter("motor_settle_s", 1.0).value
        self.gripper_max_effort = self.declare_parameter("gripper_max_effort", 50.0).value
        self.skip_grippers = self.declare_parameter("skip_grippers", False).value
        self.arm_settle_s = self.declare_parameter("arm_settle_s", 0.5).value

        self.use_compliant_pull = self.declare_parameter("use_compliant_pull", False).value
        self.pull_target_deg = self.declare_parameter("pull_target_deg", -120.0).value
        self.pull_tol_deg = self.declare_parameter("pull_tol_deg", 3.0).value
        self.pull_timeout_s = self.declare_parameter("pull_timeout_s", 15.0).value
        self.pull_rate_hz = self.declare_parameter("pull_rate_hz", 40.0).value
        self.pull_effort_limit = self.declare_parameter("pull_effort_limit", 8.0).value
        self.pull_twist = self.declare_parameter(
            "pull_twist", [0.0, -0.05, 0.0, 0.0, 0.0, 0.0]).value
        self.twist_controller = self.declare_parameter(
            "twist_controller", "t1_a2_twist_controller").value
        self.arm2_traj_controller = self.declare_parameter(
            "arm2_traj_controller", "arm_2_controller").value

        self._joint_state = {}
        self._joint_effort = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self._twist_pub = self.create_publisher(
            Twist, f"/{self.twist_controller}/commands", 10)
        self._switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller")

        self._move_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("MoveGroup action server not available.")

        self._table_client = self.create_client(MovingTable, "move_dual_table")
        self.get_logger().info("Waiting for move_dual_table service...")
        if not self._table_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("move_dual_table service not available.")

        self._gripper_clients = {
            g: ActionClient(self, GripperCommand, f"/{g}_controller/gripper_cmd")
            for g in GRIPPER_JOINT
        }

        self.get_logger().info("Connected. Open-curtain runner ready.")

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_state[name] = pos
        for name, eff in zip(msg.name, msg.effort):
            self._joint_effort[name] = eff

    def _spin_until(self, future, timeout_sec=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

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
        if self.skip_grippers:
            self.get_logger().warn(f"{gripper_group}: skip_grippers set — skipping.")
            return True

        joint = GRIPPER_JOINT[gripper_group]
        start_pos = self._joint_state.get(joint)

        if start_pos is not None and abs(start_pos - math.radians(degrees)) < 0.05:
            self.get_logger().info(f"→ {gripper_group}: already at {degrees}° — skipping.")
            return True

        client = self._gripper_clients[gripper_group]
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f"{gripper_group}: gripper action server not available.")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = math.radians(degrees)
        goal.command.max_effort = float(self.gripper_max_effort)

        self.get_logger().info(f"→ {gripper_group}: {degrees}°")
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
        end_pos = self._joint_state.get(joint, start_pos)
        moved = (start_pos is not None and end_pos is not None
                 and abs(end_pos - start_pos) > 0.02)
        if r.stalled and moved:
            self.get_logger().info(f"{gripper_group}: stalled on object (pos={r.position:.3f}).")
            return True
        self.get_logger().error(
            f"{gripper_group}: did not actuate (stalled={r.stalled}, "
            f"reached={r.reached_goal}, start={start_pos}, end={end_pos}). "
            f"Check gripper hardware/driver, or run with skip_grippers:=true."
        )
        return False

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

        return self._wait_for_table(table_id, target_mm, target_deg)

    def _wait_for_table(self, table_id, target_mm, target_deg) -> bool:
        lin_joint, rot_joint = TABLE_JOINTS[table_id]
        target_m = target_mm / 1000.0
        target_rad = math.radians(target_deg)
        tol_m = self.table_tol_mm / 1000.0
        tol_rad = math.radians(self.table_tol_deg)

        deadline = time.time() + self.table_timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            lin = self._joint_state.get(lin_joint)
            rot = self._joint_state.get(rot_joint)
            if lin is None or rot is None:
                continue
            if abs(lin - target_m) <= tol_m and abs(rot - target_rad) <= tol_rad:
                if self.motor_settle_s > 0:
                    time.sleep(self.motor_settle_s)
                return True
        self.get_logger().error(
            f"{table_id}: timed out waiting to reach target "
            f"(last lin={self._joint_state.get(lin_joint)}, rot={self._joint_state.get(rot_joint)})."
        )
        return False

    def move_joints_and_table(self, group_name, joint_names, degrees_list,
                               table_id, target_mm, target_deg) -> bool:
        # Fire table command so both move concurrently; arm executes while table travels
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
        table_future = self._table_client.call_async(req)

        arm_ok = self.move_joints(group_name, joint_names, degrees_list)

        result = self._spin_until(table_future, timeout_sec=10.0)
        if result is None:
            self.get_logger().error(f"{table_id}: service call timed out.")
            return False
        if not result.success:
            self.get_logger().error(f"{table_id}: service rejected — {result.message}")
            return False

        return arm_ok and self._wait_for_table(table_id, target_mm, target_deg)

    def _switch_controllers(self, activate, deactivate) -> bool:
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("switch_controller service not available.")
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(seconds=2.0).to_msg()
        result = self._spin_until(self._switch_client.call_async(req), timeout_sec=8.0)
        if result is None or not result.ok:
            self.get_logger().error(
                f"switch_controller failed (activate={activate}, deactivate={deactivate}).")
            return False
        return True

    def compliant_pull_arm2(self) -> bool:
        """Pull the curtain under Cartesian-velocity (twist) control for arm_2.

        Mirrors close_curtain compliant_pull() but targets arm_2. Switches from
        its trajectory controller to the twist controller, streams a constant
        tool-frame velocity, and stops on: target joint_1 angle reached, effort
        watchdog, wrong-direction sanity check, or timeout. Always zeroes the
        twist and restores the trajectory controller even on error.
        """
        joint1 = ARM_JOINTS["arm_2"][0]  # t1_a2_joint_1
        target_rad = math.radians(self.pull_target_deg)
        tol_rad = math.radians(self.pull_tol_deg)

        start = self._joint_state.get(joint1)
        if start is None:
            self.get_logger().error("compliant_pull_arm2: no joint_1 state yet.")
            return False
        expected_sign = 1.0 if target_rad > start else -1.0

        if not self._switch_controllers([self.twist_controller], [self.arm2_traj_controller]):
            return False

        t = self.pull_twist
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = t[0], t[1], t[2]
        twist.angular.x, twist.angular.y, twist.angular.z = t[3], t[4], t[5]
        zero = Twist()

        period = 1.0 / self.pull_rate_hz
        deadline = time.time() + self.pull_timeout_s
        ok = False
        reason = "timeout"
        try:
            while time.time() < deadline:
                self._twist_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=period)
                pos = self._joint_state.get(joint1, start)

                if abs(pos - target_rad) <= tol_rad:
                    ok, reason = True, "target reached"
                    break
                if (pos - start) * expected_sign < -math.radians(5.0):
                    reason = "wrong direction — check pull_twist sign/axis"
                    break
                eff = max((abs(self._joint_effort.get(j, 0.0))
                           for j in ARM_JOINTS["arm_2"]), default=0.0)
                if eff >= self.pull_effort_limit:
                    if abs(pos - target_rad) <= math.radians(15.0):
                        ok, reason = True, f"effort stop near target ({eff:.1f} Nm)"
                    else:
                        reason = f"effort limit hit mid-pull ({eff:.1f} Nm)"
                    break
        finally:
            for _ in range(5):
                self._twist_pub.publish(zero)
                time.sleep(0.02)
            if not self._switch_controllers(
                    [self.arm2_traj_controller], [self.twist_controller]):
                self.get_logger().error(
                    "compliant_pull_arm2: FAILED to restore arm_2_controller — "
                    "arm_2 may be left without an active position controller!")
                return False

        final_deg = math.degrees(self._joint_state.get(joint1, start))
        self.get_logger().info(
            f"compliant_pull_arm2 ended: {reason} (joint_1={final_deg:.1f} deg).")
        return ok

    def run_sequence(self) -> bool:
        grip = self.gripper_grip_deg
        opn = self.gripper_open_deg
        a1 = ARM_JOINTS["arm_1"]
        a2 = ARM_JOINTS["arm_2"]

        steps = [
            # Ensure grippers open before moving
            ("1  gripper1 open",                           lambda: self.move_gripper("gripper_1", opn)),
            ("2  gripper2 open",                           lambda: self.move_gripper("gripper_2", opn)),
            # Position table before arm1 approaches
            ("3  table1 -> 1268mm / 0deg",                 lambda: self.move_table("table1", 1268.0, 0.0)),
            # arm1 grabs and sweeps curtain open first
            ("4  arm1 pre-approach",                       lambda: self.move_joints("arm_1", a1, [55, 0, -90, 0, 0, 0])),
            ("5  arm1 approach",                           lambda: self.move_joints("arm_1", a1, [55, 50, -30, 0, 0, 0])),
            ("6  gripper1 grip",                           lambda: self.move_gripper("gripper_1", grip)),
            ("7  arm1 open-curtain swing",                 lambda: self.move_joints("arm_1", a1, [115, 50, -30, 0, 0, 0])),
            ("8  table1 -> 1352mm",                        lambda: self.move_table("table1", 1352.0, 0.0)),
            ("9  gripper1 open",                           lambda: self.move_gripper("gripper_1", opn)),
            ("10 arm1 retreat + table1 -> 1268mm",         lambda: self.move_joints_and_table("arm_1", a1, [115, 0, 0, 0, 0, 0], "table1", 1268.0, 0.0)),
            ("11 arm1 home",                               lambda: self.move_joints("arm_1", a1, [0, 150, 150, 0, 0, 0])),
            # Confirm table position before arm2
            ("12 table1 -> 1268mm / 0deg",                 lambda: self.move_table("table1", 1268.0, 0.0)),
            # arm2 grabs and sweeps curtain open
            ("13 arm2 pre-approach",                       lambda: self.move_joints("arm_2", a2, [-60, 0, -90, 0, 0, 0])),
            ("14 arm2 approach",                           lambda: self.move_joints("arm_2", a2, [-60, 50, -40, 0, 0, 0])),
            ("15 gripper2 grip",                           lambda: self.move_gripper("gripper_2", grip)),
            ("16 arm2 open-curtain swing",                 lambda: self.compliant_pull_arm2() if self.use_compliant_pull
                                                                   else self.move_joints("arm_2", a2, [-120, 50, -40, 0, 0, 0])),
            ("17 table1 -> 1110mm",                        lambda: self.move_table("table1", 1110.0, 0.0)),
            ("18 gripper2 open",                           lambda: self.move_gripper("gripper_2", opn)),
            ("20 arm2 lift",                               lambda: self.move_joints("arm_2", a2, [-120, 150, 150, 0, 0, 0])),
            ("21 arm2 home",                               lambda: self.move_joints("arm_2", a2, [0, 150, 150, 0, 0, 0])),
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
        node = OpenCurtainRunner()
        if node.startup_delay_s > 0:
            node.get_logger().info(f"Startup delay {node.startup_delay_s}s...")
            end = time.time() + node.startup_delay_s
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.run_sequence()
        if ok:
            node.get_logger().info("Open-curtain sequence complete.")
        else:
            node.get_logger().error("Open-curtain sequence did not complete.")
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
