#!/usr/bin/env python3
"""One-shot runner for the open-curtain choreography.

Sequence:
  1       gripper1 open
  2       gripper2 open
  3       table1 → 1268 mm / 0 deg
  4a+13a  dual joint1 rotate (arm1 + arm2, combined; joints 2-6 held at home)
  4b+13b  dual pre-approach (arm1 + arm2, combined; remaining joints move)
  5+14    dual approach (arm1 + arm2, combined)
  6+15    dual grip (gripper1 + gripper2, fired concurrently)
  7+16    dual open-curtain swing (arm1 + arm2, combined) —
          if use_compliant_pull: arm1 swings (joint-space), then arm2 pulls via
          Cartesian twist control instead, run sequentially (can't be combined
          with a joint-space goal)
  8       table1 → 1352 mm
  9       gripper1 open
  10      arm1 retreat + table1 → 1268 mm (parallel)
  12      table1 → 1268 mm / 0 deg
  17      table1 → 1170 mm
  18      gripper2 open
  20      arm2 lift
  11+21   dual arms home (arm1 + arm2, combined; arm1 goes home from its
          step-10 retreat pose, arm2 from its lift pose)

arm1 and arm2 are both mounted on gantry_1/table1, and move_action only runs
one goal at a time, so steps 4-7 and 13-16 from the original sequential
choreography are combined into single goals on the "arm_1_and_arm_2" planning
group (see move_dual_arms()) rather than sent as two separate, mutually-
preempting goals. That group deliberately excludes the gantry_1 joints, which
have no ros2_control controller and would make execution fail outright.
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

    def move_dual_arms(self, a1_joint_names, a1_degrees, a2_joint_names, a2_degrees) -> bool:
        """Move arm_1 and arm_2 in one MoveGroup goal so they execute simultaneously.

        arm_1 and arm_2 share the same move_action server, which only runs one
        goal at a time — sending two separate arm goals back-to-back would have
        the second preempt the first. Planning both arms as one goal on the
        "arm_1_and_arm_2" group (arm joints only, no gantry_1) lets MoveIt
        dispatch the resulting trajectory to arm_1_controller and arm_2_controller
        together. It deliberately excludes t1_linear_joint/t1_rotation_joint:
        those have no ros2_control FollowJointTrajectory controller (the gantry
        is driven separately over Modbus), so a goal that includes them can
        never be matched to a controller and execution fails outright.
        """
        joint_names = list(a1_joint_names) + list(a2_joint_names)
        radians_list = (
            [math.radians(d) for d in a1_degrees]
            + [math.radians(d) for d in a2_degrees]
        )

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm_1_and_arm_2"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = float(self.planning_time)
        goal.request.max_velocity_scaling_factor = float(self.vel_scale)
        goal.request.max_acceleration_scaling_factor = float(self.acc_scale)

        constraints = Constraints()
        constraints.name = "goal"
        for name, rad in zip(joint_names, radians_list):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = rad
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(
            f"→ arm_1_and_arm_2 (dual): arm1={a1_degrees} arm2={a2_degrees}")
        if self.arm_settle_s > 0:
            time.sleep(self.arm_settle_s)
        send_future = self._move_client.send_goal_async(goal)
        goal_handle = self._spin_until(send_future)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"arm_1_and_arm_2: dual goal rejected by move_action.")
            return False

        result = self._spin_until(goal_handle.get_result_async())
        if result is None:
            self.get_logger().error(f"arm_1_and_arm_2: no result returned.")
            return False
        code = result.result.error_code.val
        if code != 1:
            self.get_logger().error(
                f"arm_1_and_arm_2: dual planning/execution failed (code {code}).")
            return False
        return True

    def _eval_gripper_result(self, gripper_group, joint, start_pos, result) -> bool:
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

    def move_grippers_parallel(self, commands) -> bool:
        """Send several gripper goals concurrently, then wait for all results.

        commands: list of (gripper_group, degrees) pairs.
        """
        if self.skip_grippers:
            self.get_logger().warn("skip_grippers set — skipping parallel grip.")
            return True

        pending = []  # (gripper_group, joint, start_pos, goal_handle_future)
        for gripper_group, degrees in commands:
            joint = GRIPPER_JOINT[gripper_group]
            start_pos = self._joint_state.get(joint)
            if start_pos is not None and abs(start_pos - math.radians(degrees)) < 0.05:
                self.get_logger().info(f"→ {gripper_group}: already at {degrees}° — skipping.")
                continue
            client = self._gripper_clients[gripper_group]
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error(f"{gripper_group}: gripper action server not available.")
                return False
            goal = GripperCommand.Goal()
            goal.command.position = math.radians(degrees)
            goal.command.max_effort = float(self.gripper_max_effort)
            self.get_logger().info(f"→ {gripper_group}: {degrees}° (parallel)")
            pending.append((gripper_group, joint, start_pos, client.send_goal_async(goal)))

        ok = True
        result_futures = []
        for gripper_group, joint, start_pos, send_future in pending:
            goal_handle = self._spin_until(send_future)
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{gripper_group}: goal rejected.")
                ok = False
                continue
            result_futures.append((gripper_group, joint, start_pos, goal_handle.get_result_async()))

        for gripper_group, joint, start_pos, result_future in result_futures:
            result = self._spin_until(result_future)
            if result is None:
                self.get_logger().error(f"{gripper_group}: no result returned.")
                ok = False
                continue
            if not self._eval_gripper_result(gripper_group, joint, start_pos, result):
                ok = False
        return ok

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

        return self._eval_gripper_result(gripper_group, joint, start_pos, result)

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
            # Position table before arms approach
            ("3  table1 -> 1268mm / 0deg",                 lambda: self.move_table("table1", 1268.0, 0.0)),
            # arm1 + arm2 approach and grip the curtain together, one combined
            # MoveGroup goal per pose so both arms execute simultaneously.
            # joint_1 rotates into position first (arms start from home, joints
            # 2-6 held at their home values), then the remaining joints move.
            ("4a+13a dual joint1 rotate",                  lambda: self.move_dual_arms(
                a1, [55, 150, 150, 0, 0, 0], a2, [-60, 150, 150, 0, 0, 0])),
            ("4b+13b dual pre-approach",                   lambda: self.move_dual_arms(
                a1, [55, 0, -90, 0, 0, 0], a2, [-60, 0, -90, 0, 0, 0])),
            ("5+14 dual approach",                         lambda: self.move_dual_arms(
                a1, [55, 50, -30, 0, 0, 0], a2, [-60, 50, -40, 0, 0, 0])),
            ("6+15 dual grip",                              lambda: self.move_grippers_parallel(
                [("gripper_1", grip), ("gripper_2", grip)])),
        ]

        if self.use_compliant_pull:
            # compliant_pull_arm2 drives arm_2 via Cartesian twist control, not a
            # joint-space MoveGroup goal, so it cannot be folded into a combined
            # dual-arm goal — arm1's swing and arm2's pull run sequentially here.
            steps.append(("7  arm1 open-curtain swing", lambda: self.move_joints("arm_1", a1, [115, 50, -30, 0, 0, 0])))
            steps.append(("16 arm2 open-curtain swing (compliant pull)", lambda: self.compliant_pull_arm2()))
        else:
            steps.append(("7+16 dual open-curtain swing", lambda: self.move_dual_arms(
                a1, [115, 50, -30, 0, 0, 0], a2, [-120, 50, -40, 0, 0, 0])))

        steps += [
            ("8  table1 -> 1352mm",                        lambda: self.move_table("table1", 1352.0, 0.0)),
            ("9  gripper1 open",                           lambda: self.move_gripper("gripper_1", opn)),
            ("10 arm1 retreat + table1 -> 1268mm",         lambda: self.move_joints_and_table("arm_1", a1, [115, 0, 0, 0, 0, 0], "table1", 1268.0, 0.0)),
            ("12 table1 -> 1268mm / 0deg",                 lambda: self.move_table("table1", 1268.0, 0.0)),
            ("17 table1 -> 1170mm",                        lambda: self.move_table("table1", 1170.0, 0.0)),
            ("18 gripper2 open",                           lambda: self.move_gripper("gripper_2", opn)),
            ("20 arm2 lift",                               lambda: self.move_joints("arm_2", a2, [-120, 150, 150, 0, 0, 0])),
            # arm1 (still retreated at [115,0,0,0,0,0] since step 10) and arm2
            # both go home together, one combined MoveGroup goal
            ("11+21 dual arms home",                       lambda: self.move_dual_arms(
                a1, [0, 150, 150, 0, 0, 0], a2, [0, 150, 150, 0, 0, 0])),
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
