#!/usr/bin/env python3
"""One-shot runner for the take-bottle choreography.

Sequence:
  1  table2 → 880 mm / 90°
  2  arm4 approach 1
  3  arm4 approach 2 + gripper4 grip
  4  arm4 lift
  7  arm4 carry
  7b arm3 pre-approach
  8  arm3 approach + gripper3 grip
  9  gripper4 release + arm4 retreat
  10 arm3 move
  11 table1 → home
  12 arm2 approach 1 + 2 + gripper2 grip
  13 gripper3 loosen + arm3 retreat
  14 arm2 place + gripper2 release
  15 all arms home
  16 table1 + table2 home
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


ARM_JOINTS = {
    "arm_2": [f"t1_a2_joint_{i}" for i in range(1, 7)],
    "arm_3": [f"t2_a1_joint_{i}" for i in range(1, 7)],
    "arm_4": [f"t2_a2_joint_{i}" for i in range(1, 7)],
}
GRIPPER_JOINT = {
    "gripper_2": "t1_a2_right_finger_bottom_joint",
    "gripper_3": "t2_a1_right_finger_bottom_joint",
    "gripper_4": "t2_a2_right_finger_bottom_joint",
}
TABLE_JOINTS = {
    "table1": ("t1_linear_joint", "t1_rotation_joint"),
    "table2": ("t2_linear_joint", "t2_rotation_joint"),
}


class TakeBottleRunner(Node):
    def __init__(self):
        super().__init__("take_bottle_runner")

        self.gripper_grip_deg = self.declare_parameter("gripper_grip_deg", 49.0).value
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

        self.table_cmd = {
            "table1": {"mm": 0.0, "deg": 0.0},
            "table2": {"mm": 0.0, "deg": 0.0},
        }

        self._joint_state = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

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

        self.get_logger().info("Connected. Take-bottle runner ready.")

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_state[name] = pos

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
        cur = self.table_cmd[table_id]
        delta_mm = target_mm - cur["mm"]
        delta_deg = target_deg - cur["deg"]

        if abs(delta_mm) < 1e-6 and abs(delta_deg) < 1e-6:
            self.get_logger().info(f"→ {table_id}: already at target ({target_mm} mm, {target_deg} deg).")
            return True

        req = MovingTable.Request()
        req.table_id = table_id
        req.distance_mm = float(delta_mm)
        req.angle_deg = float(delta_deg)
        req.linear_speed = int(self.linear_speed)
        req.rotate_speed = int(self.rotate_speed)
        req.operation_type = 2

        self.get_logger().info(
            f"→ {table_id}: target {target_mm} mm / {target_deg} deg "
            f"(delta {delta_mm:+.1f} mm, {delta_deg:+.1f} deg)"
        )
        result = self._spin_until(self._table_client.call_async(req), timeout_sec=10.0)
        if result is None:
            self.get_logger().error(f"{table_id}: service call timed out.")
            return False
        if not result.success:
            self.get_logger().error(f"{table_id}: service rejected — {result.message}")
            return False

        if not self._wait_for_table(table_id, target_mm, target_deg):
            return False

        self.table_cmd[table_id] = {"mm": target_mm, "deg": target_deg}
        return True

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

    def run_sequence(self) -> bool:
        grip = self.gripper_grip_deg
        opn = self.gripper_open_deg

        steps = [
            ("1  table2 -> 880mm / 90deg",  lambda: self.move_table("table2", 880.0, 90.0)),
            ("2  arm4 approach 1",           lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -43, -56, -90, 76, 90])),
            ("3  arm4 approach 2",           lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, 5, 2, -92, 62, 90])),
            ("3  gripper4 grip",             lambda: self.move_gripper("gripper_4", grip)),
            ("4  arm4 lift",                 lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -75, -130, -90, 20, 90])),
            ("7  arm4 carry",                lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [0, -15, -90, -90, 0, 90])),
            ("7b arm3 pre-approach",         lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [-3, -16, -61, 95, -46, 90])),
            ("8  arm3 approach",             lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [-3, 17, -28, 90, -45, 90])),
            ("8  gripper3 grip",             lambda: self.move_gripper("gripper_3", grip)),
            ("9  gripper4 release",          lambda: self.move_gripper("gripper_4", 20.0)),
            ("9  arm4 retreat",              lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [0, -60, -115, -90, 30, 2])),
            ("10 arm3 move",                 lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [98, 66, 20, 108, -45, 82])),
            ("11 table1 -> home",            lambda: self.move_table("table1", 0.0, 0.0)),
            ("12a arm2 approach 1",          lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [28, -19, -120, 90, 9, -90])),
            ("12b arm2 approach 2",          lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [28, 21, -86, 90, 16, -90])),
            ("12  gripper2 grip",            lambda: self.move_gripper("gripper_2", grip)),
            ("13  gripper3 loosen",          lambda: self.move_gripper("gripper_3", opn)),
            ("13  arm3 retreat",             lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [63, 93, 130, 148, -119, 117])),
            ("14  arm2 place",               lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [0, -45, -99, 90, 55, -90])),
            ("14  gripper2 release",         lambda: self.move_gripper("gripper_2", opn)),
            ("15  arm2 home",                lambda: self.move_joints("arm_2", ARM_JOINTS["arm_2"], [90, -150, -150, 0, 0, 0])),
            ("15  arm3 home",                lambda: self.move_joints("arm_3", ARM_JOINTS["arm_3"], [90, -150, -150, 0, 0, 0])),
            ("15  arm4 home",                lambda: self.move_joints("arm_4", ARM_JOINTS["arm_4"], [90, -150, -150, 0, 0, 0])),
            ("16  table1 -> home",           lambda: self.move_table("table1", 0.0, 0.0)),
            ("16  table2 -> home",           lambda: self.move_table("table2", 0.0, 0.0)),
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
        node = TakeBottleRunner()
        if node.startup_delay_s > 0:
            node.get_logger().info(f"Startup delay {node.startup_delay_s}s...")
            end = time.time() + node.startup_delay_s
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.run_sequence()
        if ok:
            node.get_logger().info("✅ Take-bottle sequence complete.")
        else:
            node.get_logger().error("❌ Take-bottle sequence did not complete.")
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
