#!/usr/bin/env python3
"""One-shot runner for the close-curtain choreography.

Sequence:
  0a    gripper1 open (0 deg)
  0b    gripper2 open (0 deg)
  1     table1 → 1268 mm / 0 deg
  2a/10a arm2 + arm1 joint_1 only (yaw first, joints 2-6 held), IN PARALLEL
  2/10  arm2 + arm1 pre-approach, IN PARALLEL   [-105,0,-90,0,0,0] / [106,-39,-112,24,-35,-29]
  3/11  arm2 + arm1 approach, IN PARALLEL       [-105,35,-76,0,0,0] / [106,30,-60,0,0,0]
  4/12  gripper2 + gripper1 grip (40 deg), IN PARALLEL
  5/13  arm2 swing + arm1 close curtain swing, IN PARALLEL   [-60,35,-76,0,0,0] / [35,30,-60,0,0,0]
  7   gripper2 open (20 deg)
  15  gripper1 open (20 deg)
  *   arm2 + arm1 all joints zero, IN PARALLEL   [0,0,0,0,0,0] / [0,0,0,0,0,0]
  9/16 arm2 + arm1 home, IN PARALLEL   [0,150,150,0,0,0] / [0,150,150,0,0,0]
  17  table1 rotate 0 deg

!! PARALLEL ARM MOTION — READ THIS !!
Steps 2-5 (arm_2) and 10-13 (arm_1) run concurrently, joint-1-first: each arm
yaws joint_1 to its pre-approach angle while joints 2-6 hold their current
position, THEN the remaining joints move to the full pre-approach pose.
NOTE: the joint_1-only sub-step for arm_2 was previously removed because it
swept arm_2 into the power cable, then explicitly restored — re-verify
clearance around the power cable before running this on hardware. This
mirrors the pattern in run_take_bottle.py: MoveIt's
`move_action` takes one goal at a time, so each arm is planned via
`plan_kinematic_path` (planning only) and the resulting trajectory is sent
straight to that arm's `follow_joint_trajectory` controller; both controllers
run at the same time. Each plan is collision-checked against the scene as it
is BEFORE the pair starts — arm_1 and arm_2 are NOT checked against each other
while they execute together, and they share gantry 1's workspace, so this is
the riskiest part of the sequence. Run with sequential_arms:=true to fall back
to one-arm-at-a-time execution through MoveIt.

Steps 7 (gripper2 open) and 15 (gripper1 open) still run in series, in their
original relative order, right after the parallel block. Steps 6, 8, and 14
(table1 rotates to -20deg/0deg/10deg — all no-ops once 6/14 were removed,
since nothing else moves table1 away from 0 in between) were removed on
request; step 17 is the only remaining table1 move, at the very end. Step 9
(arm2 home) was moved out of that series and merged with step 16 (arm1 home)
into a single parallel stage, run right after step 15 and before step 17.

use_compliant_pull:=true takes step 13 off the parallel path: it needs the
twist controller instead of a planned trajectory, so arm_2's swing (step 5)
runs first, then arm_1's compliant pull runs alone.

a2_pre/a2_approach and a1_pre/a1_approach in run_sequence() are plain
hand-tuned joint degrees, same as the rest of the file. A runtime FK/IK
z-offset attempt (arm_2 end effector -10cm, arm_1 +10cm) was tried here and
reverted after compute_ik kept returning NO_IK_SOLUTION (-31) — hand-tune the
degrees directly instead.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.action import ActionClient

from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from control_msgs.action import GripperCommand, FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController

from moving_table_interfaces.srv import MovingTable

# operation_type: move to an ABSOLUTE table position. The controller reads the
# motor encoder server-side and computes the move itself, so the sequence works
# from any starting position without homing and without depending on /joint_states.
OP_GOTO_ABS = 97


ARM_JOINTS = {
    "arm_1": [f"t1_a1_joint_{i}" for i in range(1, 7)],
    "arm_2": [f"t1_a2_joint_{i}" for i in range(1, 7)],
}
# Planning group -> the FollowJointTrajectory controller that executes it.
# Used for the parallel steps; move_action takes one goal at a time.
ARM_CONTROLLER = {
    "arm_1": "arm_1_controller",
    "arm_2": "arm_2_controller",
}
GRIPPER_JOINT = {
    "gripper_1": "t1_a1_right_finger_bottom_joint",
    "gripper_2": "t1_a2_right_finger_bottom_joint",
}
TABLE_JOINTS = {
    "table1": ("t1_linear_joint", "t1_rotation_joint"),
}


class CloseCurtainRunner(Node):
    def __init__(self):
        super().__init__("close_curtain_runner")

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
        # Escape hatch: run steps 2-5/10-13 one arm at a time through MoveIt
        # instead of the parallel plan-then-fire path. Slower, but arm-vs-arm
        # collisions between arm_1 and arm_2 are then impossible.
        self.sequential_arms = self.declare_parameter("sequential_arms", False).value
        self.arm_exec_timeout_s = self.declare_parameter("arm_exec_timeout_s", 120.0).value

        # --- Compliant curtain pull (step 11) ---
        # Opt-in: when False, step 11 stays the original joint-space swing.
        # When True, arm_1 is switched to the Cartesian-velocity (twist) controller
        # so it yields to the curtain under load instead of building up position-
        # error torque that trips the Kinova into a protective fault.
        self.use_compliant_pull = self.declare_parameter("use_compliant_pull", False).value
        self.pull_target_deg = self.declare_parameter("pull_target_deg", 50.0).value
        self.pull_tol_deg = self.declare_parameter("pull_tol_deg", 3.0).value
        self.pull_timeout_s = self.declare_parameter("pull_timeout_s", 15.0).value
        self.pull_rate_hz = self.declare_parameter("pull_rate_hz", 40.0).value
        self.pull_effort_limit = self.declare_parameter("pull_effort_limit", 8.0).value
        # Tool-frame twist [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z] in m/s, rad/s.
        # MUST be tuned on hardware so joint_1 swings toward pull_target_deg.
        self.pull_twist = self.declare_parameter(
            "pull_twist", [0.0, -0.05, 0.0, 0.0, 0.0, 0.0]).value
        self.twist_controller = self.declare_parameter(
            "twist_controller", "t1_a1_twist_controller").value
        self.arm1_traj_controller = self.declare_parameter(
            "arm1_traj_controller", "arm_1_controller").value

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

        # Planning-only service, for the parallel arm_1/arm_2 steps.
        self._plan_client = self.create_client(GetMotionPlan, "plan_kinematic_path")
        self.get_logger().info("Waiting for plan_kinematic_path service...")
        if not self._plan_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("plan_kinematic_path service not available.")

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

        self._gripper_clients = {
            g: ActionClient(self, GripperCommand, f"/{g}_controller/gripper_cmd")
            for g in GRIPPER_JOINT
        }

        self.get_logger().info("Connected. Close-curtain runner ready.")

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_state[name] = pos
        # effort may be empty on some publishers; zip stops at the shorter list.
        for name, eff in zip(msg.name, msg.effort):
            self._joint_effort[name] = eff

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

    def _joint1_first_targets(self, joint_names, final_degrees):
        """Split a full-arm target into (joint_1-only move, full move).

        The first list keeps joints 2-6 at their CURRENT position and only
        moves joint_1 to its final angle, so the arm yaws into direction
        before extending/lowering. The second list is the original target,
        unchanged.
        """
        current_degrees = []
        for j in joint_names[1:]:
            pos = self._joint_state.get(j)
            current_degrees.append(math.degrees(pos) if pos is not None else 0.0)
        joint1_step = [final_degrees[0]] + current_degrees
        return joint1_step, list(final_degrees)

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
        goal.request.goal_constraints.append(
            self._joint_goal_constraints(joint_names, degrees_list)
        )

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

    def move_arms_parallel(self, targets) -> bool:
        """Move several arms to joint targets at the SAME time.

        `targets` is a list of (group_name, joint_names, degrees_list).

        move_action accepts one goal at a time (a second goal preempts the
        first), so this plans each arm through `plan_kinematic_path` (planning
        only, no motion) and then sends the trajectories straight to the
        per-arm controllers. If any plan fails nothing has moved yet and we
        abort.

        Collision caveat: each plan is checked against the scene as it is
        BEFORE the batch starts; arms executing concurrently are NOT checked
        against each other. arm_1/arm_2 share gantry 1's workspace — run with
        sequential_arms:=true if their poses put them near each other.
        """
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

        # Fire every trajectory first (controllers are independent, one per
        # arm, so these run concurrently), then wait for all of them.
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
                self.get_logger().error(
                    "Some arms may still be executing — issue a stop if needed."
                )
                return False
            handles.append((group_name, goal_handle))

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

    def _run_parallel_or_sequential(self, targets) -> bool:
        if self.sequential_arms:
            for group, joints, degs in targets:
                if not self.move_joints(group, joints, degs):
                    return False
            return True
        return self.move_arms_parallel(targets)

    def move_grippers_parallel(self, commands) -> bool:
        """Fire GripperCommand goals for several grippers at the same time."""
        if self.skip_grippers:
            for g, _ in commands:
                self.get_logger().warn(f"{g}: skip_grippers set — skipping.")
            return True

        handles = []
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
            goal_handle = self._spin_until(client.send_goal_async(goal))
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{gripper_group}: goal rejected.")
                return False
            handles.append((gripper_group, joint, start_pos, goal_handle))

        ok = True
        for gripper_group, joint, start_pos, goal_handle in handles:
            result = self._spin_until(goal_handle.get_result_async())
            if result is None:
                self.get_logger().error(f"{gripper_group}: no result returned.")
                ok = False
                continue
            r = result.result
            if r.reached_goal:
                continue
            end_pos = self._joint_state.get(joint, start_pos)
            moved = (start_pos is not None and end_pos is not None
                     and abs(end_pos - start_pos) > 0.02)
            if r.stalled and moved:
                self.get_logger().info(f"{gripper_group}: stalled on object (pos={r.position:.3f}).")
                continue
            self.get_logger().error(
                f"{gripper_group}: did not actuate (stalled={r.stalled}, "
                f"reached={r.reached_goal}, start={start_pos}, end={end_pos})."
            )
            ok = False
        return ok

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

    def compliant_pull(self) -> bool:
        """Pull the curtain under Cartesian-velocity (twist) control.

        Switches arm_1 from its trajectory controller to the twist controller,
        streams a constant tool-frame velocity, and stops on: target joint_1
        angle reached, effort watchdog, wrong-direction sanity check, or timeout.
        Always zeroes the twist and restores the trajectory controller, even on
        error, so the arm can never be left under live velocity command.
        """
        joint1 = ARM_JOINTS["arm_1"][0]  # t1_a1_joint_1
        target_rad = math.radians(self.pull_target_deg)
        tol_rad = math.radians(self.pull_tol_deg)

        start = self._joint_state.get(joint1)
        if start is None:
            self.get_logger().error("compliant_pull: no joint_1 state yet.")
            return False
        expected_sign = 1.0 if target_rad > start else -1.0

        if not self._switch_controllers([self.twist_controller], [self.arm1_traj_controller]):
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
                # joint_1 must move toward the target, not away from it.
                if (pos - start) * expected_sign < -math.radians(5.0):
                    reason = "wrong direction — check pull_twist sign/axis"
                    break
                eff = max((abs(self._joint_effort.get(j, 0.0))
                           for j in ARM_JOINTS["arm_1"]), default=0.0)
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
                    [self.arm1_traj_controller], [self.twist_controller]):
                self.get_logger().error(
                    "compliant_pull: FAILED to restore arm_1_controller — "
                    "arm_1 may be left without an active position controller!")
                return False

        final_deg = math.degrees(self._joint_state.get(joint1, start))
        self.get_logger().info(
            f"compliant_pull ended: {reason} (joint_1={final_deg:.1f} deg).")
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

    def _send_table(self, table_id, target_mm, target_deg) -> bool:
        # Send the ABSOLUTE target. The controller reads the motor encoder
        # server-side and computes the move — no /joint_states delta on the client,
        # so this is correct from any starting position. The service returns as
        # soon as the move is accepted; the motion runs in a background thread.
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
        return True

    def move_table(self, table_id, target_mm, target_deg) -> bool:
        if not self._send_table(table_id, target_mm, target_deg):
            return False
        return self._wait_for_table(table_id, target_mm, target_deg)

    def arm_swing_with_table_rotate(self, group, joints, degrees_list,
                                    table_id, table_mm, table_deg) -> bool:
        """Run an arm swing and a table move concurrently.

        The table service returns immediately (motion runs server-side in a
        background thread), so we fire the table command first, then plan and
        execute the arm swing while the table rotates, then wait for the table
        to settle before reporting success.
        """
        if not self._send_table(table_id, table_mm, table_deg):
            return False
        if not self.move_joints(group, joints, degrees_list):
            return False
        return self._wait_for_table(table_id, table_mm, table_deg)

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

    def _swing_stage(self, a2, a1, a2_swing, a1_swing) -> bool:
        """Step 5/13: arm_2 swing + arm_1 close-curtain swing.

        Parallel by default. use_compliant_pull:=true takes arm_1 off the
        planned-trajectory path (it needs the twist controller instead), so in
        that mode arm_2's swing runs first, then arm_1's compliant pull runs
        alone.
        """
        if self.use_compliant_pull:
            if not self.move_joints("arm_2", a2, a2_swing):
                return False
            return self.compliant_pull()
        return self._run_parallel_or_sequential([
            ("arm_2", a2, a2_swing),
            ("arm_1", a1, a1_swing),
        ])

    def run_sequence(self) -> bool:
        grip = self.gripper_grip_deg
        opn = self.gripper_open_deg
        a1 = ARM_JOINTS["arm_1"]
        a2 = ARM_JOINTS["arm_2"]

        a2_pre = [-105, 0, -90, 0, 0, 0]
        a1_pre = [106, -39, -112, 24, -35, -29]
        a2_approach = [-105, 35, -76, 0, 0, 0]
        a1_approach = [106, 30, -60, 0, 0, 0]
        a2_swing = [-60, 35, -76, 0, 0, 0]
        a1_swing = [35, 30, -60, 0, 0, 0]

        steps = [
            ("0a    gripper1 open",              lambda: self.move_gripper("gripper_1", opn)),
            ("0b    gripper2 open",               lambda: self.move_gripper("gripper_2", opn)),
            ("1     table1 -> 1268mm / 0deg",     lambda: self.move_table("table1", 1268.0, 0.0)),
            ("2a/10a arm2+arm1 joint_1 first, parallel",
                lambda: self._run_parallel_or_sequential([
                    ("arm_2", a2, self._joint1_first_targets(a2, a2_pre)[0]),
                    ("arm_1", a1, self._joint1_first_targets(a1, a1_pre)[0]),
                ])),
            ("2/10b arm2+arm1 pre-approach, parallel",
                lambda: self._run_parallel_or_sequential([
                    ("arm_2", a2, a2_pre),
                    ("arm_1", a1, a1_pre),
                ])),
            ("3/11  arm2+arm1 approach, parallel",
                lambda: self._run_parallel_or_sequential([
                    ("arm_2", a2, a2_approach),
                    ("arm_1", a1, a1_approach),
                ])),
            ("4/12  gripper2+gripper1 grip, parallel",
                lambda: self.move_grippers_parallel([
                    ("gripper_2", grip),
                    ("gripper_1", grip),
                ])),
            ("5/13  arm2 swing + arm1 close curtain swing",
                lambda: self._swing_stage(a2, a1, a2_swing, a1_swing)),
            ("7   gripper2 open (20deg)",        lambda: self.move_gripper("gripper_2", 20.0)),
            ("15  gripper1 open (20deg)",        lambda: self.move_gripper("gripper_1", 20.0)),
            ("*   arm2+arm1 all joints zero, parallel",
                lambda: self._run_parallel_or_sequential([
                    ("arm_2", a2, [0, 0, 0, 0, 0, 0]),
                    ("arm_1", a1, [0, 0, 0, 0, 0, 0]),
                ])),
            ("9/16 arm2+arm1 home, parallel",
                lambda: self._run_parallel_or_sequential([
                    ("arm_2", a2, [0, 150, 150, 0, 0, 0]),
                    ("arm_1", a1, [0, 150, 150, 0, 0, 0]),
                ])),
            ("17  table1 rotate 0deg",           lambda: self.move_table("table1", 1268.0, 0.0)),
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
        node = CloseCurtainRunner()
        if node.startup_delay_s > 0:
            node.get_logger().info(f"Startup delay {node.startup_delay_s}s...")
            end = time.time() + node.startup_delay_s
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.run_sequence()
        if ok:
            node.get_logger().info("Close-curtain sequence complete.")
        else:
            node.get_logger().error("Close-curtain sequence did not complete.")
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
