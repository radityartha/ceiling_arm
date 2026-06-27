#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import rclpy.duration
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

# NEW IMPORT: We need this to read the robot's current state
from sensor_msgs.msg import JointState


# -----------------------------------------------------------------
# ## HELPER FUNCTION TO GET CURRENT JOINT STATES
# -----------------------------------------------------------------
def get_current_joint_state(node: Node) -> JointState:
    """
    Creates a temporary subscriber, waits for one message,
    and returns the current JointState.
    """
    node.get_logger().info("Waiting for one /joint_states message...")

    # This is a "trick" to get a single message from a topic
    # We create a temporary node just for this
    temp_node = rclpy.create_node("joint_state_listener_temp")

    try:
        # Subscribe to the /joint_states topic
        joint_state_sub = temp_node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: setattr(temp_node, "joint_state", msg),
            10,
        )

        executor = SingleThreadedExecutor()
        executor.add_node(temp_node)

        # Spin until the message is received or we time out
        timeout_start = node.get_clock().now()
        while not hasattr(temp_node, "joint_state"):
            executor.spin_once(timeout_sec=0.1)
            if (node.get_clock().now() - timeout_start) > rclpy.duration.Duration(
                seconds=2.0
            ):
                node.get_logger().error("Timeout waiting for /joint_states message.")
                return None

        # We got the message
        return temp_node.joint_state
    finally:
        # Clean up the temporary node
        temp_node.destroy_node()


class MoveArmNode(Node):
    def __init__(self):
        super().__init__("move_arm_node")

        # --- 1. MoveGroup Action Client (for EXECUTING planned trajectories) ---
        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            rclpy.shutdown()
            return
        self.get_logger().info("MoveGroup action server is available.")

    # -----------------------------------------------------------------
    # ## SEND A 3D POSE GOAL (NOW WITH PATH CONSTRAINTS)
    # -----------------------------------------------------------------
    def send_pose_goal(
        self,
        pose: PoseStamped,
        group_name: str,
        end_effector_link: str,
        joints_to_lock: list[str] = None,
    ):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = (
            10.0  # Increased for complex 14-joint planning
        )

        # --- 1. ADD GOAL CONSTRAINTS (Where to end up) ---
        goal_msg.request.goal_constraints.append(
            self._pose_to_constraint(pose, end_effector_link)
        )

        # --- 2. ADD PATH CONSTRAINTS (How to get there) ---
        if joints_to_lock:
            self.get_logger().info(
                f"Adding path constraints to lock {len(joints_to_lock)} joints..."
            )
            current_joint_state = get_current_joint_state(self)

            if current_joint_state:
                path_constraints = Constraints()
                # Create a lock for each joint
                for joint_name_to_lock in joints_to_lock:
                    try:
                        # Find the joint's current position from /joint_states
                        index = current_joint_state.name.index(joint_name_to_lock)
                        position = current_joint_state.position[index]

                        jc = JointConstraint()
                        jc.joint_name = joint_name_to_lock
                        jc.position = position
                        jc.tolerance_above = 0.01  # Tight tolerance
                        jc.tolerance_below = 0.01
                        jc.weight = 1.0
                        path_constraints.joint_constraints.append(jc)

                    except ValueError:
                        self.get_logger().warn(
                            f"Joint '{joint_name_to_lock}' not found in /joint_states."
                        )

                # Add the path constraints to the main goal request
                goal_msg.request.path_constraints = path_constraints
            else:
                self.get_logger().error(
                    "Could not get current joint state, planning without path constraints."
                )

        # --- 3. SEND THE GOAL ---
        self.get_logger().info(f"Sending POSE goal for group '{group_name}'...")
        future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info("Trajectory successfully planned and executed!")
        else:
            self.get_logger().error(
                f"Planning failed with error code: {result.error_code.val}"
            )

    # Helper function to create pose constraints (Unchanged)
    @staticmethod
    def _pose_to_constraint(pose: PoseStamped, link_name):
        from moveit_msgs.msg import (
            PositionConstraint,
            OrientationConstraint,
        )
        from shape_msgs.msg import SolidPrimitive

        constraints = Constraints()
        constraints.name = "goal"
        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = link_name
        pos.target_point_offset.x = 0.0
        pos.target_point_offset.y = 0.0
        pos.target_point_offset.z = 0.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]
        pos.constraint_region.primitives.append(primitive)
        pos.constraint_region.primitive_poses.append(pose.pose)
        pos.weight = 1.0
        constraints.position_constraints.append(pos)
        ori = OrientationConstraint()
        ori.header = pose.header
        ori.link_name = link_name
        ori.orientation = pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.05
        ori.absolute_y_axis_tolerance = 0.05
        ori.absolute_z_axis_tolerance = 0.05
        ori.weight = 1.0
        constraints.orientation_constraints.append(ori)
        return constraints


# -----------------------------------------------------------------
# ## MAIN FUNCTION (REWRITTEN)
# -----------------------------------------------------------------
def main():
    rclpy.init()
    node = MoveArmNode()

    # --- 1. Define the joints we want to lock ---
    # We want to move arm_1, so we must lock arm_2.
    arm_2_joints_to_lock = [
        "t1_a2_joint_1",
        "t1_a2_joint_2",
        "t1_a2_joint_3",
        "t1_a2_joint_4",
        "t1_a2_joint_5",
        "t1_a2_joint_6",
    ]

    # --- 2. Define the pose goal for arm_1 ---
    # This is the pose from your log
    pose_goal_arm1 = PoseStamped()
    pose_goal_arm1.header.frame_id = "world"
    pose_goal_arm1.pose.position.x = 0.9736375207418347
    pose_goal_arm1.pose.position.y = 0.11829771805227382
    pose_goal_arm1.pose.position.z = 1.645184351619195
    pose_goal_arm1.pose.orientation.x = 0.6977397051529791
    pose_goal_arm1.pose.orientation.y = -0.29551575901348265
    pose_goal_arm1.pose.orientation.z = 0.2116595180479572
    pose_goal_arm1.pose.orientation.w = 0.6172762659032224

    # --- 3. Send the goal with constraints ---
    node.get_logger().info("--- Sending Goal to move arm_1 (while locking arm_2) ---")
    node.send_pose_goal(
        pose_goal_arm1,
        group_name="gantry_1_with_arm",  # The 14-joint group
        end_effector_link="t1_a1_end_effector_link",  # The link to move
        joints_to_lock=arm_2_joints_to_lock,  # The joints to keep fixed
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
