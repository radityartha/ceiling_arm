#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup


class MoveArmNode(Node):
    def __init__(self):
        super().__init__("move_arm_node")
        self._action_client = ActionClient(self, MoveGroup, "move_action")

    def send_goal(
        self,
        pose: PoseStamped,
        group_name="arm_1",
        end_effector_link="t1_a1_end_effector_link",
    ):
        # Wait until the action server is available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            return

        goal_msg = MoveGroup.Goal()

        # Basic goal setup: planning group and target pose
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.goal_constraints.append(
            self._pose_to_constraint(pose, end_effector_link)
        )

        self.get_logger().info("Sending goal to MoveGroup action server...")
        future = self._action_client.send_goal_async(goal_msg)

        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return

        # Get the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info("Trajectory successfully planned and executed!")
        else:
            self.get_logger().error(
                f"Planning failed with error code: {result.error_code.val}"
            )

    @staticmethod
    def _pose_to_constraint(pose: PoseStamped, link_name):
        from moveit_msgs.msg import (
            Constraints,
            PositionConstraint,
            OrientationConstraint,
        )
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Vector3

        constraints = Constraints()
        constraints.name = "goal"

        # Position constraint
        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = link_name
        pos.target_point_offset.x = 0.0
        pos.target_point_offset.y = 0.0
        pos.target_point_offset.z = 0.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]  # small tolerance box
        pos.constraint_region.primitives.append(primitive)
        pos.constraint_region.primitive_poses.append(pose.pose)
        pos.weight = 1.0
        constraints.position_constraints.append(pos)

        # Orientation constraint
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


def main():
    rclpy.init()
    node = MoveArmNode()

    # Define a goal pose
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "world"
    pose_goal.pose.position.x = -0.656
    pose_goal.pose.position.y = 0.62999
    pose_goal.pose.position.z = -0.32627
    pose_goal.pose.orientation.x = -0.35656
    pose_goal.pose.orientation.y = -0.60695
    pose_goal.pose.orientation.z = -0.35052
    pose_goal.pose.orientation.w = 0.61774

    node.send_goal(pose_goal)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
