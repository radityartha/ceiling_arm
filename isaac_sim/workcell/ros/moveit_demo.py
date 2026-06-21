"""Drive all 4 workcell arms via MoveIt (plan+execute), one group at a time, looping.
Watch them move in Isaac (noVNC). Ctrl-C to stop.

    source /opt/ros/humble/setup.bash
    source /srv/data/users/raditya/kortex_min_ws/install/setup.bash
    source /srv/data/users/raditya/workcell_overlay_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 moveit_demo.py
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions

ARMS = {
    "arm_1": ["t1_a1_joint_1", "t1_a1_joint_2", "t1_a1_joint_3", "t1_a1_joint_4", "t1_a1_joint_5", "t1_a1_joint_6"],
    "arm_2": ["t1_a2_joint_1", "t1_a2_joint_2", "t1_a2_joint_3", "t1_a2_joint_4", "t1_a2_joint_5", "t1_a2_joint_6"],
    "arm_3": ["t2_a1_joint_1", "t2_a1_joint_2", "t2_a1_joint_3", "t2_a1_joint_4", "t2_a1_joint_5", "t2_a1_joint_6"],
    "arm_4": ["t2_a2_joint_1", "t2_a2_joint_2", "t2_a2_joint_3", "t2_a2_joint_4", "t2_a2_joint_5", "t2_a2_joint_6"],
}
POSE_A = [0.5, 0.6, 0.4, -0.4, 0.3, 0.0]
POSE_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def send(node, client, group, joints, positions):
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = group
    req.allowed_planning_time = 5.0
    req.num_planning_attempts = 10
    req.max_velocity_scaling_factor = 0.4
    req.max_acceleration_scaling_factor = 0.4
    c = Constraints()
    for name, pos in zip(joints, positions):
        jc = JointConstraint()
        jc.joint_name = name; jc.position = pos
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        return -1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    return rf.result().result.error_code.val


def main():
    rclpy.init()
    node = Node("workcell_moveit_demo")
    client = ActionClient(node, MoveGroup, "/move_action")
    client.wait_for_server()
    node.get_logger().info("connected; cycling 4 arms (Ctrl-C to stop)")
    try:
        while rclpy.ok():
            for group, joints in ARMS.items():
                code = send(node, client, group, joints, POSE_A)
                node.get_logger().info(f"{group} -> POSE_A: {'OK' if code == 1 else f'code {code}'}")
            for group, joints in ARMS.items():
                code = send(node, client, group, joints, POSE_HOME)
                node.get_logger().info(f"{group} -> HOME: {'OK' if code == 1 else f'code {code}'}")
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
