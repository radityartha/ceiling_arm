"""Send a sequence of MoveGroup plan+execute goals so the arm visibly moves in Isaac.
Loops through a few joint-space poses. Ctrl-C to stop.

    source /opt/ros/humble/setup.bash
    source /srv/data/users/raditya/kortex_min_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 moveit_demo.py
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions

JN = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
POSES = [
    ("home",   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ("reach",  [0.6, 0.8, 0.5, -0.5, 0.3, 0.0]),
    ("left",   [-0.8, 0.4, 0.9, 0.6, -0.4, 0.5]),
    ("tuck",   [0.0, 1.2, 1.2, 0.0, 0.6, 0.0]),
]


def send(node, client, positions):
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = "arm"
    req.allowed_planning_time = 5.0
    req.num_planning_attempts = 10
    req.max_velocity_scaling_factor = 0.4
    req.max_acceleration_scaling_factor = 0.4
    c = Constraints()
    for name, pos in zip(JN, positions):
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
    node = Node("moveit_demo")
    client = ActionClient(node, MoveGroup, "/move_action")
    client.wait_for_server()
    node.get_logger().info("connected to /move_action; cycling poses (Ctrl-C to stop)")
    i = 0
    try:
        while rclpy.ok():
            name, pos = POSES[i % len(POSES)]
            code = send(node, client, pos)
            node.get_logger().info(f"-> {name}: {'OK' if code == 1 else f'code {code}'}")
            i += 1
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
