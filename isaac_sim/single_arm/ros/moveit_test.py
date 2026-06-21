"""Send a MoveGroup plan+execute goal (joint-space) to prove MoveIt drives Isaac.

    source /opt/ros/humble/setup.bash
    source /srv/data/users/raditya/kortex_min_ws/install/setup.bash
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 moveit_test.py
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions

TARGET = {"joint_1": 0.0, "joint_2": 0.8, "joint_3": 0.0,
          "joint_4": -0.5, "joint_5": 0.0, "joint_6": 0.0}


def main():
    rclpy.init()
    node = Node("moveit_test_client")
    client = ActionClient(node, MoveGroup, "/move_action")
    node.get_logger().info("waiting for /move_action ...")
    client.wait_for_server()

    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = "arm"
    req.allowed_planning_time = 5.0
    req.num_planning_attempts = 10
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    c = Constraints()
    for name, pos in TARGET.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False  # plan AND execute

    node.get_logger().info("sending plan+execute goal (joint_2->0.8, joint_4->-0.5) ...")
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        print("RESULT: goal REJECTED"); rclpy.shutdown(); return
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    code = res_fut.result().result.error_code.val
    # MoveItErrorCodes.SUCCESS == 1
    print(f"RESULT: error_code={code} ({'SUCCESS' if code == 1 else 'FAIL'})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
