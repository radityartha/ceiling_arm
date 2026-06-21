"""Single-arm MoveIt + ros2_control bringup that drives Isaac Sim via topic_based_ros2_control.

robot_description is built from the kortex gen3_lite_gen3_lite_2f xacro with sim_isaac:=true,
so the ros2_control hardware is topic_based_ros2_control/TopicBasedSystem talking on
/isaac_joint_commands and /isaac_joint_states (the topics isaac_sim/single_arm/ros2_bridge.py serves).

Run order:
  1) python isaac_sim/single_arm/ros2_bridge.py        (Isaac, CycloneDDS, domain 42)
  2) ros2 launch isaac_sim/single_arm/ros/bringup.launch.py   (same env)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

KORTEX_XACRO = os.path.join(
    get_package_share_directory("kortex_description"),
    "robots", "gen3_lite_gen3_lite_2f.xacro",
)
CONTROLLERS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros2_controllers.yaml")


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("gen3_lite_gen3_lite_2f", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(file_path=KORTEX_XACRO, mappings={"sim_isaac": "true"})
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[moveit_config.robot_description],
    )
    ros2_control = Node(
        package="controller_manager", executable="ros2_control_node", output="screen",
        parameters=[moveit_config.robot_description, CONTROLLERS_YAML],
    )
    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    jtc = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_trajectory_controller", "-c", "/controller_manager"])
    grip = Node(package="controller_manager", executable="spawner", output="screen",
                arguments=["gen3_lite_2f_gripper_controller", "-c", "/controller_manager"])
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )
    return LaunchDescription([rsp, ros2_control, jsb, jtc, grip, move_group])
