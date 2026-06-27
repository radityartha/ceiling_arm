"""4-arm workcell MoveIt + ros2_control bringup driving Isaac via topic_based_ros2_control.

robot_description is built from workcell_description/urdf/workcell.urdf.xacro with
sim_isaac:=true, so each arm/gripper uses topic_based on /isaac_joint_commands and
/isaac_joint_states (the topics isaac_sim/workcell/ros2_bridge*.py serves). Tables stay
mock (not bridged to Isaac in this scope). MoveIt groups: arm_1..4 + gripper_1..4.

Run order:
  1) python isaac_sim/workcell/ros2_bridge_gui.py        (Isaac, CycloneDDS, domain 42)
  2) ros2 launch isaac_sim/workcell/ros/bringup.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

WORKCELL_XACRO = os.path.join(
    get_package_share_directory("workcell_description"), "urdf", "workcell.urdf.xacro",
)
CONTROLLERS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros2_controllers.yaml")

ARMS = ["arm_1_controller", "arm_2_controller", "arm_3_controller", "arm_4_controller"]
GRIPPERS = ["gripper_1_controller", "gripper_2_controller", "gripper_3_controller", "gripper_4_controller"]


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(file_path=WORKCELL_XACRO,
                           mappings={"sim_isaac": "true", "use_fake_hardware": "false"})
        .robot_description_semantic(file_path="config/trailer_workcell.srdf")
        .trajectory_execution(file_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[moveit_config.robot_description]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[moveit_config.robot_description, CONTROLLERS_YAML]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit_config.to_dict(), {"use_sim_time": False}]),
    ]
    tables = ["gantry_1_controller", "gantry_2_controller"]
    for c in ["joint_state_broadcaster"] + ARMS + GRIPPERS + tables:
        nodes.append(Node(package="controller_manager", executable="spawner",
                          output="screen", arguments=[c, "-c", "/controller_manager"]))
    return LaunchDescription(nodes)
