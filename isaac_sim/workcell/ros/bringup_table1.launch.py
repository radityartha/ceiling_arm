"""TABLE_1-ONLY MoveIt + ros2_control bringup for the Isaac GNG view.

Same topic_based bridge to Isaac (/isaac_joint_commands, /isaac_joint_states) as
bringup.launch.py, but the robot_description is the trimmed table1_isaac.urdf
(gantry_1 + arm_1 + arm_2 only) with trailer_table1.srdf — so move_group / RViz
contain NO gantry_2/arm_3/arm_4 (the reliable way to hide them). Isaac still runs
the full 4-arm articulation; this model just tracks/commands the gantry_1 joints.

Regenerate the trimmed model first if URDF/SRDF changed:
  python3 isaac_sim/workcell/ros/make_table1_model.py

Run order (same env: ROS_DOMAIN_ID=42 RMW=cyclonedds):
  1) python isaac_sim/workcell/ros2_bridge_gui.py        (Isaac)
  2) ros2 launch isaac_sim/workcell/ros/bringup_table1.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "table1_isaac.urdf")
SRDF = os.path.join(HERE, "trailer_table1.srdf")
CONTROLLERS_YAML = os.path.join(HERE, "ros2_controllers.yaml")
MOVEIT_CTRL = os.path.join(HERE, "moveit_controllers_table1.yaml")

# only controllers whose joints exist in the trimmed model
SPAWN = ["joint_state_broadcaster", "arm_1_controller", "arm_2_controller",
         "gripper_1_controller", "gripper_2_controller", "gantry_1_controller"]


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(file_path=URDF)
        .robot_description_semantic(file_path=SRDF)
        .trajectory_execution(file_path=MOVEIT_CTRL)
        .planning_pipelines(pipelines=["ompl"])
        # load the octomap updaters (config/sensors_3d.yaml) so move_group
        # voxelizes /rgbd*/collision_cloud into a planning-scene octomap;
        # without this the planner sees an EMPTY world (no collision avoidance).
        .sensors_3d()
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
    for c in SPAWN:
        nodes.append(Node(package="controller_manager", executable="spawner",
                          output="screen", arguments=[c, "-c", "/controller_manager"]))
    return LaunchDescription(nodes)
