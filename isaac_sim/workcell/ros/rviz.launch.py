"""RViz MotionPlanning for the workcell, to drive arms + tables interactively.
Run alongside ros2_bridge_gui.py + bringup.launch.py (same env).

    DISPLAY=:22380 ros2 launch isaac_sim/workcell/ros/rviz.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

WORKCELL_XACRO = os.path.join(
    get_package_share_directory("workcell_description"), "urdf", "workcell.urdf.xacro",
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(file_path=WORKCELL_XACRO,
                           mappings={"sim_isaac": "true", "use_fake_hardware": "false"})
        .robot_description_semantic(file_path="config/trailer_workcell.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("workcell_moveit_config"), "config", "moveit.rviz")

    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([rviz])
