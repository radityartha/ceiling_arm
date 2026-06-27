"""RViz (GNG config) for the TABLE_1-ONLY Isaac view.

Builds the MoveIt params from the trimmed table1_isaac.urdf / trailer_table1.srdf
so the MotionPlanning panel + RobotModel contain NO gantry_2/arm_3/arm_4, and
loads reachability_gng/config/gng_moveit.rviz (clouds + MotionPlanning). Run
alongside bringup_table1.launch.py + the bridge + gng_clouds.launch.py.

    DISPLAY=:22380 ros2 launch isaac_sim/workcell/ros/rviz_table1.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "table1_isaac.urdf")
SRDF = os.path.join(HERE, "trailer_table1.srdf")
_REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
GNG_RVIZ = os.path.join(_REPO, "ros2_ws", "src", "reachability_gng",
                        "config", "gng_moveit.rviz")


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(file_path=URDF)
        .robot_description_semantic(file_path=SRDF)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", GNG_RVIZ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([rviz])
