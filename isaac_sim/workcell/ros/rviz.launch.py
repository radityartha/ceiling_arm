"""RViz MotionPlanning for the workcell, to drive arms + tables interactively.
Run alongside ros2_bridge_gui.py + bringup.launch.py (same env).

Loads the reachability_gng RViz config (gng_moveit.rviz): MotionPlanning
(groups table_1_with_arm_1 / table_1_with_arm_2) + RobotModel + the two GNG
reachability MarkerArray displays (/gng_arm1//gng_arm2/gng_markers), with the
table_2/arm_3/arm_4 links hidden. Publish the clouds alongside this with
`ros2 launch reachability_gng gng_clouds.launch.py` (from THIS repo's workspace,
same ROS_DOMAIN_ID / RMW), so they appear over the Isaac-driven robot.

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

# reachability_gng config lives in this repo's ros2_ws (not the Isaac overlay),
# so reference it by repo-relative path rather than get_package_share_directory.
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
GNG_RVIZ = os.path.join(
    _REPO, "ros2_ws", "src", "reachability_gng", "config", "gng_moveit.rviz")


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(file_path=WORKCELL_XACRO,
                           mappings={"sim_isaac": "true", "use_fake_hardware": "false"})
        .robot_description_semantic(file_path="config/trailer_workcell.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_cfg = GNG_RVIZ

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
