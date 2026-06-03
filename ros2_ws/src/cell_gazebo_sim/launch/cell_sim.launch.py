"""
Gazebo Fortress simulation launch for the ceiling-robot workcell.

This launch file is NEW and ADDITIVE.  It does NOT modify or replace
any existing real-hardware launch file.

What it starts:
  1. Gazebo Fortress (headless or with GUI) with cell.sdf world
  2. robot_state_publisher with the sim xacro (cell_sim.urdf.xacro)
  3. ros_gz_sim spawner — creates the robot inside Gazebo
  4. ros_gz_bridge — PointCloud2, Image, CameraInfo, Clock
  5. joint_state_broadcaster
  6. table_1_controller + table_2_controller
  (Arm controllers are spawned but left unconfigured; activate them
   when you want to command arms in sim.)

Usage:
  ros2 launch cell_gazebo_sim cell_sim.launch.py
  ros2 launch cell_gazebo_sim cell_sim.launch.py gz_gui:=false
"""

import os

# This workspace has two kortex_description installs:
#   /home/mobi/Documents/moonshot_project/ros2_ws → uses gazebo_ros2_control/GazeboSystem (Gazebo11)
#   /home/mobi/ros2_ws                            → uses gz_ros2_control/GazeboSimSystem (Fortress)
# We must ensure the Fortress-compatible one wins the ament_index lookup.
# Prepend the correct prefix so $(find kortex_description) resolves correctly in xacro.
_KORTEX_FORTRESS_PREFIX = "/home/mobi/ros2_ws/install/kortex_description"
if os.path.isdir(_KORTEX_FORTRESS_PREFIX):
    _ap = os.environ.get("AMENT_PREFIX_PATH", "")
    if _KORTEX_FORTRESS_PREFIX not in _ap:
        os.environ["AMENT_PREFIX_PATH"] = _KORTEX_FORTRESS_PREFIX + ":" + _ap

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_sim = get_package_share_directory("cell_gazebo_sim")

    # ── Arguments ──────────────────────────────────────────────────────────────
    gz_gui_arg = DeclareLaunchArgument(
        "gz_gui", default_value="true",
        description="Launch Gazebo with GUI (false = headless)"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    gz_gui = LaunchConfiguration("gz_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── 1. Robot description ───────────────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        os.path.join(pkg_sim, "urdf", "cell_sim.urdf.xacro"),
        " sim_controllers:=",
        os.path.join(pkg_sim, "config", "sim_ros2_controllers.yaml"),
    ])
    robot_description = {"robot_description": robot_description_content}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ── 2. Gazebo Fortress ─────────────────────────────────────────────────────
    world_file = os.path.join(pkg_sim, "worlds", "cell.sdf")

    gz_sim_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        condition=IfCondition(gz_gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r -s {world_file}"}.items(),
        condition=UnlessCondition(gz_gui),
    )

    # ── 3. Spawn robot into Gazebo ─────────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "trailer_workcell",
            "-z", "0.0",
        ],
        output="screen",
    )

    # ── 4. ros_gz_bridge ────────────────────────────────────────────────────────
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(pkg_sim, "config", "bridge.yaml"),
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # ── 5. Controllers (spawned sequentially) ──────────────────────────────────
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    table_1_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["table_1_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    table_2_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["table_2_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Activate table controllers after joint_state_broadcaster is active.
    activate_tables = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[table_1_spawner, table_2_spawner],
        )
    )

    # Spawn joint_state_broadcaster after robot is in Gazebo.
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner],
        )
    )

    return LaunchDescription([
        gz_gui_arg,
        use_sim_time_arg,
        rsp_node,
        gz_sim_with_gui,
        gz_sim_headless,
        spawn_robot,
        bridge_node,
        start_jsb_after_spawn,
        activate_tables,
    ])
