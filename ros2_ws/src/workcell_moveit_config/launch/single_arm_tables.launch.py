import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _strip_broken_overlays():
    """Remove the custom rviz2_ws / moveit2_ws builds from this launch's
    environment. The VNC/XFCE session injects them ahead of /opt/ros/humble,
    so ros2 launch would otherwise spawn the broken rviz2 (renders no window)
    and the wrong move_group. Stripping os.environ here makes every node this
    launch spawns resolve to the system install — so a single-terminal launch
    works regardless of the calling terminal's polluted PATH."""
    for var in ("PATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH",
                "CMAKE_PREFIX_PATH", "PYTHONPATH"):
        val = os.environ.get(var)
        if not val:
            continue
        cleaned = [p for p in val.split(os.pathsep)
                   if "rviz2_ws" not in p and "moveit2_ws" not in p]
        os.environ[var] = os.pathsep.join(cleaned)


def generate_launch_description():
    _strip_broken_overlays()

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.2.11",
        description="IP of the Kinova arm (default: arm 3, table 2 left)",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz via TimerAction. UNRELIABLE in this combined "
        "launch — the included kortex/move_group sub-launches starve the launch "
        "event loop so the timer never fires and no window appears. Prefer "
        "scripts/start_single_arm.sh, which polls for /move_group then runs "
        "system rviz2 in the foreground.",
    )
    use_fake_tables_arg = DeclareLaunchArgument(
        "use_fake_tables",
        default_value="false",
        description="Use fake hardware for table motors (true = no Modbus needed)",
    )

    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_tables = LaunchConfiguration("use_fake_tables")

    kortex_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kortex_bringup"), "launch", "gen3_lite.launch.py"
            ])
        ]),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": "false",
            "launch_rviz": "false",
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kinova_gen3_lite_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        ]),
    )

    moveit_config = MoveItConfigsBuilder(
        "gen3_lite_gen3_lite_2f", package_name="kinova_gen3_lite_moveit_config"
    ).to_moveit_configs()

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            str(moveit_config.package_path / "config/moveit.rviz"),
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(launch_rviz),
    )

    dual_table_controller = Node(
        package="moving_table_pkg",
        executable="dual_table_controller",
        name="dual_table_controller",
        parameters=[{"use_fake_hardware": use_fake_tables}],
        output="screen",
        # publish table joints on a separate topic so MoveIt doesn't spam
        # "joint not found" errors for t1_*/t2_* joints
        remappings=[("/joint_states", "/table_joint_states")],
    )

    return LaunchDescription([
        robot_ip_arg,
        launch_rviz_arg,
        use_fake_tables_arg,
        kortex_bringup,
        dual_table_controller,
        move_group,
        # wait 20 s for arm hardware + controllers to fully activate before RViz
        TimerAction(period=20.0, actions=[rviz_node]),
    ])
