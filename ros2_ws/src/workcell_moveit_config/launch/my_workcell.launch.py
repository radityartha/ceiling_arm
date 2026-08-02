import os
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    # Arm IP arguments — confirmed IPs on 192.168.2.x subnet
    # Override example: ros2 launch workcell_moveit_config my_workcell.launch.py \
    #            use_fake_hardware:=false arm1_ip:=192.168.2.13
    declared_hw_args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="Use mock hardware interfaces (false = real arms)"),
        # Default false: reachability_gng's gng_collision node now feeds
        # /planning_scene from the GNG env map instead (see
        # reachability_gng/launch/topo_fusion.launch.py). Raw livox_ros_driver2
        # still runs below either way (e.g. for visualization); this only gates
        # the lidar_filter.py node that used to feed the octomap's livox_lidar
        # sensor plugin via /livox/filtered. Pass true to restore the old
        # octomap-from-LIDAR collision path.
        DeclareLaunchArgument("enable_lidar_octomap_filter", default_value="false",
                              description="Publish /livox/filtered for the "
                                          "octomap livox_lidar sensor (legacy; "
                                          "GNG collision replaces this)"),
        # Fixed 2026-07-30: these 4 defaults were reversed (arm1<->arm4,
        # arm2<->arm3) relative to CLAUDE.md's documented wiring and
        # single_rviz_workcell.launch.py's (correct) defaults -- verified via
        # workcell.urdf.xacro's arm1_ip->t1_a1 / arm2_ip->t1_a2 /
        # arm3_ip->t2_a1 / arm4_ip->t2_a2 mapping.
        DeclareLaunchArgument("arm1_ip", default_value="192.168.2.13",
                              description="IP of Arm 1 / t1_a1 (gantry_1, mount_right)"),
        DeclareLaunchArgument("arm2_ip", default_value="192.168.2.12",
                              description="IP of Arm 2 / t1_a2 (gantry_1, mount_left)"),
        DeclareLaunchArgument("arm3_ip", default_value="192.168.2.11",
                              description="IP of Arm 3 / t2_a1 (gantry_2, mount_right)"),
        DeclareLaunchArgument("arm4_ip", default_value="192.168.2.10",
                              description="IP of Arm 4 / t2_a2 (gantry_2, mount_left)"),
        # Real-hardware gantry bridge (see workcell.urdf.xacro's TableRealTopicBased
        # block + dual_table_controller.py's bridge.* params). Two gates, both
        # must be true to actually drive the steppers from a MoveIt trajectory:
        # dual_table_controller must be running (it isn't in fake-hardware mode,
        # since the table joints are covered by mock_components/FakeSystem then)
        # AND enable_gantry_bridge:=true. Defaults to false even on real hardware
        # so a first real-mode launch doesn't move the tables until reviewed.
        DeclareLaunchArgument("enable_gantry_bridge", default_value="false",
                              description="Let MoveIt trajectories drive the real "
                                          "Modbus table via dual_table_controller's "
                                          "topic_based_ros2_control bridge"),
    ]

    # 1. MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(
            file_path="config/trailer_workcell.urdf.xacro",
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "arm1_ip":           LaunchConfiguration("arm1_ip"),
                "arm2_ip":           LaunchConfiguration("arm2_ip"),
                "arm3_ip":           LaunchConfiguration("arm3_ip"),
                "arm4_ip":           LaunchConfiguration("arm4_ip"),
                "enable_gantry_bridge": LaunchConfiguration("enable_gantry_bridge"),
                "use_sim_time":      "false",
            },
        )
        .to_moveit_configs()
    )

    # Give the Kortex arms more time to finish trajectories. Isaac physics runs
    # slower than the planned trajectory duration, so MoveIt's default 1.2x
    # execution-duration monitor cancels mid-flight and reports CONTROL_FAILED
    # (-4). 3.0x (matching single_rviz_workcell.launch.py) lets slow moves land.
    moveit_config.trajectory_execution["trajectory_execution.allowed_execution_duration_scaling"] = 3.0
    moveit_config.trajectory_execution["trajectory_execution.allowed_goal_duration_margin"] = 5.0

    ld = generate_demo_launch(moveit_config)

    # 2. Use Sim Time
    use_sim_time_config = LaunchConfiguration("use_sim_time", default="false")
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name == "use_sim_time":
            use_sim_time_config = LaunchConfiguration("use_sim_time")
            break

    # 3. Fix Controllers (Spawn only the 7 correct ones)
    controller_names = (
        "joint_state_broadcaster,"
        "gripper_1_controller,gripper_2_controller,gripper_3_controller,gripper_4_controller,"
        "gantry_1_with_arm_controller,"
        "gantry_2_with_arm_controller"
    )

    spawn_launch_path = (
        moveit_config.package_path / "launch" / "spawn_controllers.launch.py"
    )

    # Remove old spawner
    for action in ld.entities[:]:
        if isinstance(action, IncludeLaunchDescription):
            if "spawn_controllers.launch.py" in str(action.launch_description_source):
                ld.entities.remove(action)
                break

    # Add new spawner
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(spawn_launch_path)),
            launch_arguments={
                "controller_names": controller_names,
                "use_sim_time": use_sim_time_config,
            }.items(),
        )
    )

    # 4. Static TF (Sensor Position)
    # Z=1.5 puts it safely BELOW the ceiling so rays hit the floor.
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="livox_static_tf",
            arguments=[
                # X, Y, Z (Your specific coordinates)
                "2.3",
                "0",
                "1.9",
                # Yaw, Pitch, Roll (in Radians)
                # Pitch = 3.14159 (180 degrees) makes it look DOWN
                "3.14159",
                "3.14159",
                "0",
                "world",  # Parent frame
                "livox_frame",  # Child frame
            ],
            parameters=[{"use_sim_time": use_sim_time_config}],
        )
    )

    # 5. Livox Driver (Direct Node) -- degrade gracefully if the livox_ros_driver2
    # submodule source isn't checked out (empty dir with no .gitmodules mapping is
    # a known state on some checkouts). Without this, get_package_share_directory
    # raises and the WHOLE bringup (arms, gantries, MoveIt) fails to launch, even
    # though none of that needs the LIDAR. Skipping just means no LIDAR collision
    # sensing this run -- surfaced loudly below, not silently dropped.
    try:
        livox_pkg = get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError:
        livox_pkg = None
        print("[my_workcell.launch.py] WARNING: livox_ros_driver2 package not "
              "found -- skipping the LIDAR driver + filter. No LIDAR collision "
              "sensing this run; check out the submodule source and rebuild to "
              "restore it.")

    if livox_pkg is not None:
        config_path = os.path.join(livox_pkg, "config", "MID360_config.json")

        ld.add_action(
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "xfer_format": 0,  # Standard PointCloud2
                        "publish_freq": 20.0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "output_data_type": 0,
                        "frame_id": "livox_frame",
                        "lvx_file_path": "/home/livox/livox_test.lvx",
                        "user_config_path": config_path,
                        "cmdline_input_bd_code": "livox0000000001",
                        "use_sim_time": use_sim_time_config,
                    }
                ],
                # REMAP to a unique topic to avoid conflicts
                remappings=[("/livox/lidar", "/livox/points")],
            )
        )

        # 6. Python Filter -- feeds the octomap's livox_lidar sensor plugin via
        # /livox/filtered. Off by default: reachability_gng's gng_collision now
        # publishes GNG-derived CollisionObjects to /planning_scene instead (see
        # enable_lidar_octomap_filter above).
        ld.add_action(
            Node(
                package="workcell_description",
                executable="lidar_filter.py",
                name="lidar_filter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time_config}],
                condition=IfCondition(LaunchConfiguration("enable_lidar_octomap_filter")),
            )
        )

    # 7. Real Modbus table driver -- only meaningful on real hardware; in
    # fake-hardware mode the table joints are served by mock_components/
    # FakeSystem (see workcell.urdf.xacro) and this node would just fight it
    # over the "joint_states" topic name.
    ld.add_action(
        Node(
            package="moving_table_pkg",
            executable="dual_table_controller",
            name="dual_table_controller",
            output="screen",
            parameters=[{
                "use_fake_hardware": False,
                "bridge.enable": LaunchConfiguration("enable_gantry_bridge"),
                "use_sim_time": use_sim_time_config,
            }],
            condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
        )
    )

    # Prepend the hardware args so they appear in --show-args
    for arg in reversed(declared_hw_args):
        ld.entities.insert(0, arg)

    return ld
