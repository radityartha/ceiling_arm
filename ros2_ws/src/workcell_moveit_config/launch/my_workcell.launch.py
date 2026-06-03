import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    # Arm IP arguments — confirmed IPs on 192.168.2.x subnet
    # Override example: ros2 launch workcell_moveit_config my_workcell.launch.py \
    #            use_fake_hardware:=false arm1_ip:=192.168.2.10
    declared_hw_args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="Use mock hardware interfaces (false = real arms)"),
        DeclareLaunchArgument("arm1_ip", default_value="192.168.2.10",
                              description="IP of Arm 1 (Table-1 Left)"),
        DeclareLaunchArgument("arm2_ip", default_value="192.168.2.11",
                              description="IP of Arm 2 (Table-1 Right)"),
        DeclareLaunchArgument("arm3_ip", default_value="192.168.2.12",
                              description="IP of Arm 3 (Table-2 Left)"),
        DeclareLaunchArgument("arm4_ip", default_value="192.168.2.13",
                              description="IP of Arm 4 (Table-2 Right)"),
        DeclareLaunchArgument("use_fake_tables", default_value="true",
                              description="Use fake hardware for the table motors "
                              "(true = no Modbus needed)"),
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
                "use_sim_time":      "false",
            },
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )
    # Note: arm execution uses the per-arm controllers (arm_1..arm_4) defined in
    # the default config/moveit_controllers.yaml. The old coupled table_*_with_arm
    # controllers were removed because the table joints are no longer under
    # ros2_control — they are driven by dual_table_controller (added below) and
    # commanded via the move_dual_table service.

    ld = generate_demo_launch(moveit_config)

    # 2. Use Sim Time
    use_sim_time_config = LaunchConfiguration("use_sim_time", default="false")
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name == "use_sim_time":
            use_sim_time_config = LaunchConfiguration("use_sim_time")
            break

    # 3. Spawn controllers explicitly, sequenced.
    # generate_demo_launch's spawn_controllers.launch.py spawns every controller
    # in parallel, which intermittently loses the configure/activate handshake
    # race against a still-initializing controller_manager. Strip that include and
    # spawn joint_state_broadcaster first, then the arm/gripper controllers on its
    # exit. The include's source path is a substitution, so resolve it to match.
    _ctx = LaunchContext()

    def _include_path(action):
        src = action.launch_description_source
        raw = src._LaunchDescriptionSource__location
        return perform_substitutions(_ctx, normalize_to_list_of_substitutions(raw))

    for action in ld.entities[:]:
        if isinstance(action, IncludeLaunchDescription):
            if "spawn_controllers.launch.py" in _include_path(action):
                ld.entities.remove(action)
                break

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    ld.add_action(jsb_spawner)

    other_controllers = [
        "arm_1_controller", "arm_2_controller", "arm_3_controller", "arm_4_controller",
        "gripper_1_controller", "gripper_2_controller",
        "gripper_3_controller", "gripper_4_controller",
    ]
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[c],
                        output="screen",
                    )
                    for c in other_controllers
                ],
            )
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

    # 5. Livox Driver (Direct Node)
    livox_pkg = get_package_share_directory("livox_ros_driver2")
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

    # 6. Python Filter
    ld.add_action(
        Node(
            package="workcell_description",
            executable="lidar_filter.py",
            name="lidar_filter",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time_config}],
        )
    )

    # 7. Moving tables — driver node.
    # dual_table_controller owns the four table joints and publishes them
    # directly on /joint_states (a disjoint set from joint_state_broadcaster's
    # arm/gripper joints, so robot_state_publisher merges both). The tables then
    # animate in RViz when commanded via the move_dual_table service.
    ld.add_action(
        Node(
            package="moving_table_pkg",
            executable="dual_table_controller",
            name="dual_table_controller",
            output="screen",
            parameters=[{
                "use_fake_hardware": LaunchConfiguration("use_fake_tables"),
                "use_sim_time": use_sim_time_config,
            }],
        )
    )

    # Prepend the hardware args so they appear in --show-args
    for arg in reversed(declared_hw_args):
        ld.entities.insert(0, arg)

    return ld
