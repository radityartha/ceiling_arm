import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    # 1. MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(
            file_path="config/trailer_workcell.urdf.xacro",
            mappings={"use_fake_hardware": "true", "use_sim_time": "false"},
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

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
        "table_1_with_arm_controller,"
        "table_2_with_arm_controller"
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
                "2.13",
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

    return ld
