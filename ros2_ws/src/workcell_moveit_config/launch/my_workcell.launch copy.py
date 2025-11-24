import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node  # <--- 1. ADD THIS IMPORT


def generate_launch_description():
    # 1. Build the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config")
        .robot_description(file_path="config/trailer_workcell.urdf.xacro")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # 2. Generate the base launch description
    ld = generate_demo_launch(moveit_config)

    # 3. Handle use_sim_time
    use_sim_time_config = LaunchConfiguration("use_sim_time", default="true")
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name == "use_sim_time":
            use_sim_time_config = LaunchConfiguration("use_sim_time")
            break

    # --- 4. ADD THE STATIC TRANSFORM PUBLISHER ---
    # This tells ROS: "The 'livox_frame' is attached to 'world' at these coordinates."
    # Format: [x, y, z, yaw, pitch, roll, parent_frame, child_frame]
    # Example: 2.0 meters up (z=2.0), facing down? Adjust accordingly.
    # --- 4. ADD THE STATIC TRANSFORM PUBLISHER ---
    node_static_tf = Node(
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
    ld.add_action(node_static_tf)
    # ---------------------------------------------

    # 5. Fix the Controller Spawners (Your previous fix)
    controller_names_to_spawn = (
        "joint_state_broadcaster,"
        "gripper_1_controller,gripper_2_controller,gripper_3_controller,gripper_4_controller,"
        "table_1_with_arm_controller,"
        "table_2_with_arm_controller"
    )

    spawn_controllers_launch_file = (
        moveit_config.package_path / "launch" / "spawn_controllers.launch.py"
    )

    for action in ld.entities[:]:
        if isinstance(action, IncludeLaunchDescription):
            if "spawn_controllers.launch.py" in str(action.launch_description_source):
                ld.entities.remove(action)
                break

    spawn_controllers_fixed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(spawn_controllers_launch_file)),
        launch_arguments={
            "controller_names": controller_names_to_spawn,
            "use_sim_time": use_sim_time_config,
        }.items(),
    )

    ld.add_action(spawn_controllers_fixed)

    # --- 6. Livox Driver (DEFINED DIRECTLY) ---
    # We define the node here to GUARANTEE parameters are correct.

    # Find the config file
    livox_pkg = get_package_share_directory("livox_ros_driver2")
    user_config_path = os.path.join(livox_pkg, "config", "MID360_config.json")

    livox_driver_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {
                "xfer_format": 1,  # <--- CRITICAL: 1 = Standard PointCloud2
                "publish_freq": 10.0,  # 10 Hz
                "multi_topic": 0,
                "data_src": 0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": user_config_path,
                "cmdline_input_bd_code": "livox0000000001",
                "use_sim_time": use_sim_time_config,
            }
        ],
    )
    ld.add_action(livox_driver_node)

    # --- 7. Python LiDAR Filter ---
    lidar_filter_node = Node(
        package="workcell_description",
        executable="lidar_filter.py",
        name="lidar_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_config}],
    )
    ld.add_action(lidar_filter_node)

    return ld
