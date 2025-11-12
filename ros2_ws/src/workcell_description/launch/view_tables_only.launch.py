import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    # --- 1. Locate packages ---
    pkg_workcell_description = get_package_share_directory("workcell_description")
    pkg_moving_table = get_package_share_directory("moving_table_pkg")

    # --- 2. Define file paths ---
    # --- We are loading the NEW test file ---
    xacro_file_path = os.path.join(
        pkg_workcell_description, "urdf", "view_two_tables.urdf.xacro"
    )
    rviz_config_path = os.path.join(
        pkg_workcell_description,
        "rviz",
        "workcell.rviz",  # You can use your existing config
    )

    # --- 3. Launch arguments ---
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
    ]

    # --- 4. Generate robot_description (Robust Python Method) ---
    try:
        robot_description_xml = xacro.process_file(
            xacro_file_path, mappings={"workcell_description": pkg_workcell_description}
        ).toxml()
    except Exception as e:
        print("-------------------------------------------------")
        print(f"Xacro processing failed: {e}")
        print(f"Check your xacro file: {xacro_file_path}")
        print("-------------------------------------------------")
        raise e

    robot_description = {"robot_description": robot_description_xml}

    # --- 5. Core nodes ---

    # This is the ONLY Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # This is your controller for the tables. It will publish their joint states.
    node_dual_table_controller = Node(
        package="moving_table_pkg",
        executable="dual_table_controller",
        name="dual_table_controller",
        output="screen",
        parameters=[{"use_fake_hardware": use_fake_hardware}],
    )

    # This is the ONLY RViz
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    # --- 6. Return all ---
    # We DO NOT start the joint_state_publisher_gui.
    # This will STOP the flickering.
    return LaunchDescription(
        declared_arguments
        + [
            node_robot_state_publisher,
            node_rviz,
            node_dual_table_controller,
        ]
    )
