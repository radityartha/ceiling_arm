import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # Import the fix


def generate_launch_description():

    # --- 1. Find Files ---

    # Get the path to your 'workcell_description' package
    pkg_workcell_description = FindPackageShare("workcell_description")

    # Path to the test xacro file we want to view
    xacro_file_path = PathJoinSubstitution(
        [pkg_workcell_description, "urdf", "view_table_test.urdf.xacro"]
    )

    # Path to a default rviz config file (optional, but good)
    # You can create a simple 'view_test.rviz' file and save it here
    rviz_config_file = PathJoinSubstitution(
        [
            pkg_workcell_description,
            "rviz",
            "view_model.rviz",  # Assumes you create this file
        ]
    )

    # --- 2. Define Launch Arguments ---

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    ]

    # --- 3. Run Xacro to Get Robot Description ---

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file_path]
    )

    # --- FIX: Wrap the command output as a string parameter ---
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- 4. Define Nodes to Launch ---

    # Robot State Publisher
    # Publishes the TF frames from the URDF
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher GUI
    # Provides sliders for your 'test_table_linear_joint', etc.
    node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz
    # The 3D visualizer
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],  # Load the config file
        output="screen",
    )

    # --- 5. Create the Launch Description ---

    return LaunchDescription(
        declared_arguments
        + [node_robot_state_publisher, node_joint_state_publisher_gui, node_rviz]
    )
