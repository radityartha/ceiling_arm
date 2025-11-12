import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- 1. Paths ---
    pkg_workcell_description = FindPackageShare("workcell_description")

    xacro_file = PathJoinSubstitution(
        [pkg_workcell_description, "urdf", "workcell.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [pkg_workcell_description, "rviz", "workcell.rviz"]
    )

    # --- 2. Launch Arguments ---
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true",
        )
    ]

    # --- 3. Robot Description ---
    robot_description_content = Command(["xacro ", xacro_file])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- 4. Nodes ---
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # --- 5. Return LaunchDescription ---
    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz2,
        ]
    )
