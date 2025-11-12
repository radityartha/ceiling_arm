import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # -------------------------
    # Package directories
    # -------------------------
    pkg_workcell_description = get_package_share_directory("workcell_description")
    pkg_moveit_config = get_package_share_directory("workcell_moveit_config")
    pkg_moving_table = get_package_share_directory("moving_table_pkg")

    # -------------------------
    # Launch configurations
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
    ]

    # -------------------------
    # Process XACRO to URDF dynamically
    # -------------------------
    xacro_file = os.path.join(pkg_workcell_description, "urdf", "workcell.urdf.xacro")
    try:
        robot_description_xml = xacro.process_file(
            xacro_file,
            mappings={
                "workcell_description": pkg_workcell_description,
            },
        ).toxml()
    except Exception as e:
        print("Xacro processing failed:", e)
        raise e

    robot_description = {"robot_description": robot_description_xml}

    # -------------------------
    # Robot State Publisher
    # -------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # -------------------------
    # Dual Table Controller (fake hardware)
    # -------------------------
    table_controller_node = Node(
        package="moving_table_pkg",
        executable="dual_table_controller",
        name="dual_table_controller",
        output="screen",
        parameters=[{"use_fake_hardware": use_fake_hardware}],
    )

    # -------------------------
    # Joint State Publisher GUI (publish all arm + table joints)
    # -------------------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # -------------------------
    # MoveIt move_group
    # -------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            {
                "robot_description_semantic": os.path.join(
                    pkg_moveit_config, "config", "workcell.srdf"
                )
            },
            os.path.join(pkg_moveit_config, "config", "kinematics.yaml"),
            os.path.join(pkg_moveit_config, "config", "controllers.yaml"),
            os.path.join(pkg_moveit_config, "config", "ompl_planning.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # -------------------------
    # RViz
    # -------------------------
    rviz_config_file = os.path.join(pkg_moveit_config, "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # -------------------------
    # Launch Description
    # -------------------------
    return LaunchDescription(
        declared_arguments
        + [
            rsp_node,
            table_controller_node,
            joint_state_publisher_node,
            move_group_node,
            rviz_node,
        ]
    )
