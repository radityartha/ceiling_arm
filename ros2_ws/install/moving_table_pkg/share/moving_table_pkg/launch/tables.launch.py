from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Path to XACRO file
    xacro_file = os.path.join(
        get_package_share_directory("moving_table_pkg"), "robots", "tables.xacro"
    )

    # Convert XACRO to URDF
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return LaunchDescription(
        [
            # # Robot State Publisher
            # Node(
            #     package="robot_state_publisher",
            #     executable="robot_state_publisher",
            #     name="robot_state_publisher",
            #     output="screen",
            #     parameters=[{"robot_description": robot_description_config}],
            # ),
            # Dual Table Controller
            Node(
                package="moving_table_pkg",
                executable="dual_table_controller",
                name="dual_table_controller",
                output="screen",
            ),
        ]
    )
