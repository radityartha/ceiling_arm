from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", ""],
                output="screen",
                parameters=[{"robot_description": open("/tmp/workcell.urdf").read()}],
            ),
        ]
    )
