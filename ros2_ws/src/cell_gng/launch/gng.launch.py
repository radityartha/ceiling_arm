"""
gng.launch.py — launch the GNG node by itself.

This is the SENSOR-AGNOSTIC launch.  It starts only the GNG mapping node,
reading parameters from config/gng_params.yaml.  Point it at any
sensor_msgs/PointCloud2 source via the `input_cloud_topic` parameter:
  - sim LiDAR:  /livox/points  (default; from cell_gazebo_sim)
  - real Mid360: set input_cloud_topic to the real driver topic

Usage:
  ros2 launch cell_gng gng.launch.py
  ros2 launch cell_gng gng.launch.py input_cloud_topic:=/livox/points
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cell_gng")
    params = os.path.join(pkg, "config", "gng_params.yaml")

    input_topic_arg = DeclareLaunchArgument(
        "input_cloud_topic", default_value="/livox/points",
        description="PointCloud2 topic to map (sim LiDAR or real Mid360)")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false")

    gng_node = Node(
        package="cell_gng",
        executable="gng_node",
        name="gng_node",
        output="screen",
        parameters=[
            params,
            {"input_cloud_topic": LaunchConfiguration("input_cloud_topic")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([input_topic_arg, use_sim_time_arg, gng_node])
