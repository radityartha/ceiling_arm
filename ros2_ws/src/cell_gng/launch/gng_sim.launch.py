"""
gng_sim.launch.py — SIM SANDBOX: GNG node + ground-truth bridge + validation.

This launch wires the GNG pipeline for the Gazebo Fortress sandbox:
  1. gng_node                — subscribes to the sim LiDAR PointCloud2
  2. ros_gz_bridge (poses)   — bridges Gazebo model poses → PoseArray
  3. gng_validation_node     — SIM ONLY centroid-vs-truth reporter

It deliberately does NOT start Gazebo or the sim robot itself — launch
cell_gazebo_sim/cell_sim.launch.py separately (or your own sim bringup) so this
file stays focused on the GNG layer.  Keeping them separate also means this
file never accidentally pulls sim-only nodes into a real-hardware launch.

For machines where the Gazebo sensors cannot render (GPU limitation), set
use_fake_cloud:=true to publish a synthetic PointCloud2 on the same topic so
the GNG pipeline can be exercised end-to-end without the GPU.

Usage:
  # On a machine with working sim sensors:
  ros2 launch cell_gazebo_sim cell_sim.launch.py            # terminal 1
  ros2 launch cell_gng gng_sim.launch.py                    # terminal 2

  # On a machine without sensor rendering (verification):
  ros2 launch cell_gng gng_sim.launch.py use_fake_cloud:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cell_gng")
    gng_params = os.path.join(pkg, "config", "gng_params.yaml")
    pose_bridge = os.path.join(pkg, "config", "pose_bridge.yaml")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_fake_cloud_arg = DeclareLaunchArgument(
        "use_fake_cloud", default_value="false",
        description="Publish a synthetic PointCloud2 (no GPU sensors needed)")
    input_topic_arg = DeclareLaunchArgument(
        "input_cloud_topic", default_value="/livox/points")

    use_sim_time = LaunchConfiguration("use_sim_time")
    input_topic = LaunchConfiguration("input_cloud_topic")

    gng_node = Node(
        package="cell_gng", executable="gng_node", name="gng_node",
        output="screen",
        parameters=[gng_params,
                    {"input_cloud_topic": input_topic},
                    {"use_sim_time": use_sim_time}],
    )

    pose_bridge_node = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="gng_pose_bridge",
        parameters=[{"config_file": pose_bridge, "use_sim_time": use_sim_time}],
        output="screen",
    )

    validation_node = Node(
        package="cell_gng", executable="gng_validation_node",
        name="gng_validation_node", output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    fake_cloud = Node(
        package="cell_gng", executable="fake_cloud_publisher",
        name="fake_cloud_publisher", output="screen",
        condition=IfCondition(LaunchConfiguration("use_fake_cloud")),
        parameters=[{"topic": input_topic, "frame_id": "world"}],
    )

    return LaunchDescription([
        use_sim_time_arg, use_fake_cloud_arg, input_topic_arg,
        gng_node, pose_bridge_node, validation_node, fake_cloud,
    ])
