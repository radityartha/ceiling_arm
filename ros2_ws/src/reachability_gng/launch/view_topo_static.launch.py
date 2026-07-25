"""View a saved STATIC GNG topo map in RViz -- one command, one terminal.

Starts topo_static_pub (republishes the saved map on /topo_map/static/markers,
transient-local) + RViz preloaded with the TopoStatic MarkerArray display, in
one process tree (single Ctrl-C stops both -- no orphan nodes).

    ros2 launch reachability_gng view_topo_static.launch.py
    ros2 launch reachability_gng view_topo_static.launch.py \
        map_file:=/tmp/topo_static_rgbd2.npz

RViz renders on the noVNC display :1 (DISPLAY is auto-set by ~/.bashrc); view it
in the browser at http://<pc-ip>:22380/vnc.html
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('reachability_gng')
    rviz_cfg = os.path.join(share, 'config', 'topo_static.rviz')

    map_file = LaunchConfiguration('map_file')

    return LaunchDescription([
        DeclareLaunchArgument('map_file',
                              default_value='/tmp/topo_static_rgbd.npz'),

        Node(package='reachability_gng', executable='topo_static_pub',
             name='topo_static_pub', output='screen',
             parameters=[{'map_file': map_file}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_cfg]),
    ])
