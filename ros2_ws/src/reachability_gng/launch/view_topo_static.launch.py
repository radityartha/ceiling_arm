"""View a saved STATIC GNG topo map in RViz -- one command, one terminal.

Starts topo_static_pub (republishes the saved map on /topo_map/static/markers,
transient-local) + RViz preloaded with the TopoStatic MarkerArray display, in
one process tree (single Ctrl-C stops both -- no orphan nodes).

    ros2 launch reachability_gng view_topo_static.launch.py
    ros2 launch reachability_gng view_topo_static.launch.py \
        map_file:=/tmp/topo_static_rgbd2.npz

with_cloud:=true adds the live camera clouds next to the map -- the colour
clouds (what the room looks like) plus the depth clouds that actually fed the
map, the latter off by default. That is the view you want when asking "does
this map match the room?", because a map alone cannot show you what it missed.

    # map + live colour cloud, cameras ALREADY running
    ros2 launch reachability_gng view_topo_static.launch.py \
        map_file:=/tmp/topo_static_a.npz with_cloud:=true

    # same, but bring the 2 cameras up too (only if nothing else has them:
    # two drivers fighting over one USB device knocks BOTH cameras offline)
    ros2 launch reachability_gng view_topo_static.launch.py \
        map_file:=/tmp/topo_static_a.npz with_cloud:=true with_cameras:=true

RViz renders on the noVNC display :1 (DISPLAY is auto-set by ~/.bashrc); view it
in the browser at http://<pc-ip>:22380/vnc.html
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('reachability_gng')

    map_file = LaunchConfiguration('map_file')
    with_cloud = LaunchConfiguration('with_cloud')
    with_cameras = LaunchConfiguration('with_cameras')

    # one config or the other -- the cloud one is a superset, but loading it
    # without cameras leaves two displays permanently "no messages received",
    # which reads like a fault when it is just a viewer choice.
    rviz_cfg = PythonExpression([
        "'", os.path.join(share, 'config', 'topo_static_cloud.rviz'),
        "' if '", with_cloud, "' == 'true' else '",
        os.path.join(share, 'config', 'topo_static.rviz'), "'"])

    return LaunchDescription([
        DeclareLaunchArgument('map_file',
                              default_value='/tmp/topo_static_rgbd.npz'),
        DeclareLaunchArgument('with_cloud', default_value='false',
                              description='show the live camera clouds too'),
        DeclareLaunchArgument('with_cameras', default_value='false',
                              description='also start the 2 RealSense drivers'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'realsense_dual.launch.py')),
            condition=IfCondition(with_cameras)),

        Node(package='reachability_gng', executable='topo_static_pub',
             name='topo_static_pub', output='screen',
             parameters=[{'map_file': map_file}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_cfg]),
    ])
