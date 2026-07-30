"""Bringup RGBD + live detection viewer -- one command, one terminal.

Includes rgbd_perception.launch.py (cameras + YOLOE seg_router + depth_cloud) AND
opens rqt_image_view on the segmentation debug overlay, in one process tree
(single Ctrl-C stops everything -- no orphan nodes).

    ros2 launch reachability_gng rgbd_view.launch.py
    ros2 launch reachability_gng rgbd_view.launch.py image_topic:=/rgbd2/seg/debug_image
    ros2 launch reachability_gng rgbd_view.launch.py with_depth_cloud:=false \
        seg_prompts:=box,bottle,person

rqt_image_view renders on the noVNC display :1 (DISPLAY auto-set by ~/.bashrc);
view it in the browser at http://<pc-ip>:22380/vnc.html
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_launch = os.path.join(
        get_package_share_directory('reachability_gng'),
        'launch', 'rgbd_perception.launch.py')

    # Pass-through args to rgbd_perception (so this wrapper stays a superset).
    # serial1/serial2 deliberately NOT declared/forwarded here -- this file
    # used to redeclare them with defaults that were REVERSED relative to
    # realsense_dual.launch.py's (241122302297/234222303079), and because
    # DeclareLaunchArgument only applies its default when the launch
    # configuration isn't already set, that wrong pair silently leaked down
    # through rgbd_perception.launch.py's include of realsense_dual.launch.py
    # -- same bug class already fixed once in rgbd_perception.launch.py itself
    # (see its comment). realsense_dual.launch.py is the single source of
    # truth for which camera is `rgbd`; override there, or pass serial1/serial2
    # straight through to THIS launch file's own IncludeLaunchDescription below
    # if you ever need a one-off override.
    _passthrough = ['with_depth_cloud', 'seg_source',
                    'seg_model', 'seg_device', 'seg_prompts', 'seg_conf',
                    'seg_imgsz']
    fwd = {k: LaunchConfiguration(k) for k in _passthrough}

    return LaunchDescription([
        DeclareLaunchArgument('with_depth_cloud', default_value='true'),
        DeclareLaunchArgument('seg_source', default_value='yoloe'),
        DeclareLaunchArgument('seg_model', default_value=os.path.expanduser(
            '~/Documents/ceiling_arm/ros2_ws/yoloe-11m-seg.pt')),
        DeclareLaunchArgument('seg_device', default_value='cuda:0'),
        DeclareLaunchArgument('seg_prompts',
                              default_value='box,bottle,cup,person,'
                                            'paper bag,bowl,doll,pan'),
        DeclareLaunchArgument('seg_conf', default_value='0.25'),
        DeclareLaunchArgument('seg_imgsz', default_value='768'),
        DeclareLaunchArgument('image_topic',
                              default_value='/rgbd/seg/debug_image'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments=fwd.items()),

        # Live detection overlay. The topic is passed positionally so rqt
        # auto-selects it (it appears once seg_router finishes loading YOLOE).
        Node(package='rqt_image_view', executable='rqt_image_view',
             name='rqt_image_view', output='screen',
             arguments=[LaunchConfiguration('image_topic')]),
    ])
