"""One-command RGBD bring-up for the REAL-hardware topo-mapping workflow.

Starts, in one process tree (single Ctrl-C stops everything -- no orphan nodes):

    realsense_dual.launch.py   2x RealSense D455 -> /rgbd*, /rgbd2* + world TF
    seg_router (YOLOE)         -> /<ns>/seg/debug_image + /<ns>/seg/instance_*
    depth_cloud                -> /<ns>/depth_cloud   (geometry for map_topo_static)
    color_cloud                -> /<ns>/color_cloud   (true-colour cloud, viz only)

This is the LIGHT stack: just what you need to (a) eyeball detection on
/rgbd/seg/debug_image and (b) feed build_topo.sh / map_topo_static. It does NOT
start the heavy pick stack (object_localizer, reachability, octomap) -- that's
perception.launch.py, which also assumes the Isaac cameras.

    ros2 launch reachability_gng rgbd_perception.launch.py
    ros2 launch reachability_gng rgbd_perception.launch.py with_depth_cloud:=false
    ros2 launch reachability_gng rgbd_perception.launch.py with_color_cloud:=true
    ros2 launch reachability_gng rgbd_perception.launch.py \
        seg_prompts:=box,bottle,person seg_device:=cuda:0 serial1:=234222303079

color_cloud is off by default (viz-only, not needed by map_topo_static); add a
PointCloud2 display on /rgbd/color_cloud + /rgbd2/color_cloud in RViz to see it.

Then view the detection overlay (DISPLAY :1 via noVNC is auto-set by ~/.bashrc):
    ros2 run rqt_image_view rqt_image_view /rgbd/seg/debug_image
And build a static topo map from one camera:
    ros2_ws/src/reachability_gng/build_topo.sh rgbd
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Absolute default so YOLOE() resolves the weights regardless of launch cwd.
_DEFAULT_MODEL = os.path.expanduser(
    '~/Documents/ceiling_arm/ros2_ws/yoloe-11m-seg.pt')
_CAMERA_NS = ['rgbd', 'rgbd2']


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory('reachability_gng'),
        'launch', 'realsense_dual.launch.py')

    with_depth_cloud = LaunchConfiguration('with_depth_cloud')
    seg_source = LaunchConfiguration('seg_source')
    seg_model = LaunchConfiguration('seg_model')
    seg_device = LaunchConfiguration('seg_device')
    seg_prompts = ParameterValue(LaunchConfiguration('seg_prompts'),
                                 value_type=str)
    seg_conf = ParameterValue(LaunchConfiguration('seg_conf'), value_type=float)
    seg_imgsz = ParameterValue(LaunchConfiguration('seg_imgsz'), value_type=int)

    return LaunchDescription([
        # NOTE: serial1/serial2 are deliberately NOT declared here. They used
        # to be, with their own hardcoded defaults, which silently OVERRODE
        # realsense_dual.launch.py's -- so fixing the serial<->namespace
        # mapping there had no effect when coming through this file (the two
        # copies had drifted to opposite pairings). realsense_dual.launch.py
        # is the single source of truth for which camera is `rgbd`; override
        # there, or launch it directly.
        DeclareLaunchArgument('enable1', default_value='true'),
        DeclareLaunchArgument('enable2', default_value='true'),
        DeclareLaunchArgument('with_depth_cloud', default_value='true'),
        DeclareLaunchArgument('with_color_cloud', default_value='false'),
        DeclareLaunchArgument('seg_source', default_value='yoloe'),
        DeclareLaunchArgument('seg_model', default_value=_DEFAULT_MODEL),
        DeclareLaunchArgument('seg_device', default_value='cuda:0'),
        DeclareLaunchArgument('seg_prompts',
                              default_value='box,bottle,cup,person,'
                                            'paper bag,bowl,doll,pan'),
        DeclareLaunchArgument('seg_conf', default_value='0.25'),
        DeclareLaunchArgument('seg_imgsz', default_value='768'),

        # 1) cameras (+ world->camera static TF). Serials intentionally not
        # forwarded -- see the note above; realsense_dual owns them.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={'enable1': LaunchConfiguration('enable1'),
                              'enable2': LaunchConfiguration('enable2')}.items()),

        # 2) YOLOE segmentation -> /<ns>/seg/debug_image + instance masks.
        Node(package='reachability_gng', executable='seg_router',
             name='seg_router', output='screen',
             parameters=[{'source': seg_source,
                          'camera_namespaces': _CAMERA_NS,
                          'model_path': seg_model,
                          'device': seg_device,
                          'conf': seg_conf,
                          'imgsz': seg_imgsz,
                          'prompts': seg_prompts}]),

        # 3) geometric world-frame cloud for map_topo_static (optional).
        Node(package='reachability_gng', executable='depth_cloud',
             name='depth_cloud', output='screen',
             condition=IfCondition(with_depth_cloud)),

        # 4) true-colour world-frame cloud for visualization (optional).
        Node(package='reachability_gng', executable='color_cloud',
             name='color_cloud', output='screen',
             condition=IfCondition(LaunchConfiguration('with_color_cloud'))),
    ])
