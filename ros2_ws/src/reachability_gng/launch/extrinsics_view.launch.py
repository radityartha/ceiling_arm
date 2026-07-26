"""Check the world->camera extrinsics in RViz -- one command, one terminal.

Brings up everything needed to SEE whether the two RGBD cameras are
calibrated, and nothing else (no YOLOE/seg_router, which is heavy and
irrelevant here):

    realsense_dual.launch.py  2x D455 -> /rgbd*, /rgbd2* + world->cam static TF
    robot_state_publisher     workcell URDF -> t1_base_link / t2_base_link TF,
                              so a camera frame can be judged against the
                              gantry it is supposed to sit beside
    color_cloud               /<ns>/color_cloud, true-colour cloud in `world`
    rviz2                     config/extrinsics_view.rviz

    ros2 launch reachability_gng extrinsics_view.launch.py
    ros2 launch reachability_gng extrinsics_view.launch.py with_rviz:=false

What to look for, in order:
  1. TF sanity -- `rgbd_camera_optical` must sit beside `t1_base_link` (+Y)
     and `rgbd2_camera_optical` beside `t2_base_link` (-Y). If they are
     swapped, the tf1_*/tf2_* blocks in realsense_dual.launch.py are in the
     wrong order (that is a labelling bug, not a calibration one).
  2. Cloud agreement -- the two clouds must land on the SAME floor/walls.
     Two offset copies of the room = the extrinsics are genuinely wrong;
     re-run calibrate_extrinsics.py, then verify_extrinsics.py (<5 cm).

Renders on the noVNC display :1 (DISPLAY auto-set by ~/.bashrc); view at
http://<pc-ip>:22380/vnc.html

** If RViz shows a layout matching NO version of realsense_dual.launch.py,
suspect stale static_transform_publisher processes from an earlier run
before suspecting the config: they are not killed by stopping the camera
nodes alone, they accumulate, and several publishing different values to the
same frame makes tf2 return whichever won. `ps aux | grep
static_transform_publisher` should show exactly two. **
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('reachability_gng')
    realsense_launch = os.path.join(pkg, 'launch', 'realsense_dual.launch.py')
    rviz_config = os.path.join(pkg, 'config', 'extrinsics_view.rviz')
    workcell_xacro = os.path.join(
        get_package_share_directory('workcell_description'),
        'urdf', 'workcell.urdf.xacro')

    # use_fake_hardware:=true so the URDF expands without any arm/gantry
    # driver running -- we only want the link tree for its TF, never control.
    robot_description = ParameterValue(
        Command(['xacro ', workcell_xacro, ' use_fake_hardware:=true']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('with_rviz', default_value='true'),
        DeclareLaunchArgument('with_robot_tf', default_value='true',
                              description='publish t1/t2 gantry frames to '
                                          'compare the camera frames against'),
        DeclareLaunchArgument('enable1', default_value='true'),
        DeclareLaunchArgument('enable2', default_value='true'),

        # Reap static_transform_publishers left behind by an earlier run of
        # this or realsense_dual: stopping the camera nodes does not take
        # them with it, and duplicates publishing different values to the
        # same world->*_camera_optical frame make RViz show a layout that
        # matches nothing. Excludes its own PID; exits 1 when nothing
        # matched, which is harmless.
        ExecuteProcess(
            cmd=['pkill', '-9', '-f', 'static_transform_publisher.*camera_optical'],
            output='screen'),

        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    'enable1': LaunchConfiguration('enable1'),
                    'enable2': LaunchConfiguration('enable2')}.items()),

            Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 condition=IfCondition(LaunchConfiguration('with_robot_tf')),
                 parameters=[{'robot_description': robot_description}]),

            Node(package='reachability_gng', executable='color_cloud',
                 name='color_cloud', output='screen'),

            # RViz last: the cameras need a moment to publish camera_info
            # before color_cloud emits anything, so starting it earlier just
            # shows an empty scene and invites a "nothing works" conclusion.
            TimerAction(period=6.0, actions=[
                Node(package='rviz2', executable='rviz2', name='rviz2',
                     output='screen',
                     condition=IfCondition(LaunchConfiguration('with_rviz')),
                     arguments=['-d', rviz_config]),
            ]),
        ]),
    ])
