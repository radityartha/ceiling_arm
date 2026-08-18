"""All three layers of the cell in ONE RViz -- one command, one terminal.

    camera colour cloud   what the room LOOKS like   live, needs the 2 D455s
    static GNG topo map   what was MAPPED of it      saved .npz, no cameras
    capability map        where an arm can REACH     saved .npz, no hardware

Everything runs in one process tree, so a single Ctrl-C stops all of it (no
orphan nodes -- an orphan color_cloud or a stale static_transform_publisher is
the usual cause of "RViz shows the layout in the wrong place").

    # cameras + both saved maps (the full picture)
    ros2 launch reachability_gng view_scene.launch.py

    # maps only, no cameras (nothing to plug in, the maps are files)
    ros2 launch reachability_gng view_scene.launch.py with_cameras:=false

    # a different arm / a different rail position / the pose-independent index
    ros2 launch reachability_gng view_scene.launch.py arm:=arm_2 lin:=0.9
    ros2 launch reachability_gng view_scene.launch.py mode:=index

`lin` is the gantry rail position in METRES and it is not cosmetic: the
reachable set moves with the rail, so the default 0.55 draws the wrong
workspace if the gantry is not there. Read it off the machine first --

    ros2 topic echo /joint_states --once | grep -A30 t1_linear

-- and note /joint_states has two publishers, so `--once` is a coin flip; echo
without --once if the numbers look wrong.

with_cameras:=true starts the RealSense drivers. Do NOT use it if anything else
already has them: two drivers fighting over one USB device knocks BOTH cameras
offline. The maps need no cameras at all, so with_cameras:=false is the safe
default choice when something else is already running.

This does not draw the robot itself -- robot_description comes from
my_workcell.launch.py; run that alongside and add a RobotModel display if you
want the arms in the picture.

RViz renders on the noVNC display :1 (DISPLAY is auto-set by ~/.bashrc); view it
in the browser at http://<pc-ip>:22380/vnc.html
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


def generate_launch_description():
    share = get_package_share_directory('reachability_gng')

    return LaunchDescription([
        # The dual-camera capture of the cleared scene, 2026-08-13 (docs/
        # p1_g6_map.md B). `a` and `b` are the two captures of that session.
        DeclareLaunchArgument('map_file', default_value='/tmp/topo_static_a.npz',
                              description='saved static GNG map (.npz)'),
        DeclareLaunchArgument('with_cameras', default_value='true',
                              description='start the 2 RealSense drivers'),
        DeclareLaunchArgument('arm', default_value='arm_1',
                              description='arm_1 | arm_2 | arm_3 | arm_4'),
        DeclareLaunchArgument('lin', default_value='0.55',
                              description='MEASURED rail position, metres'),
        DeclareLaunchArgument('rot', default_value='0.0',
                              description='gantry rotation, degrees'),
        DeclareLaunchArgument('mode', default_value='pose',
                              description='pose (at lin/rot) | index (all poses)'),
        DeclareLaunchArgument('cap_map_file', default_value='',
                              description="'' -> the repo copy for that gantry"),

        # 1) cameras -> /<ns>/color_cloud (color_cloud is started in there).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'realsense_dual.launch.py')),
            launch_arguments={'with_color_cloud': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('with_cameras'))),

        # 2) saved static GNG map -> /topo_map/static/markers
        Node(package='reachability_gng', executable='topo_static_pub',
             name='topo_static_pub', output='screen',
             parameters=[{'map_file': LaunchConfiguration('map_file')}]),

        # 3) capability map -> /capability/markers
        Node(package='reachability_gng', executable='capability_pub',
             name='capability_pub', output='screen',
             parameters=[{'arm': LaunchConfiguration('arm'),
                          # lin/rot are declared as doubles in the node;
                          # a launch argument arrives as a string, so it has to
                          # be typed here or the node rejects it at startup.
                          'lin': ParameterValue(LaunchConfiguration('lin'),
                                                value_type=float),
                          'rot': ParameterValue(LaunchConfiguration('rot'),
                                                value_type=float),
                          'mode': LaunchConfiguration('mode'),
                          'map_file': LaunchConfiguration('cap_map_file')}]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'config', 'scene.rviz')]),
    ])
