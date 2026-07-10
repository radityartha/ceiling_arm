"""One-terminal GNG perceiving-acting stack: perception + env map + fusion.

Bundles the three pieces the reach_fusion demo needs so they no longer each want
their own terminal:

    perception.launch.py   -> seg_router (seg_source:=isaac by default here, so
                              no manual `ros2 topic pub /seg_source` needed),
                              object_localizer, seg_cloud, ...  (/detected_objects,
                              /rgbd*/seg_cloud, raw instance labels)
    env_gng                -> /topo_map/markers  (GNG environment perception map)
    reach_fusion           -> /reach_fusion/markers  (collision-free fusion)

Run AFTER `./isaac_sim/launch_workcell.sh full` (Isaac + bridge + MoveIt + RViz).
The interactive target picker stays separate (it reads the keyboard):

    ros2 run reachability_gng target_cli

So the whole demo is 3 terminals: workcell, this launch, target_cli.

    ros2 launch reachability_gng topo_fusion.launch.py
    ros2 launch reachability_gng topo_fusion.launch.py target_label:=obj_4
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception = PathJoinSubstitution(
        [FindPackageShare('reachability_gng'), 'launch', 'perception.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_label', default_value='',
            description='initial reach_fusion target (obj_N / class / index)'),
        DeclareLaunchArgument(
            'seg_source', default_value='isaac',
            description='segmentation source (isaac ground truth by default)'),
        # perception on Isaac GT, WITHOUT the octomap/object-box nodes: the GNG
        # topological map replaces the octomap for MoveIt collision.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception),
            launch_arguments={
                'seg_source': LaunchConfiguration('seg_source'),
                'use_octomap': 'false'}.items()),
        Node(package='reachability_gng', executable='env_gng',
             name='env_gng', output='screen'),
        Node(package='reachability_gng', executable='reach_fusion',
             name='reach_fusion', output='screen',
             parameters=[{'target_label': LaunchConfiguration('target_label')}]),
        # GNG env nodes -> MoveIt collision spheres (replaces octomap)
        Node(package='reachability_gng', executable='gng_collision',
             name='gng_collision', output='screen'),
        # flush any residual octomap once (nothing feeds it now)
        TimerAction(period=6.0, actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/clear_octomap',
                 'std_srvs/srv/Empty'], output='screen')]),
    ])
