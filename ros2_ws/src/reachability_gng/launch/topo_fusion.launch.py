"""One-terminal GNG perceiving-acting stack: perception + env map + fusion.

Bundles the three pieces the reach_fusion demo needs so they no longer each want
their own terminal:

    perception.launch.py   -> seg_router (seg_source:=yoloe by default here, so
                              no manual `ros2 topic pub /seg_source` needed;
                              pass seg_source:=isaac for ground truth),
                              object_localizer, seg_cloud, ...  (/detected_objects,
                              /rgbd*/seg_cloud, instance labels)
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Per-frame numpy point-cloud work here (env_gng especially) spawns one BLAS
# thread per host core if left uncapped -- see perception.launch.py's
# _THREAD_CAP comment for the full story (72 threads/process measured, starved
# move_group's IK/plan calls). Same fix here.
_THREAD_CAP = {'additional_env': {
    'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
    'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}}


def generate_launch_description():
    perception = PathJoinSubstitution(
        [FindPackageShare('reachability_gng'), 'launch', 'perception.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_label', default_value='',
            description='initial reach_fusion target (obj_N / class / index)'),
        DeclareLaunchArgument(
            'seg_source', default_value='yoloe',
            description='segmentation source (yoloe open-vocab by default; '
                        'pass seg_source:=isaac for ground truth)'),
        # Static GNG background layer: empty (default) = original all-live map.
        # Pass static_map:=/tmp/topo_static.npz (after running map_topo_static
        # once on the empty scene) to get a reproducible fixed backbone + a live
        # map that only tracks the dynamic remainder.
        DeclareLaunchArgument(
            'static_map', default_value='',
            description='saved static GNG (map_topo_static output); '
                        'empty = whole-scene live map'),
        DeclareLaunchArgument(
            'bg_dist', default_value='0.08',
            description='drop live points within this of a static node (m)'),
        # Geometry source for the topo map. depth_cloud (default) deprojects RGBD
        # depth only -> the map is independent of seg_source (works even when the
        # detector is blind, e.g. YOLOE on sim imagery, and is the real-world
        # RGBD-only path). Pass topo_cloud:=seg_cloud for the old segmented cloud.
        DeclareLaunchArgument(
            'topo_cloud', default_value='depth_cloud',
            description='cloud topic suffix env_gng consumes '
                        '(depth_cloud = geometry-only; seg_cloud = segmented)'),
        # Stale-node pruning for the live (green) layer: drop dynamic nodes that
        # float farther than prune_dist from any live point, every prune_every
        # updates. Lower prune_dist / prune_every = more aggressive cleanup of
        # stray floating nodes + bridge edges left in free space.
        DeclareLaunchArgument(
            'prune_dist', default_value='0.10',
            description='delete live nodes floating > this from data (m)'),
        DeclareLaunchArgument(
            'prune_every', default_value='5',
            description='run the stale-node prune every N updates'),
        # Crop height for the topo map. Points above this are dropped BEFORE the
        # GNG, in BOTH the live map and (match it) the static capture. Set below
        # the ceiling gantry / arm-mount structure (~1.8 m) so the moving robot
        # body is never baked as static nor mapped live. Default 1.9 keeps the
        # old behaviour (crops only the platform top at 2.05).
        DeclareLaunchArgument(
            'max_z', default_value='1.9',
            description='drop topo-map points above this height (m)'),
        # Arm self-filter: filter the arm against the last N pose snapshots (its
        # recent swept path) to remove a MOVING arm despite cloud/TF lag; raise
        # if a moving/settling arm still flickers green. self_filter_radius is the
        # link-capsule radius (m).
        DeclareLaunchArgument(
            'self_filter_frames', default_value='6',
            description='filter arm against last N pose snapshots (swept path)'),
        DeclareLaunchArgument(
            'self_filter_radius', default_value='0.07',
            description='arm link-capsule filter radius (m)'),
        DeclareLaunchArgument(
            'finger_radius', default_value='0.05',
            description='gripper capsule filter radius (end_effector->fingers, m)'),
        # Kill any perception nodes left over from a PREVIOUS topo_fusion launch
        # BEFORE spawning ours -- two live env_gng/depth_cloud publishing to the
        # same topics makes the RViz map flicker/"double". `pkill` excludes its
        # own PID and the pattern doesn't match the `ros2 launch` process, so it
        # only reaps stale nodes; our own nodes spawn 3 s later (below), after it
        # has finished. Exits 1 when nothing matched -- harmless.
        ExecuteProcess(
            cmd=['pkill', '-9', '-f', 'lib/reachability_gng/'], output='screen'),
        # spawn everything only AFTER the cleanup has completed
        TimerAction(period=3.0, actions=[
            # perception WITHOUT the octomap/object-box nodes: the GNG topological
            # map replaces the octomap for MoveIt collision. seg_source defaults
            # to yoloe; the stable-track identity layer is source-agnostic so
            # isaac GT works identically with seg_source:=isaac.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception),
                launch_arguments={
                    'seg_source': LaunchConfiguration('seg_source'),
                    'use_octomap': 'false'}.items()),
            # geometry-only cloud (RGBD depth deproject, no segmentation gate)
            Node(package='reachability_gng', executable='depth_cloud',
                 name='depth_cloud', output='screen', **_THREAD_CAP),
            Node(package='reachability_gng', executable='env_gng',
                 name='env_gng', output='screen', **_THREAD_CAP,
                 parameters=[{'static_map': LaunchConfiguration('static_map'),
                              'bg_dist': ParameterValue(
                                  LaunchConfiguration('bg_dist'), value_type=float),
                              'cloud_topic_suffix': LaunchConfiguration('topo_cloud'),
                              'prune_dist': ParameterValue(
                                  LaunchConfiguration('prune_dist'), value_type=float),
                              'prune_every': ParameterValue(
                                  LaunchConfiguration('prune_every'), value_type=int),
                              'max_z': ParameterValue(
                                  LaunchConfiguration('max_z'), value_type=float),
                              'self_filter_frames': ParameterValue(
                                  LaunchConfiguration('self_filter_frames'), value_type=int),
                              'self_filter_radius': ParameterValue(
                                  LaunchConfiguration('self_filter_radius'), value_type=float),
                              'finger_radius': ParameterValue(
                                  LaunchConfiguration('finger_radius'), value_type=float)}]),
            # fixed background layer (no-op if static_map is empty / file missing)
            Node(package='reachability_gng', executable='topo_static_pub',
                 name='topo_static_pub', output='screen',
                 parameters=[{'map_file': LaunchConfiguration('static_map')}]),
            Node(package='reachability_gng', executable='reach_fusion',
                 name='reach_fusion', output='screen', **_THREAD_CAP,
                 parameters=[{'target_label': LaunchConfiguration('target_label')}]),
            # GNG env nodes -> MoveIt collision spheres (replaces octomap)
            Node(package='reachability_gng', executable='gng_collision',
                 name='gng_collision', output='screen', **_THREAD_CAP),
            # flush any residual octomap once (nothing feeds it now)
            TimerAction(period=6.0, actions=[ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/clear_octomap',
                     'std_srvs/srv/Empty'], output='screen')]),
        ]),
    ])
