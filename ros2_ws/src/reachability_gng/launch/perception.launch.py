"""RGBD perception + reachability nodes that consume the Isaac cameras.

Starts:
    object_localizer        -> /detected_objects (+ /detected_objects/markers)
    reachability_check      -> /reachability/markers  (green/red CUBE + text)
    reachability_cloud      -> /reachability/voxels (per-voxel green/red, any shape)
    collision_cloud         -> /<ns>/collision_cloud (environment only, MoveIt octomap)
    object_collision        -> /planning_scene (objects as CollisionObjects + attach)
    octomap_refresher       -> /clear_octomap (flush stale arm voxels, ~1 Hz)
    seg_cloud               -> /<ns>/seg_cloud (full depth reading as a 3D cloud)
    table_collision         -> /planning_scene (mapped-once static work table box)

Expects the Isaac bridge (cameras /rgbd*, /rgbd2*) and the world->camera static
TFs to already be up (launch_workcell.sh provides both). The seg_colorizer 2D
colour-mask helper is NOT started here -- run it by hand if you want it.

    source ros2_ws/install/setup.bash
    ros2 launch reachability_gng perception.launch.py
"""
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_STALE = ('lib/reachability_gng/object_localizer'
          '|lib/reachability_gng/reachability_check'
          '|lib/reachability_gng/reachability_cloud'
          '|lib/reachability_gng/collision_cloud'
          '|lib/reachability_gng/object_collision'
          '|lib/reachability_gng/octomap_refresher'
          '|lib/reachability_gng/seg_cloud'
          '|lib/reachability_gng/table_collision')


def _kill_stale(context, *args, **kwargs):
    subprocess.run(['pkill', '-9', '-f', _STALE],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)  # let DDS discovery drop the killed nodes
    return []


def generate_launch_description():
    # Single control point for which object is the grasp TARGET. Empty ->
    # legacy behaviour (every object boxed + kept out of the octomap). When set,
    # only the target is boxed/attached + excluded from the octomap; other
    # objects stay as octomap obstacles. target_id is a per-camera seg-id
    # fallback used when the label is absent in a camera.
    target_label = LaunchConfiguration('target_label')
    target_id = ParameterValue(LaunchConfiguration('target_id'), value_type=int)
    target_params = [{'target_label': target_label, 'target_id': target_id}]
    return LaunchDescription([
        DeclareLaunchArgument('target_label', default_value=''),
        DeclareLaunchArgument('target_id', default_value='-1'),
        OpaqueFunction(function=_kill_stale),
        Node(package='reachability_gng', executable='object_localizer',
             name='object_localizer', output='screen',
             parameters=target_params),
        Node(package='reachability_gng', executable='reachability_check',
             name='reachability_check', output='screen'),
        Node(package='reachability_gng', executable='reachability_cloud',
             name='reachability_cloud', output='screen'),
        # environment depth (objects excluded) -> MoveIt octomap.
        # stride=6 (was 3) keeps the cloud small so MoveIt's octomap updater
        # processes it in ~0.25 s instead of ~1 s -- at ~1 s/cloud it could not
        # keep up and the octomap stayed empty/stale (saturated + raced clears).
        Node(package='reachability_gng', executable='collision_cloud',
             name='collision_cloud', output='screen',
             parameters=target_params + [{'stride': 6}]),
        # detected objects -> exact CollisionObject boxes (+ attach/detach for grasp)
        Node(package='reachability_gng', executable='object_collision',
             name='object_collision', output='screen',
             parameters=target_params),
        # periodically clear the octomap so moving arms don't bake in as stale
        # voxels. period=5 s (was 1 s default): clearing the WHOLE map every 1 s
        # outpaced the ~1 s rebuild, so the planner almost always saw an EMPTY
        # octomap and drove the arm through the table. 5 s keeps the map populated
        # while still flushing stale arm trails (no ray-carving in world frame).
        Node(package='reachability_gng', executable='octomap_refresher',
             name='octomap_refresher', output='screen',
             parameters=[{'period': 5.0}]),
        # full depth reading -> 3D point cloud (table grey + objects coloured)
        Node(package='reachability_gng', executable='seg_cloud',
             name='seg_cloud', output='screen'),
        # NOTE: table_collision (a mapped-once static work-table BOX) is disabled
        # on purpose: map_table fits an oversized/wrong box and the work table is
        # already captured by the live octomap (collision_cloud) as voxels, which
        # looks cleaner. Re-enable by uncommenting + running map_table on a clear
        # table:
        # Node(package='reachability_gng', executable='table_collision',
        #      name='table_collision', output='screen'),
    ])
