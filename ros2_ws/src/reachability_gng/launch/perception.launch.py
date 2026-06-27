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
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

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
    return LaunchDescription([
        OpaqueFunction(function=_kill_stale),
        Node(package='reachability_gng', executable='object_localizer',
             name='object_localizer', output='screen'),
        Node(package='reachability_gng', executable='reachability_check',
             name='reachability_check', output='screen'),
        Node(package='reachability_gng', executable='reachability_cloud',
             name='reachability_cloud', output='screen'),
        # environment depth (objects excluded) -> MoveIt octomap
        Node(package='reachability_gng', executable='collision_cloud',
             name='collision_cloud', output='screen'),
        # detected objects -> exact CollisionObject boxes (+ attach/detach for grasp)
        Node(package='reachability_gng', executable='object_collision',
             name='object_collision', output='screen'),
        # periodically clear the octomap so moving arms don't bake in as stale voxels
        Node(package='reachability_gng', executable='octomap_refresher',
             name='octomap_refresher', output='screen'),
        # full depth reading -> 3D point cloud (table grey + objects coloured)
        Node(package='reachability_gng', executable='seg_cloud',
             name='seg_cloud', output='screen'),
        # mapped-once static work table -> MoveIt collision box (idles if unmapped)
        Node(package='reachability_gng', executable='table_collision',
             name='table_collision', output='screen'),
    ])
