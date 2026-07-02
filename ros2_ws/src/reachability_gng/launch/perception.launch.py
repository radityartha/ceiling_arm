"""RGBD perception + reachability nodes that consume the Isaac cameras.

Starts:
    object_localizer        -> /detected_objects (+ /detected_objects/markers)
    reachability_check      -> /reachability/markers  (green/red CUBE + text)
    reachability_cloud      -> /reachability/voxels (per-voxel green/red, any shape)
    collision_cloud         -> /<ns>/collision_cloud (environment only, MoveIt octomap)
    object_collision        -> /planning_scene (objects as CollisionObjects + attach)
    octomap_refresher       -> /clear_octomap (flush stale arm voxels, ~1 Hz)
    seg_cloud               -> /<ns>/seg_cloud (full depth reading as a 3D cloud)
    static_collision        -> /planning_scene (mapped-once static geometry boxes)

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
          '|lib/reachability_gng/static_collision'
          '|lib/reachability_gng/table_slab')


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
        # stride=3 gives a denser cloud so the octomap fills more of the surface
        # (fewer holes) and follows the sensed shape. This was 6 earlier because a
        # racing /clear_octomap refresher couldn't keep up at stride 3; that
        # refresher now idles (period 60 s), so stride 3 populates fine. If the
        # octomap lags at 0.02 m resolution, raise stride back toward 4-6.
        Node(package='reachability_gng', executable='collision_cloud',
             name='collision_cloud', output='screen',
             parameters=target_params + [{'stride': 3}]),
        # detected objects -> exact CollisionObject boxes (+ attach/detach for grasp)
        Node(package='reachability_gng', executable='object_collision',
             name='object_collision', output='screen',
             parameters=target_params),
        # Safety-net only. collision_cloud now publishes in the camera optical
        # frame, so MoveIt ray-carves and clears moving-arm voxels incrementally
        # -- the whole-map wipe is no longer the primary cleaner (it was churning
        # the scene and invalidating plans, MoveGroup err=-3). period=60 s just
        # flushes any rare residue; set very large / remove once carving is trusted.
        Node(package='reachability_gng', executable='octomap_refresher',
             name='octomap_refresher', output='screen',
             parameters=[{'period': 60.0}]),
        # full depth reading -> 3D point cloud (table grey + objects coloured)
        Node(package='reachability_gng', executable='seg_cloud',
             name='seg_cloud', output='screen'),
        # table_slab (a solid thin table-surface CollisionObject) is intentionally
        # NOT autostarted -- user opted out (it covered too much). The node + entry
        # point remain available to run by hand if reconsidered:
        #   ros2 run reachability_gng table_slab
        # static_collision (mapped-once static-geometry BOXES) is DISABLED on
        # purpose: it is the prior-box path, and without an ROI map_static fits ONE
        # box over the whole z-slice -> a giant box filling the room (the "table
        # became huge" bug). We are on the pure sensor-driven octomap path instead.
        # To use it deliberately: map each piece with an ROI + name, then run it:
        #   ros2 run reachability_gng map_static --ros-args -p name:=work_table \
        #       -p roi:="[xmin, xmax, ymin, ymax]"
        #   ros2 run reachability_gng static_collision
        # Node(package='reachability_gng', executable='static_collision',
        #      name='static_collision', output='screen'),
    ])
