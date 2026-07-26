"""RGBD perception + reachability nodes that consume the Isaac cameras.

Starts:
    seg_router              -> /<ns>/seg/instance_segmentation* (source: isaac|yoloe)
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
from launch.conditions import IfCondition
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
          '|lib/reachability_gng/seg_router'
          '|lib/reachability_gng/static_collision'
          '|lib/reachability_gng/table_slab')

# Cameras whose segmentation contract is routed through seg_router. The seg
# consumers below are remapped from Isaac's raw /<ns>/instance_segmentation* to
# the neutral /<ns>/seg/instance_segmentation* that seg_router publishes, so the
# active source (Isaac ground truth vs YOLOE) can be switched live on /seg_source
# without relaunching anything.
_CAMERA_NS = ['rgbd', 'rgbd2']
_SEG_REMAP = []
for _ns in _CAMERA_NS:
    _SEG_REMAP += [
        (f'/{_ns}/instance_segmentation', f'/{_ns}/seg/instance_segmentation'),
        (f'/{_ns}/instance_segmentation_labels',
         f'/{_ns}/seg/instance_segmentation_labels'),
        (f'/{_ns}/instance_segmentation_conf',
         f'/{_ns}/seg/instance_segmentation_conf'),
    ]


def _kill_stale(context, *args, **kwargs):
    subprocess.run(['pkill', '-9', '-f', _STALE],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)  # let DDS discovery drop the killed nodes
    return []


# Silence the background perception nodes to WARN so the shared pick terminal
# shows the executor's pick pipeline (target, J, arm, plan, success) instead of
# per-frame chatter. reachability_cloud stays at INFO -- it prints the per-object
# "% reachable", now only when it changes. Raise any node with `-p`/relaunch, or
# `ros2 run <node> --ros-args --log-level info` if you need its detail back.
_QUIET = {'ros_arguments': ['--log-level', 'WARN']}

# Every node here does per-frame numpy/point-cloud work; unconstrained, each
# process's BLAS backend spawns one thread per host core (measured: 72 threads
# on a 64-core box). With this many nodes running concurrently that starves
# move_group of CPU (IK/plan calls that normally take <1s start timing out).
# Capping each process to 1 BLAS thread fixes it -- these nodes' per-call
# arrays are small, so single-threaded is also faster than the spawn overhead.
_THREAD_CAP = {'additional_env': {
    'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
    'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}}


def generate_launch_description():
    # Single control point for which object is the grasp TARGET. Empty ->
    # legacy behaviour (every object boxed + kept out of the octomap). When set,
    # only the target is boxed/attached + excluded from the octomap; other
    # objects stay as octomap obstacles. target_id is a per-camera seg-id
    # fallback used when the label is absent in a camera.
    target_label = LaunchConfiguration('target_label')
    target_id = ParameterValue(LaunchConfiguration('target_id'), value_type=int)
    target_params = [{'target_label': target_label, 'target_id': target_id}]
    # Segmentation source router: 'yoloe' (DEFAULT, open-vocab YOLOE on /<ns>/rgb)
    # or 'isaac' (relay Isaac ground-truth segmentation). Switch live with either
    # pick_cli (y/i) or:  ros2 topic pub -1 /seg_source std_msgs/String "data: isaac"
    seg_source = LaunchConfiguration('seg_source')
    seg_prompts = ParameterValue(LaunchConfiguration('seg_prompts'),
                                 value_type=str)
    seg_conf = ParameterValue(LaunchConfiguration('seg_conf'), value_type=float)
    seg_imgsz = ParameterValue(LaunchConfiguration('seg_imgsz'), value_type=int)
    carve_target = ParameterValue(LaunchConfiguration('carve_target'),
                                  value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('target_label', default_value=''),
        DeclareLaunchArgument('target_id', default_value='-1'),
        # true (grasp mode): carve the target out of the octomap so the gripper
        # can reach it. false (approach-only): keep the target in the octomap as a
        # hard obstacle so the arm stands off above it without touching it.
        DeclareLaunchArgument('carve_target', default_value='true'),
        # false -> skip the octomap/object-box collision nodes (collision_cloud,
        # object_collision, octomap_refresher). Use with the GNG collision path
        # (gng_collision) so the topological map replaces the octomap.
        DeclareLaunchArgument('use_octomap', default_value='true'),
        DeclareLaunchArgument('seg_source', default_value='yoloe'),
        DeclareLaunchArgument('seg_model', default_value='yoloe-11m-seg.pt'),
        DeclareLaunchArgument('seg_device', default_value=''),   # '' -> auto
        # Default open-vocab classes for the workcell scene (also the labels you
        # target in pick_cli). Change live with pick_cli `p ...` or /seg_prompts.
        DeclareLaunchArgument('seg_prompts',
                              default_value='box,tin can,canned food,bottle,'
                                            'banana,teddy bear,mug,glass beaker,'
                                            'paper bag,bowl,doll,pan'),
        # 0.25 drops weak/wrong labels; object_localizer's tracking + label
        # voting bridge the rest. Lower toward 0.1 if real objects get missed.
        DeclareLaunchArgument('seg_conf', default_value='0.25'),
        # Higher inference resolution = better accuracy on small objects (slower).
        # 768 vs 640 is a modest cost; raise to 1024 for max accuracy.
        DeclareLaunchArgument('seg_imgsz', default_value='768'),
        OpaqueFunction(function=_kill_stale),
        # Publishes /<ns>/seg/instance_segmentation* (the neutral contract the
        # consumers below are remapped to). prompts is a comma string here and is
        # split by the node; change it live on /seg_prompts.
        Node(package='reachability_gng', executable='seg_router',
             name='seg_router', output='screen', **_QUIET, **_THREAD_CAP,
             parameters=[{'source': seg_source,
                          'camera_namespaces': _CAMERA_NS,
                          'model_path': LaunchConfiguration('seg_model'),
                          'device': LaunchConfiguration('seg_device'),
                          'conf': seg_conf,
                          'imgsz': seg_imgsz,
                          'prompts': seg_prompts}]),
        Node(package='reachability_gng', executable='object_localizer',
             name='object_localizer', output='screen', **_QUIET, **_THREAD_CAP,
             parameters=target_params, remappings=_SEG_REMAP),
        Node(package='reachability_gng', executable='reachability_check',
             name='reachability_check', output='screen', **_QUIET,
             condition=IfCondition(LaunchConfiguration('use_octomap'))),
        # old reach-voxel viz (spammy per-frame INFO); not needed on the GNG path.
        Node(package='reachability_gng', executable='reachability_cloud',
             name='reachability_cloud', output='screen', remappings=_SEG_REMAP,
             condition=IfCondition(LaunchConfiguration('use_octomap'))),
        # environment depth (objects excluded) -> MoveIt octomap.
        # stride=3 gives a denser cloud so the octomap fills more of the surface
        # (fewer holes) and follows the sensed shape. This was 6 earlier because a
        # racing /clear_octomap refresher couldn't keep up at stride 3; that
        # refresher now idles (period 60 s), so stride 3 populates fine. If the
        # octomap lags at 0.02 m resolution, raise stride back toward 4-6.
        Node(package='reachability_gng', executable='collision_cloud',
             name='collision_cloud', output='screen', **_QUIET, **_THREAD_CAP,
             condition=IfCondition(LaunchConfiguration('use_octomap')),
             parameters=target_params + [{'stride': 3,
                                          'carve_target': carve_target}],
             remappings=_SEG_REMAP),
        # detected objects -> exact CollisionObject boxes (+ attach/detach for grasp)
        Node(package='reachability_gng', executable='object_collision',
             name='object_collision', output='screen', **_QUIET,
             condition=IfCondition(LaunchConfiguration('use_octomap')),
             parameters=target_params, remappings=_SEG_REMAP),
        # Safety-net only. collision_cloud now publishes in the camera optical
        # frame, so MoveIt ray-carves and clears moving-arm voxels incrementally
        # -- the whole-map wipe is no longer the primary cleaner (it was churning
        # the scene and invalidating plans, MoveGroup err=-3). period=60 s just
        # flushes any rare residue; set very large / remove once carving is trusted.
        Node(package='reachability_gng', executable='octomap_refresher',
             name='octomap_refresher', output='screen', **_QUIET,
             condition=IfCondition(LaunchConfiguration('use_octomap')),
             parameters=[{'period': 60.0}]),
        # full depth reading -> 3D point cloud (table grey + objects coloured)
        Node(package='reachability_gng', executable='seg_cloud',
             name='seg_cloud', output='screen', **_QUIET, **_THREAD_CAP,
             remappings=_SEG_REMAP),
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
