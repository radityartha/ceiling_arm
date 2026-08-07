"""Energy-aware gantry pick: arm selection + base placement via the GNG maps.

Starts the gantry_reach_executor, which on each `~/pick` chooses arm_1 vs arm_2
and the 8-DOF goal config by energy (J), then asks MoveIt to plan (and, with
execute:=true, execute) collision-free against the live octomap.

Assumes ALREADY running (this launch does NOT start them):
    * move_group  (/compute_ik + move_action)      -> my_workcell.launch.py
    * perception  (/detected_objects + octomap)    -> perception.launch.py

    ros2 launch reachability_gng gantry_pick.launch.py
    ros2 launch reachability_gng gantry_pick.launch.py execute:=true csv:=/tmp/picks.csv
    ros2 topic pub --once /gantry_reach_executor/pick std_msgs/String "{data: '0'}"

Override model paths (rare) via -p, e.g.:
    ros2 run reachability_gng gantry_reach_executor --ros-args \
        -p arm_models:='[/path/a1.npz,/path/a2.npz]'
"""
import subprocess
import time
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Per-frame numpy point-cloud work spawns one BLAS thread per host core if left
# uncapped -- see topo_fusion.launch.py's _THREAD_CAP comment for the full story
# (72 threads/process measured, starved move_group's IK/plan calls). Same fix,
# applied only to the wrist depth_cloud instance added below.
_THREAD_CAP = {'additional_env': {
    'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
    'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}}

_STALE = 'lib/reachability_gng/gantry_reach_executor'


def _kill_stale(context, *args, **kwargs):
    subprocess.run(['pkill', '-9', '-f', _STALE],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    return []


def generate_launch_description():
    execute = LaunchConfiguration('execute')
    csv = LaunchConfiguration('csv')
    box_clearance = LaunchConfiguration('box_clearance')
    allow_target_collision = LaunchConfiguration('allow_target_collision')
    compute_traj_energy = LaunchConfiguration('compute_traj_energy')
    do_grasp = LaunchConfiguration('do_grasp')
    attach_object_id = LaunchConfiguration('attach_object_id')
    grasp_descend = LaunchConfiguration('grasp_descend')
    lift_height = LaunchConfiguration('lift_height')
    place_enabled = LaunchConfiguration('place_enabled')
    place_x = LaunchConfiguration('place_x')
    place_y = LaunchConfiguration('place_y')
    place_z = LaunchConfiguration('place_z')
    wrist_cloud = LaunchConfiguration('wrist_cloud')

    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false',
                              description='true = plan AND execute; '
                                          'false = plan-only (benchmark)'),
        DeclareLaunchArgument('csv', default_value='',
                              description='per-pick CSV log path (empty = off)'),
        DeclareLaunchArgument('box_clearance', default_value='0.05',
                              description='EE stand-off (m) above the target '
                                          "box top; also tunable live via "
                                          '`ros2 param set`'),
        DeclareLaunchArgument('allow_target_collision', default_value='true',
                              description='true = ACM-allow the gripper to touch '
                                          'the target box (grasp mode). false = '
                                          'approach-only: target stays a hard '
                                          'obstacle, gripper never enters it'),
        DeclareLaunchArgument('compute_traj_energy', default_value='false',
                              description='true = compute per-pick mechanical '
                                          'energy (Pinocchio inverse dynamics) '
                                          'for the CSV traj_energy column '
                                          '(needs arm_pin_configs; adds latency)'),
        DeclareLaunchArgument('do_grasp', default_value='false',
                              description='true = run the FULL pick-place cycle '
                                          '(descend, close gripper, attach, lift) '
                                          'after a successful approach; false = '
                                          'approach-only (prior behaviour)'),
        DeclareLaunchArgument('attach_object_id', default_value='',
                              description='scene CollisionObject id to attach on '
                                          'grasp (empty = physical grasp only, no '
                                          'MoveIt attach bookkeeping)'),
        DeclareLaunchArgument('grasp_descend', default_value='0.05',
                              description='m to lower the EE from the pre-grasp '
                                          'pose to actually enclose the object'),
        DeclareLaunchArgument('lift_height', default_value='0.15',
                              description='m to raise the EE after closing the '
                                          'gripper'),
        DeclareLaunchArgument('place_enabled', default_value='false',
                              description='true = transport to (place_x, place_y, '
                                          'place_z), open, detach, retreat after '
                                          'the lift; false = hold at lift height '
                                          'above the pick point'),
        DeclareLaunchArgument('place_x', default_value='0.0',
                              description='world x (m) drop point'),
        DeclareLaunchArgument('place_y', default_value='0.0',
                              description='world y (m) drop point'),
        DeclareLaunchArgument('place_z', default_value='0.0',
                              description='world z (m) drop point'),
        DeclareLaunchArgument('wrist_cloud', default_value='false',
                              description='true = also start a depth_cloud '
                                          'instance for wrist1 (session A2 '
                                          "~/look acquisition; SEPARATE from "
                                          "topo_fusion's ceiling-camera "
                                          'depth_cloud, tuned for the D405 '
                                          "close-range sweet spot). Requires "
                                          'launch_workcell.sh with the wrist '
                                          'camera (grasping-phase-1 checkout).'),
        OpaqueFunction(function=_kill_stale),
        Node(
            package='reachability_gng',
            executable='gantry_reach_executor',
            name='gantry_reach_executor',
            output='screen',
            parameters=[{
                # plain string lists -> STRING_ARRAY (do NOT use LaunchConfig in
                # a list: launch would concatenate them into one string).
                # All 4 arms / both gantries (this workcell's actual target
                # config -- NOT the gantry_1-only subset from earlier single-
                # gantry testing). Requires launch_workcell.sh in `full` mode
                # (default/`gng` mode hides gantry_2 + arm_3/arm_4 in Isaac).
                'arm_names': ['arm_1', 'arm_2', 'arm_3', 'arm_4'],
                'arm_models': ['/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm1_model.npz',
                              '/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm2_model.npz',
                              '/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm3_model.npz',
                              '/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm4_model.npz'],
                'arm_groups': ['gantry_1_with_arm_1', 'gantry_1_with_arm_2',
                              'gantry_2_with_arm_1', 'gantry_2_with_arm_2'],
                'arm_ee_frames': ['t1_a1_tool_frame', 't1_a2_tool_frame',
                                 't2_a1_tool_frame', 't2_a2_tool_frame'],
                'gripper_links': ['t1_a1_gripper_base_link',
                                  't1_a2_gripper_base_link',
                                  't2_a1_gripper_base_link',
                                  't2_a2_gripper_base_link'],
                # Same configs build_maps.sh uses (absolute paths -- resolved
                # regardless of this process's CWD). Enables optional per-pick
                # trajectory energy (Pinocchio inverse dynamics) for CSV logging.
                'arm_pin_configs': [
                    '/srv/data/users/raditya/arm_WS/ceiling_arm/ros2_ws/src/'
                    'reachability_gng/config/arm1_table1.yaml',
                    '/srv/data/users/raditya/arm_WS/ceiling_arm/ros2_ws/src/'
                    'reachability_gng/config/arm2_table1.yaml',
                    '/srv/data/users/raditya/arm_WS/ceiling_arm/ros2_ws/src/'
                    'reachability_gng/config/arm3_table2.yaml',
                    '/srv/data/users/raditya/arm_WS/ceiling_arm/ros2_ws/src/'
                    'reachability_gng/config/arm4_table2.yaml',
                ],
                # Energy weights (w_gantry_lin/w_gantry_rot/w_arm/w_dist/w_manip)
                # live solely in the node's declare_parameter defaults -- single
                # source of truth. Override live with -p w_*:=... if needed.
                # value_type coerces the "true"/"false" string to a real bool.
                'execute': ParameterValue(execute, value_type=bool),
                'csv_log': csv,
                'box_clearance': ParameterValue(box_clearance,
                                                value_type=float),
                'allow_target_collision': ParameterValue(
                    allow_target_collision, value_type=bool),
                'compute_traj_energy': ParameterValue(
                    compute_traj_energy, value_type=bool),
                'gripper_names': ['gripper_1', 'gripper_2',
                                 'gripper_3', 'gripper_4'],
                'gripper_joints': ['t1_a1_right_finger_bottom_joint',
                                  't1_a2_right_finger_bottom_joint',
                                  't2_a1_right_finger_bottom_joint',
                                  't2_a2_right_finger_bottom_joint'],
                'do_grasp': ParameterValue(do_grasp, value_type=bool),
                'attach_object_id': attach_object_id,
                'grasp_descend': ParameterValue(grasp_descend, value_type=float),
                'lift_height': ParameterValue(lift_height, value_type=float),
                'place_enabled': ParameterValue(place_enabled, value_type=bool),
                # Each LaunchConfiguration wrapped in its OWN list: a flat list
                # of substitutions is_substitution()==True at the top level and
                # gets CONCATENATED into one string ("1.60.7751.025", verified
                # empirically -- it does not raise, it silently produces the
                # wrong value), not treated as 3 array elements. Nesting each in
                # a length-1 list forces the per-element coercion path instead.
                'place_xyz': ParameterValue(
                    [[place_x], [place_y], [place_z]], value_type=List[float]),
            }],
        ),
        # wrist1 geometry feed for ~/look (session A2). min_depth/max_depth
        # mirror the wrist camera's OWN clipping range (0.05-1.5 m, see
        # ros2_bridge_gui.py's _add_wrist_camera) rather than the ceiling
        # depth_cloud's defaults (0.1-12.0 m, sized for the whole room).
        Node(
            package='reachability_gng',
            executable='depth_cloud',
            name='depth_cloud_wrist',
            output='screen',
            condition=IfCondition(wrist_cloud),
            parameters=[{
                'camera_namespaces': ['wrist1'],
                'min_depth': 0.05,
                'max_depth': 1.5,
                'stride': 2,
            }],
            **_THREAD_CAP,
        ),
    ])
