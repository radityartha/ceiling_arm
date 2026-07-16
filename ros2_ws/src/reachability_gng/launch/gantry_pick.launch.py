"""Energy-aware gantry pick: arm selection + base placement via the GNG maps.

Starts the gantry_reach_executor, which on each `~/pick` chooses among arm_1..arm_4
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
        -p arm_models:='[/path/a1.npz,/path/a2.npz,/path/a3.npz,/path/a4.npz]'
"""
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    selection_mode = LaunchConfiguration('selection_mode')
    fixed_arm = LaunchConfiguration('fixed_arm')

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
        DeclareLaunchArgument('selection_mode', default_value='energy',
                              description='E3 arm-selection policy: energy '
                                          '(paper method) | nearest | fixed | '
                                          'random'),
        DeclareLaunchArgument('fixed_arm', default_value='arm_1',
                              description='arm used when selection_mode=fixed '
                                          '(arm_1..arm_4)'),
        OpaqueFunction(function=_kill_stale),
        Node(
            package='reachability_gng',
            executable='gantry_reach_executor',
            name='gantry_reach_executor',
            output='screen',
            parameters=[{
                # plain string lists -> STRING_ARRAY (do NOT use LaunchConfig in
                # a list: launch would concatenate them into one string).
                'arm_names': ['arm_1', 'arm_2', 'arm_3', 'arm_4'],
                'arm_models': ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz',
                               '/tmp/arm3_model.npz', '/tmp/arm4_model.npz'],
                'arm_groups': ['gantry_1_with_arm_1', 'gantry_1_with_arm_2',
                               'gantry_2_with_arm_3', 'gantry_2_with_arm_4'],
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
                'selection_mode': selection_mode,
                'fixed_arm': fixed_arm,
            }],
        ),
    ])
