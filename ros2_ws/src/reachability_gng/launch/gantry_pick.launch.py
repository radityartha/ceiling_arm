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

    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false',
                              description='true = plan AND execute; '
                                          'false = plan-only (benchmark)'),
        DeclareLaunchArgument('csv', default_value='',
                              description='per-pick CSV log path (empty = off)'),
        OpaqueFunction(function=_kill_stale),
        Node(
            package='reachability_gng',
            executable='gantry_reach_executor',
            name='gantry_reach_executor',
            output='screen',
            parameters=[{
                # plain string lists -> STRING_ARRAY (do NOT use LaunchConfig in
                # a list: launch would concatenate them into one string).
                'arm_names': ['arm_1', 'arm_2'],
                'arm_models': ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz'],
                'arm_groups': ['gantry_1_with_arm_1', 'gantry_1_with_arm_2'],
                'arm_ee_frames': ['t1_a1_tool_frame', 't1_a2_tool_frame'],
                'gripper_links': ['t1_a1_gripper_base_link',
                                  't1_a2_gripper_base_link'],
                # value_type coerces the "true"/"false" string to a real bool.
                'execute': ParameterValue(execute, value_type=bool),
                'csv_log': csv,
            }],
        ),
    ])
