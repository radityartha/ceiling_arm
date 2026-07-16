"""Publish ONLY the per-arm GNG reachability clouds (no move_group, no RViz).

For wiring the GNG maps into an EXISTING MoveIt/RViz stack — in particular the
Isaac Sim digital twin (isaac_sim/workcell/ros/bringup.launch.py + rviz.launch.py),
which runs in its own overlay workspace. Run this from THIS repo's workspace so
the clouds publish on /gng_arm{1,2,3,4}/gng_markers; the markers reach the Isaac
RViz over DDS (use the SAME ROS_DOMAIN_ID / RMW on both sides,
e.g. ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp).

The clouds are in the `world` frame (config-independent union over all configs),
so they line up with the Isaac-driven robot as long as both share the same
workcell `world` origin (they do — same URDF definition).

  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  ros2 launch reachability_gng gng_clouds.launch.py        # /tmp/arm{1,2,3,4}_model.npz
"""

import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_STALE_PATTERN = 'lib/reachability_gng/visualize'   # GNG marker publishers
_ARM_SPECS = [
    ('arm1_model', '/tmp/arm1_model.npz', 'gng_arm1', [0.0, 1.0, 0.0, 0.6]),    # green
    ('arm2_model', '/tmp/arm2_model.npz', 'gng_arm2', [1.0, 0.55, 0.0, 0.6]),   # orange
    ('arm3_model', '/tmp/arm3_model.npz', 'gng_arm3', [0.0, 0.8, 1.0, 0.6]),    # cyan
    ('arm4_model', '/tmp/arm4_model.npz', 'gng_arm4', [1.0, 0.2, 0.8, 0.6]),    # magenta
]


def _kill_stale(context, *args, **kwargs):
    subprocess.run(['pkill', '-9', '-f', _STALE_PATTERN],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)  # let DDS discovery drop the killed publishers
    return []


def generate_launch_description():
    arm_models = {
        arg_name: LaunchConfiguration(arg_name)
        for arg_name, _, _, _ in _ARM_SPECS
    }
    color_by = LaunchConfiguration('color_by')
    frame = LaunchConfiguration('frame')

    actions = [
        DeclareLaunchArgument('color_by', default_value='manip',
                              description='manip | hits'),
        DeclareLaunchArgument('frame', default_value='world'),
    ]
    for arg_name, model_path, _, _ in _ARM_SPECS:
        actions.append(DeclareLaunchArgument(arg_name, default_value=model_path))

    actions.extend([
        OpaqueFunction(function=_kill_stale),
    ])
    for arg_name, _, node_name, edge_color in _ARM_SPECS:
        actions.append(Node(
            package='reachability_gng', executable='visualize', name=node_name,
            output='screen',
            parameters=[{
                'model_path': arm_models[arg_name], 'color_by': color_by,
                'frame': frame, 'edge_color': edge_color
            }]))

    return LaunchDescription(actions)
