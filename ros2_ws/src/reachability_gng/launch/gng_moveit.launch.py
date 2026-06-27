"""MoveIt MotionPlanning + both GNG reachability clouds in ONE RViz.

Brings up, on FAKE/MOCK hardware (no real arms, no Isaac Sim, NO LIDAR):
  - robot_state_publisher + move_group + a ros2_control mock joint_state source
    + the workcell controllers, built INLINE from a moveit_config whose
    robot_description is the FLATTENED workcell_full.urdf (mock_components baked
    in). We deliberately do NOT use trailer_workcell.urdf.xacro nor the package's
    rsp/move_group launch files: the ros2_kortex submodule is incomplete here so
    that xacro fails ("Invalid parameter gripper"). The flattened urdf is the
    source of truth (same model the GNG clouds use), per the package README.
  - one GNG `visualize` node PER ARM, on distinct node names so each owns its
    own <node>/gng_markers topic and edge color (arm_1 green, arm_2 orange).
  - RViz loaded with config/gng_moveit.rviz: MotionPlanning (groups
    gantry_1_with_arm_1 / gantry_1_with_arm_2) + RobotModel + both GNG clouds.

So you can Plan/Execute in MoveIt and see the reachability maps at the same
time. Do NOT also start joint_state_publisher_gui (one /joint_states source).

Build the maps first (dense recipe):
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  ros2_ws/src/reachability_gng/build_maps.sh      # -> /tmp/arm1_model.npz, /tmp/arm2_model.npz

Then:
  ros2 launch reachability_gng gng_moveit.launch.py
  ros2 launch reachability_gng gng_moveit.launch.py \
       arm1_model:=/tmp/arm1_model.npz arm2_model:=/tmp/arm2_model.npz color_by:=hits
"""

import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_static_virtual_joint_tfs_launch,
)

# kill only OUR own node executables before (re)starting, so a stale/orphaned
# instance can't double-publish a topic. Scoped to executable paths so it never
# matches this launch process itself.
_STALE_PATTERNS = [
    'lib/reachability_gng/visualize',   # GNG marker publishers
    'rviz2 -d .*gng_moveit.rviz',       # our RViz instance only
]

# Same 7 controllers my_workcell.launch.py spawns (the combined arm+table
# controllers, not the per-arm ones).
_CONTROLLERS = [
    'joint_state_broadcaster',
    'gripper_1_controller', 'gripper_2_controller',
    'gripper_3_controller', 'gripper_4_controller',
    'gantry_1_with_arm_controller', 'gantry_2_with_arm_controller',
]


def _kill_stale(context, *args, **kwargs):
    for pat in _STALE_PATTERNS:
        subprocess.run(['pkill', '-9', '-f', pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)  # let DDS discovery drop the killed publishers
    return []


def generate_launch_description():
    gng_pkg = get_package_share_directory('reachability_gng')
    desc_pkg = get_package_share_directory('workcell_description')
    rviz_cfg = os.path.join(gng_pkg, 'config', 'gng_moveit.rviz')
    workcell_urdf = os.path.join(desc_pkg, 'urdf', 'workcell_full.urdf')

    arm1_model = LaunchConfiguration('arm1_model')
    arm2_model = LaunchConfiguration('arm2_model')
    color_by = LaunchConfiguration('color_by')
    frame = LaunchConfiguration('frame')

    # robot_description from the flattened urdf (mock hardware baked in);
    # everything else (SRDF, kinematics, joint limits, controllers, planning
    # pipelines) from workcell_moveit_config by convention.
    moveit_config = (
        MoveItConfigsBuilder('trailer_workcell', package_name='workcell_moveit_config')
        .robot_description(file_path=workcell_urdf)
        # octomap obstacles from the two RGBD object clouds come in automatically:
        # to_moveit_configs() auto-loads config/sensors_3d.yaml, which now carries
        # the rgbd_objects / rgbd2_objects PointCloudOctomapUpdaters.
        .to_moveit_configs()
    )
    ros2_controllers = str(moveit_config.package_path / 'config' / 'ros2_controllers.yaml')

    ld = LaunchDescription([
        DeclareLaunchArgument('arm1_model', default_value='/tmp/arm1_model.npz'),
        DeclareLaunchArgument('arm2_model', default_value='/tmp/arm2_model.npz'),
        DeclareLaunchArgument('color_by', default_value='manip',
                              description='manip | hits'),
        DeclareLaunchArgument('frame', default_value='world'),
        OpaqueFunction(function=_kill_stale),
    ])

    # static TF for any SRDF virtual joints (e.g. world -> base).
    for e in generate_static_virtual_joint_tfs_launch(moveit_config).entities:
        ld.add_action(e)

    # robot_state_publisher (publishes /robot_description + link tfs).
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[moveit_config.robot_description]))

    # move_group (built inline from moveit_config; no xacro re-read).
    for e in generate_move_group_launch(moveit_config).entities:
        ld.add_action(e)

    # ros2_control mock system + the workcell's 7 controllers.
    ld.add_action(Node(
        package='controller_manager', executable='ros2_control_node',
        output='screen',
        parameters=[moveit_config.robot_description, ros2_controllers]))
    for ctrl in _CONTROLLERS:
        ld.add_action(Node(
            package='controller_manager', executable='spawner',
            arguments=[ctrl], output='screen'))

    # one GNG cloud per arm, distinct node name -> distinct topic + color.
    ld.add_action(Node(
        package='reachability_gng', executable='visualize', name='gng_arm1',
        output='screen',
        parameters=[{'model_path': arm1_model, 'color_by': color_by,
                     'frame': frame, 'edge_color': [0.0, 1.0, 0.0, 0.6]}]))  # green
    ld.add_action(Node(
        package='reachability_gng', executable='visualize', name='gng_arm2',
        output='screen',
        parameters=[{'model_path': arm2_model, 'color_by': color_by,
                     'frame': frame, 'edge_color': [1.0, 0.55, 0.0, 0.6]}]))  # orange

    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]))

    return ld
