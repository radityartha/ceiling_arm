"""Simultaneous 4-arm co-manipulation: all 4 arms grip a 25 cm box's top-face
corners and lift it together (see coalition_reach_executor.py's module
docstring for the 6-stage barrier-synchronized sequence).

Requires launch_workcell.sh in `full` mode (default/`gng` mode hides gantry_2
+ arm_3/arm_4 in Isaac) and move_group already up (my_workcell.launch.py).

    ros2 launch reachability_gng gantry_pick_coalition.launch.py execute:=true
    ros2 topic pub --once /coalition_reach_executor/pick std_msgs/String \
        "{data: '0.5,0.0,1.5,0.0'}"   # x,y,z,yaw (rad) of the box CENTRE

(0.5, 0.0, 1.5, 0.0) is VERIFIED feasible against data/maps -- nearest-FK-
sample residual 0.016 m combined over all 4 corners (see the reachability
check run alongside this file's introduction).
"""
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_STALE = 'lib/reachability_gng/coalition_reach_executor'


def _kill_stale(context, *args, **kwargs):
    subprocess.run(['pkill', '-9', '-f', _STALE],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    return []


def generate_launch_description():
    execute = LaunchConfiguration('execute')
    box_size = LaunchConfiguration('box_size')
    lift_height = LaunchConfiguration('lift_height')
    grasp_descend = LaunchConfiguration('grasp_descend')
    approach_offset = LaunchConfiguration('approach_offset')

    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false',
                              description='true = plan AND execute; '
                                          'false = plan-only (benchmark)'),
        DeclareLaunchArgument('box_size', default_value='0.25',
                              description='m, top-face edge length'),
        DeclareLaunchArgument('lift_height', default_value='0.20',
                              description='m to raise all 4 arms after grasp'),
        DeclareLaunchArgument('grasp_descend', default_value='0.05',
                              description='m to lower from the pre-grasp pose '
                                          'to enclose each corner'),
        DeclareLaunchArgument('approach_offset', default_value='0.10',
                              description='m above each corner, pre-grasp stand-off'),
        OpaqueFunction(function=_kill_stale),
        Node(
            package='reachability_gng',
            executable='coalition_reach_executor',
            name='coalition_reach_executor',
            output='screen',
            parameters=[{
                # All 4 arms / both gantries -- this node ONLY does the 4-arm
                # coalition case (raises SystemExit if fewer than 4 load).
                'arm_names': ['arm_1', 'arm_2', 'arm_3', 'arm_4'],
                'arm_models': ['/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm1_model.npz',
                              '/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm2_model.npz',
                              '/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm3_model.npz',
                              '/srv/data/users/raditya/arm_WS/ceiling_arm/'
                              'data/maps/arm4_model.npz'],
                'arm_groups': ['arm_1', 'arm_2', 'arm_3', 'arm_4'],
                'arm_ee_frames': ['t1_a1_tool_frame', 't1_a2_tool_frame',
                                 't2_a1_tool_frame', 't2_a2_tool_frame'],
                'gripper_links': ['t1_a1_gripper_base_link',
                                  't1_a2_gripper_base_link',
                                  't2_a1_gripper_base_link',
                                  't2_a2_gripper_base_link'],
                'gripper_names': ['gripper_1', 'gripper_2',
                                 'gripper_3', 'gripper_4'],
                'gripper_joints': ['t1_a1_right_finger_bottom_joint',
                                  't1_a2_right_finger_bottom_joint',
                                  't2_a1_right_finger_bottom_joint',
                                  't2_a2_right_finger_bottom_joint'],
                'execute': ParameterValue(execute, value_type=bool),
                'box_size': ParameterValue(box_size, value_type=float),
                'lift_height': ParameterValue(lift_height, value_type=float),
                'grasp_descend': ParameterValue(grasp_descend, value_type=float),
                'approach_offset': ParameterValue(approach_offset, value_type=float),
            }],
        ),
    ])
