"""View the trained GNG reachability map in RViz, together with the robot.

Starts:
  - robot_state_publisher + joint_state_publisher_gui (shows the arm + table,
    sliders let you move the 8 DOF and watch the map relate to reach)
  - the GNG visualize node (publishes /gng_visualize/gng_markers)
  - RViz with RobotModel + TF + the GNG MarkerArray

Usage:
  ros2 launch reachability_gng view_gng.launch.py model_path:=/tmp/model.npz
  ros2 launch reachability_gng view_gng.launch.py model_path:=/tmp/model.npz color_by:=hits
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# resolve the source-tree URDF relative to this launch file (symlink-install)
_HERE = os.path.dirname(os.path.realpath(__file__))
_DEFAULT_URDF = os.path.normpath(os.path.join(
    _HERE, '..', '..', 'workcell_description', 'urdf', 'workcell_full.urdf'))


def generate_launch_description():
    pkg = get_package_share_directory('reachability_gng')
    rviz_cfg = os.path.join(pkg, 'config', 'gng.rviz')

    model_path = LaunchConfiguration('model_path')
    color_by = LaunchConfiguration('color_by')
    frame = LaunchConfiguration('frame')
    urdf = LaunchConfiguration('urdf')

    def _rsp(context, *args, **kwargs):
        urdf_path = context.perform_substitution(urdf)
        with open(urdf_path) as f:
            robot_desc = f.read()
        return [Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_desc}])]

    from launch.actions import OpaqueFunction

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/tmp/model.npz'),
        DeclareLaunchArgument('color_by', default_value='manip',
                              description='manip | hits'),
        DeclareLaunchArgument('frame', default_value='world'),
        DeclareLaunchArgument('urdf', default_value=_DEFAULT_URDF),

        OpaqueFunction(function=_rsp),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', output='screen'),

        Node(package='reachability_gng', executable='visualize',
             name='gng_visualize', output='screen',
             parameters=[{'model_path': model_path,
                          'color_by': color_by,
                          'frame': frame}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen'),
    ])
