"""Full pick backend in one launch: perception + the energy-aware executor.

Combines the two backend launches so only ONE terminal is needed for them:
    perception.launch.py   -> seg_router, object_localizer, collision_cloud,
                              object_collision, ... (/detected_objects, octomap,
                              /target_collision_boxes)
    gantry_pick.launch.py  -> gantry_reach_executor (arm selection + planning)

Still assumed ALREADY running (NOT started here):
    * my_workcell.launch.py  -> move_group (/compute_ik + move_action), RViz
    * the Isaac bridge cameras + world->camera static TFs

The interactive picker stays in its OWN terminal (it reads the keyboard):
    ros2 run reachability_gng pick_cli

    ros2 launch reachability_gng pick_stack.launch.py
    ros2 launch reachability_gng pick_stack.launch.py execute:=true box_clearance:=0.08
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('reachability_gng'), 'launch')

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items())

    return LaunchDescription([
        # executor args
        DeclareLaunchArgument('execute', default_value='false',
                              description='true = plan AND execute; '
                                          'false = plan-only'),
        DeclareLaunchArgument('csv', default_value='',
                              description='per-pick CSV log path (empty = off)'),
        DeclareLaunchArgument('box_clearance', default_value='0.05',
                              description='EE stand-off (m) above the target box '
                                          'top'),
        # Approach-only mode: set BOTH false so the target stays a hard obstacle
        # and the gripper never enters it (no nudging before a real grasp).
        DeclareLaunchArgument('carve_target', default_value='true',
                              description='false = keep target in octomap '
                                          '(approach-only, do not carve)'),
        DeclareLaunchArgument('allow_target_collision', default_value='true',
                              description='false = do not ACM-allow gripper to '
                                          'touch the target (approach-only)'),
        DeclareLaunchArgument('compute_traj_energy', default_value='false',
                              description='true = compute per-pick mechanical '
                                          'energy (Pinocchio) for the CSV '
                                          'traj_energy column'),
        # perception args
        DeclareLaunchArgument('seg_source', default_value='yoloe',
                              description="'yoloe' (open-vocab) or 'isaac' "
                                          '(ground truth)'),
        DeclareLaunchArgument('seg_prompts',
                              default_value='box,tin can,canned food,bottle,'
                                            'banana,teddy bear,paper bag,bowl,'
                                            'doll,pan',
                              description='YOLOE open-vocab classes'),
        DeclareLaunchArgument('seg_model', default_value='yoloe-11m-seg.pt',
                              description='YOLOE weights (11m detects the sim '
                                          'objects far better than 11s)'),
        DeclareLaunchArgument('seg_conf', default_value='0.25'),
        DeclareLaunchArgument('seg_imgsz', default_value='768'),

        include('perception.launch.py', {
            'seg_source': LaunchConfiguration('seg_source'),
            'seg_prompts': LaunchConfiguration('seg_prompts'),
            'seg_model': LaunchConfiguration('seg_model'),
            'seg_conf': LaunchConfiguration('seg_conf'),
            'seg_imgsz': LaunchConfiguration('seg_imgsz'),
            'carve_target': LaunchConfiguration('carve_target'),
        }),
        include('gantry_pick.launch.py', {
            'execute': LaunchConfiguration('execute'),
            'csv': LaunchConfiguration('csv'),
            'box_clearance': LaunchConfiguration('box_clearance'),
            'allow_target_collision':
                LaunchConfiguration('allow_target_collision'),
            'compute_traj_energy': LaunchConfiguration('compute_traj_energy'),
        }),
    ])
