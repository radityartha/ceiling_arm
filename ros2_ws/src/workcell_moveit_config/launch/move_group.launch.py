import sys

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("trailer_workcell", package_name="workcell_moveit_config").to_moveit_configs()
    # G57: this file (included by generate_demo_launch) builds the move_group node. octomap_updaters:=false on the
    # my_workcell.launch.py command line -> no octomap updaters (2 updaters x env_static_map leak, OOM ~100 min on HW).
    if "octomap_updaters:=false" in sys.argv:
        moveit_config.sensors_3d = {k: v for k, v in moveit_config.sensors_3d.items()
                                    if k in ("octomap_frame", "octomap_resolution", "max_range")}
    return generate_move_group_launch(moveit_config)
