#!/usr/bin/env bash
# G61 mock stack, domain 77 (fake HW, NOT data). octomap_updaters:=false (leak, G57 B4). SIG_DFL via the python wrapper.
source $(dirname $0)/g61_env.sh; trap - INT
exec ros2 launch workcell_moveit_config my_workcell.launch.py use_fake_hardware:=true enable_gantry_bridge:=false use_sim_time:=false octomap_updaters:=false
