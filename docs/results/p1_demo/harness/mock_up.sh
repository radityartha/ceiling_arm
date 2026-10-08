#!/usr/bin/env bash
# DEMO-OFF mock stack, domain 77. SIG_DFL so SIGINT reaches the launch (ros2-launch-pid-leak memory).
source $(dirname $0)/env.sh
trap - INT
exec ros2 launch workcell_moveit_config my_workcell.launch.py use_fake_hardware:=true enable_gantry_bridge:=false use_sim_time:=false
