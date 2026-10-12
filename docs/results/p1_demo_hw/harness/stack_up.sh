#!/usr/bin/env bash
# Real HW bring-up (runbook §1 + G57 B4: octomap_updaters:=false). SIG_DFL so SIGINT reaches the launch.
source $(dirname $0)/env.sh
cd /home/user1/Documents/ceiling_arm
exec python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','launch','workcell_moveit_config','my_workcell.launch.py','use_fake_hardware:=false','enable_gantry_bridge:=true','use_sim_time:=false','octomap_updaters:=false'])"
