#!/usr/bin/env bash
# G63 T6 plan-check: mock stack, domain 77 (fake HW, NOT data). octomap_updaters:=false (leak, G57 B4). SIG_DFL so SIGINT reaches launch.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 DISPLAY=:1 PYTHONUNBUFFERED=1
cd /home/user1/Documents/ceiling_arm
exec python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','launch','workcell_moveit_config','my_workcell.launch.py','use_fake_hardware:=true','enable_gantry_bridge:=false','use_sim_time:=false','octomap_updaters:=false'])"
