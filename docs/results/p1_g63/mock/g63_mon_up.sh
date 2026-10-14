#!/usr/bin/env bash
# reach_dwell_monitor for the mock plan-check (probe needs a subscriber on /reach_dwell/target/<arm>).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1
exec python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=/tmp/g63_mock_mon'])"
