#!/usr/bin/env bash
# One RViz for the whole workcell — switch the planning group (arm_1..arm_4) in
# the MotionPlanning panel instead of relaunching with a different arm IP.
#
# Brings up all four real Kinova arms + both tables under one move_group and one
# RViz via single_rviz_workcell.launch.py. Strips the broken rviz2_ws/moveit2_ws
# overlays first so the SYSTEM rviz2/move_group are used (the VNC/XFCE session
# injects broken overlays ahead of /opt/ros/humble). See project-single-arm-bringup.
#
# Usage:  ./scripts/start_single_rviz.sh [extra ros2 launch args...]
#   # all arms real, confirmed Table-2 IPs (arm_3=.11, arm_4=.10):
#   ./scripts/start_single_rviz.sh
#   # table 1 not powered -> mock just those two arms:
#   ./scripts/start_single_rviz.sh arm1_ip:=0.0.0.0 arm2_ip:=0.0.0.0   # or use_fake_hardware:=true
set -e

WS="$HOME/Documents/ceiling_arm/ros2_ws"

strip() { echo "$1" | tr ':' '\n' | grep -vE 'rviz2_ws|moveit2_ws' | paste -sd: -; }
export PATH="$(strip "$PATH")"
export AMENT_PREFIX_PATH="$(strip "${AMENT_PREFIX_PATH:-}")"
export LD_LIBRARY_PATH="$(strip "${LD_LIBRARY_PATH:-}")"
export PYTHONPATH="$(strip "${PYTHONPATH:-}")"
export CMAKE_PREFIX_PATH="$(strip "${CMAKE_PREFIX_PATH:-}")"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "Using rviz2: $(command -v rviz2)"   # expect /opt/ros/humble/bin/rviz2
echo "Launching workcell (all 4 arms, switch group in RViz)…"

exec ros2 launch workcell_moveit_config single_rviz_workcell.launch.py "$@"
