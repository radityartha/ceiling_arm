#!/usr/bin/env bash
# Launch the SYSTEM rviz2 (/opt/ros/humble) with the Kinova MoveIt config.
#
# Why this exists: the VNC desktop session injects $HOME/rviz2_ws and
# $HOME/moveit2_ws into PATH / LD_LIBRARY_PATH before .bashrc runs. Those
# custom builds are broken (their MoveIt plugin libs were removed), so a
# plain `rviz2` resolves to the wrong binary and renders no window.
# This script strips those overlay paths, then runs the apt-installed rviz2
# with the MotionPlanning panel.
#
# Usage:  ./scripts/run_rviz_moveit.sh
set -e

# strip the broken overlays from every relevant env var
strip() { echo "$1" | tr ':' '\n' | grep -vE 'rviz2_ws|moveit2_ws' | paste -sd: -; }
export PATH="$(strip "$PATH")"
export AMENT_PREFIX_PATH="$(strip "${AMENT_PREFIX_PATH:-}")"
export LD_LIBRARY_PATH="$(strip "${LD_LIBRARY_PATH:-}")"
export PYTHONPATH="$(strip "${PYTHONPATH:-}")"
export CMAKE_PREFIX_PATH="$(strip "${CMAKE_PREFIX_PATH:-}")"

source /opt/ros/humble/setup.bash
source "$HOME/Documents/ceiling_arm/ros2_ws/install/setup.bash"

echo "Using rviz2: $(command -v rviz2)"   # should be /opt/ros/humble/bin/rviz2

RVIZ_CONFIG="$HOME/Documents/ceiling_arm/ros2_ws/install/kinova_gen3_lite_moveit_config/share/kinova_gen3_lite_moveit_config/config/moveit.rviz"

exec ros2 launch kinova_gen3_lite_moveit_config moveit_rviz.launch.py \
    rviz_config:="$RVIZ_CONFIG"
