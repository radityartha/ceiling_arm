#!/bin/bash
set -e # Exit on error

# Perception on this cell is TWO RGBD modules, not LIDAR. The Livox Mid360 stage
# that used to run first was removed 2026-08-16: livox_ros_driver2 had no source
# in this checkout (empty dir, and no .gitmodules to restore it from), so
# `cd src/livox_ros_driver2 && ./build.sh humble` failed on line 7 and `set -e`
# killed the whole build before a single workspace package was compiled.

source /opt/ros/humble/setup.bash

echo "--- Building Workspace ---"
# Clean only main build/log, leave install
rm -rf build log
colcon build --symlink-install

echo "--- Build Complete. Source install/setup.bash in your terminal ---"
