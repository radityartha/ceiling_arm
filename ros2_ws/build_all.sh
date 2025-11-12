#!/bin/bash
set -e # Exit on error

echo "--- Building Livox Driver ---"
source /opt/ros/humble/setup.bash # Source ROS
cd src/livox_ros_driver2
./build.sh humble
source install/setup.bash # Source Livox build output
cd ../.. # Back to workspace root (ros2_ws)

source install/setup.bash 

echo "--- Building Rest of Workspace ---"
# Clean only main build/log, leave install
rm -rf build log
colcon build --symlink-install --packages-ignore livox_ros_driver2 livox_sdk2

echo "--- Build Complete. Source install/setup.bash in your terminal ---"