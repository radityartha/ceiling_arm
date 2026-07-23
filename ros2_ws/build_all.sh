#!/bin/bash
set -e # Exit on error

source /opt/ros/humble/setup.bash # Source ROS

if [ -f src/livox_ros_driver2/build.sh ]; then
    echo "--- Building Livox Driver ---"
    cd src/livox_ros_driver2
    ./build.sh humble
    source install/setup.bash # Source Livox build output
    cd ../.. # Back to workspace root (ros2_ws)
else
    echo "--- Livox driver source not present, skipping (src/livox_ros_driver2 empty) ---"
fi

source install/setup.bash

echo "--- Building Rest of Workspace ---"
# Clean only main build/log, leave install
rm -rf build log
colcon build --symlink-install --packages-ignore livox_ros_driver2 livox_sdk2

echo "--- Build Complete. Source install/setup.bash in your terminal ---"

source /opt/ros/humble/setup.bash
source ~/Documents/ceiling_arm/ros2_ws/install/setup.bash