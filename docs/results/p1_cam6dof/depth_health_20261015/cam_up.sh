#!/usr/bin/env bash
# Depth-health: start ONE RealSense driver node (no launch wrapper) in domain 80. exec => $! is the node PID.
# cam_up.sh <ns> [extra --ros-args -p ...]
ns=${1:?ns}; shift
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=80
trap - INT
H=/home/user1/Documents/ceiling_arm/docs/results/p1_cam6dof/depth_health_20261015
exec /opt/ros/humble/lib/realsense2_camera/realsense2_camera_node \
  --ros-args -r __node:=$ns -r __ns:=/$ns --params-file $H/${ns}_params.yaml "$@"
