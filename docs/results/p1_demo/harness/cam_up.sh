#!/usr/bin/env bash
source $(dirname $0)/env79.sh; trap - INT
cd /home/user1/Documents/ceiling_arm/ros2_ws && exec ros2 launch reachability_gng rgbd_perception.launch.py seg_prompts:=bottle with_depth_cloud:=false
