#!/usr/bin/env bash
# CALIB-CHARUCO: both D455, no YOLOE, domain 0. Default extrinsics = launch defaults + G34 correction (incumbent, as DEMO-HW);
# pass tf1_*/tf2_* after to override (candidate). Later args win.
source /home/user1/Documents/ceiling_arm/docs/results/p1_demo_hw/harness/env.sh; trap - INT
exec ros2 launch reachability_gng realsense_dual.launch.py with_color_cloud:=false \
  tf1_x:=1.7166 tf1_y:=0.8656 tf1_z:=2.0782 tf1_roll:=-2.3436 tf1_pitch:=-0.1488 tf1_yaw:=2.1688 \
  tf2_x:=-0.1547 tf2_y:=-1.0197 tf2_z:=2.0905 tf2_roll:=-2.2369 tf2_pitch:=-0.1285 tf2_yaw:=-0.9189 "$@"
