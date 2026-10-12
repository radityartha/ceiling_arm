#!/usr/bin/env bash
# D455 x2 + YOLOE 'bottle'. cwd ros2_ws/ (src/reachability_gng/mobileclip_blt.ts is TRUNCATED, F4).
# Extrinsics = launch defaults composed with the G34 fixed correction (p1_g34/rail_joint4_diff.json, same as env_static_map;
# operator 2026-10-12 DEMO-HW A0). Launch defaults are NOT changed.
source $(dirname $0)/env.sh; trap - INT
cd /home/user1/Documents/ceiling_arm/ros2_ws && exec ros2 launch reachability_gng rgbd_perception.launch.py seg_prompts:=bottle with_depth_cloud:=false \
  tf1_x:=1.7166 tf1_y:=0.8656 tf1_z:=2.0782 tf1_roll:=-2.3436 tf1_pitch:=-0.1488 tf1_yaw:=2.1688 \
  tf2_x:=-0.1547 tf2_y:=-1.0197 tf2_z:=2.0905 tf2_roll:=-2.2369 tf2_pitch:=-0.1285 tf2_yaw:=-0.9189
