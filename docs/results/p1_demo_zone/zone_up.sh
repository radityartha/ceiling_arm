#!/usr/bin/env bash
# DEMO-ZONE: zone_correct_node (P3 lokal zona meja arm_1) atas /loc_rgbd2/tracks. exec -> PID $! = node: SIGINT PID itu.
# Domain dari DEMO_DOMAIN (p1_demo_hw/harness/env.sh). Arg tambahan diteruskan (mis. -p object_height:=0.165).
source /home/user1/Documents/ceiling_arm/docs/results/p1_demo_hw/harness/env.sh; trap - INT
exec python3 /home/user1/Documents/ceiling_arm/docs/results/p1_demo_zone/zone_correct_node.py --ros-args "$@"
