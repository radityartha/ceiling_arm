#!/usr/bin/env bash
# CAM-6DOF offline: mock (domain 77) plan-check of arm_3 floor-mark candidates at z 1.22 tool-down (A1.1 pointer pattern).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77
cd /home/user1/Documents/ceiling_arm; D=docs/results/p1_cam6dof/mock
for p in "$@"; do n=pc_${p/,/_}
  timeout 300 python3 scripts/reach_dwell_probe.py --arms arm_3 --target $p,1.22 --approach 0 --trials 1 --tau-max 14 \
    --plan-attempts 8 --plan-check > $D/$n.log 2>&1
  echo "$p rc $? $(grep -oE "percobaan rencana: \[[^]]*\]" $D/$n.log | tail -1) | $(grep -E "^\s+\[ 1\]" $D/$n.log | cut -c1-120)"
done
