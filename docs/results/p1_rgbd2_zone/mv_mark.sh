#!/usr/bin/env bash
# RGBD2-ZONE A1.1: one approved arm_1 move to mark <name> at (x, y, 1.22) tool-down, then TCP from TF (3 s).
# mv_mark.sh C5 1.65 0.65
cd /home/user1/Documents/ceiling_arm; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
R=docs/results/p1_rgbd2_zone
timeout 400 python3 scripts/reach_dwell_probe.py --arms arm_1 --target $2,$3,1.22 --approach 0 --trials 1 --tau-max 14 --plan-attempts 8 --move > $R/mv_$1.log 2>&1
echo "rc $?  $(grep -oE "percobaan rencana: \[[^]]*\]" $R/mv_$1.log | tail -1)"
grep -E "^\s+\[ 1\]" $R/mv_$1.log | cut -c1-160
python3 docs/results/p1_arm1_zone/tcp_read.py t1_a1_tool_frame 3 | sed "s/^/$1 cmd ($2, $3, 1.22) /" | tee -a $R/tcp_marks.txt
