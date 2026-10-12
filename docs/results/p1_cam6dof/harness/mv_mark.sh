#!/usr/bin/env bash
# CAM-6DOF: one APPROVED pointer move <arm> -> mark <name> at (x, y, 1.22) tool-down, then TCP from TF (3 s). Pattern p1_rgbd2_zone/mv_mark.sh.
#   mv_mark.sh arm_3 F1 0.25 -0.50
cd /home/user1/Documents/ceiling_arm; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
R=docs/results/p1_cam6dof; declare -A TF=([arm_1]=t1_a1 [arm_2]=t1_a2 [arm_3]=t2_a1 [arm_4]=t2_a2)
timeout 400 python3 scripts/reach_dwell_probe.py --arms $1 --target $3,$4,1.22 --approach 0 --trials 1 --tau-max 14 --plan-attempts 8 --move > $R/mv_$2.log 2>&1
echo "rc $?  $(grep -oE "percobaan rencana: \[[^]]*\]" $R/mv_$2.log | tail -1)"
grep -E "^\s+\[ 1\]" $R/mv_$2.log | cut -c1-160
python3 docs/results/p1_arm1_zone/tcp_read.py ${TF[$1]}_tool_frame 3 | sed "s/^/$2 cmd ($3, $4, 1.22) /" | tee -a $R/tcp_marks.txt
