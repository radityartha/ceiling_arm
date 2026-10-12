#!/bin/bash
# usage: p1hw_planonly.sh S PLAN V [S PLAN V ...]  (plan-only nyata k=3, nol gerak)
cd /home/user1/Documents/ceiling_arm/docs/results
source /opt/ros/humble/setup.bash; source ../../ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
while [ $# -ge 3 ]; do
  S=$1; P=$2; V=$3; shift 3
  echo "=== $(date +%T) plan-only s$S $V"
  G47_GOAL_TOL_DEG=0.05 python3 -u p1_g31/g31_screen.py --plan $P --variant $V --final-retract --task-cli \
    --repeats 3 --seeds $S --out p1_g57/hw/planonly_s${S}_${V}.json --traj p1_g57/hw/planonly_s${S}_${V}.jsonl
  echo "=== $(date +%T) rc $?"
done
