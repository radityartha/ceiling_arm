#!/usr/bin/env bash
# G36 B: g31_screen (retract_check + 3 attempts per task, = HW probe) on the MOCK stack (domain 77; DOMAIN=0 = real stack), seed 36, PLAN-ONLY.
# Independent samples (--repeats 1 per call, so one refusal does not stop the count): R10 x N10, then R0 x N0.
# usage: g36_planonly.sh <tag> <N10> <N0>
ROOT=/home/user1/Documents/ceiling_arm
source /opt/ros/humble/setup.bash
source $ROOT/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${DOMAIN:-77}   # G36b HW: DOMAIN=0
D=$ROOT/docs/results/p1_g36
TAG=$1; N10=$2; N0=$3
run() { python3 -u $ROOT/docs/results/p1_g31/g31_screen.py --seeds 36 --variant $1 --repeats 1 \
    --plan $ROOT/docs/results/p1_g32/g32_candidates.json --out $D/g36_planonly_${TAG}_$1.json \
    --traj $D/g36_planonly_plans_${TAG}_$1.jsonl 2>&1 | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"; }
for i in $(seq 1 $N10); do echo "== R10 sampel $i/$N10 $(date +%T)"; run R10; done
for i in $(seq 1 $N0); do echo "== R0 sampel $i/$N0 $(date +%T)"; run R0; done
echo "== SELESAI $(date +%T)"
