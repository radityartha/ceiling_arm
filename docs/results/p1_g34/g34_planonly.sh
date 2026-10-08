#!/usr/bin/env bash
# G34 step 3: g31_screen with the env-aware retract (return_rest.plan_retract) on the
# MOCK stack (domain 77; DOMAIN=0 = real stack, G34b), seed 36, k-of-k 3. PLAN-ONLY. env_static_map must be in the scene.
# usage: g34_planonly.sh <tag> <variant...>
ROOT=/home/user1/Documents/ceiling_arm
source /opt/ros/humble/setup.bash
source $ROOT/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${DOMAIN:-77}   # G34b HW: DOMAIN=0
D=$ROOT/docs/results/p1_g34
TAG=$1; shift
for V in "$@"; do
  python3 -u $ROOT/docs/results/p1_g31/g31_screen.py --seeds 36 --variant $V --repeats 3 \
    --plan $ROOT/docs/results/p1_g32/g32_candidates.json --out $D/g34_planonly_${TAG}_$V.json \
    --traj $D/g34_planonly_plans_${TAG}_$V.jsonl 2>&1 | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
done
echo "== SELESAI"
