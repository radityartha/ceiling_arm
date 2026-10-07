#!/usr/bin/env bash
# G33 step 4: g31_screen (now with the G33 env screen) on the MOCK stack, domain 77,
# seed 36, R0 then R10, k-of-k 3. PLAN-ONLY. env_static_map must be in the scene.
ROOT=/home/user1/Documents/ceiling_arm
source /opt/ros/humble/setup.bash
source $ROOT/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
D=$ROOT/docs/results/p1_g33
for V in R0 R10; do
  python3 -u $ROOT/docs/results/p1_g31/g31_screen.py --seeds 36 --variant $V --repeats 3 \
    --plan $ROOT/docs/results/p1_g32/g32_candidates.json --out $D/g33_planonly_$V.json \
    --traj $D/g33_planonly_plans_$V.jsonl 2>&1 | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
done
echo "== SELESAI"
