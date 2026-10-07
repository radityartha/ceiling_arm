#!/usr/bin/env bash
# G32-HW A1 tahap 1: g31_screen APA ADANYA di stack NYATA, seed 36, R10 lalu R0, k-of-k 3. PLAN-ONLY.
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
D=$ROOT/docs/results/p1_g32hw
for V in R10 R0; do
  python3 -u $ROOT/docs/results/p1_g31/g31_screen.py --seeds 36 --variant $V --repeats 3 \
    --plan $ROOT/docs/results/p1_g32/g32_candidates.json --out $D/g32hw_screen_$V.json \
    --traj $D/g32hw_screen_plans_$V.jsonl 2>&1 | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
done
echo "== SELESAI"
