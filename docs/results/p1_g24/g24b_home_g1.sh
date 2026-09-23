#!/usr/bin/env bash
# G24b pemulihan: rel g1 -> 0.00 lewat rail_to_g APA ADANYA (S12/S23/S24/S17),
# tujuan S13' dari g24b_recovery_plan.json (seed 9001). DRY kecuali --move.
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
MODE=${1:-}
[ -n "$MODE" ] && [ "$MODE" != "--move" ] && { echo "argumen hanya --move"; exit 1; }
LOG=$ROOT/docs/results/p1_g24/g24b_home_g1_$([ -n "$MODE" ] && echo move || echo dry)_$(date +%H%M%S).log
cd $ROOT/scripts
python3 -u $ROOT/docs/results/p1_g22/rail_to_g.py --gantry 1 --seed 9001 \
  --plan $ROOT/docs/results/p1_g24/g24b_recovery_plan.json 0.00 $MODE 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf" | tee "$LOG"
exit ${PIPESTATUS[0]}
