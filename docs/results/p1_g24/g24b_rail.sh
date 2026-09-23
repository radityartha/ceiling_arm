#!/usr/bin/env bash
# G24b: rail_to_g untuk instance seed 1 (g24_candidates.json). DRY kecuali --move.
#   bash g24b_rail.sh <gantry> <goal_m> [--move]
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
G=$1; GOAL=$2; MODE=${3:-}
[ -n "$MODE" ] && [ "$MODE" != "--move" ] && { echo "argumen ke-3 hanya --move"; exit 1; }
LOG=$ROOT/docs/results/p1_g24/g24b_rail_g${G}_${GOAL}_$([ -n "$MODE" ] && echo move || echo dry)_$(date +%H%M%S).log
cd $ROOT/scripts
echo "=== $(date +%T) rail_to_g --gantry $G --seed 1 $GOAL $MODE" | tee "$LOG"
python3 -u $ROOT/docs/results/p1_g22/rail_to_g.py --gantry "$G" --seed 1 \
  --plan $ROOT/docs/results/p1_g24/g24_candidates.json "$GOAL" $MODE 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf" | tee -a "$LOG"
rc=${PIPESTATUS[0]}
echo "rc=$rc -> $LOG" | tee -a "$LOG"
exit $rc
