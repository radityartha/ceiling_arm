#!/usr/bin/env bash
# G24b: return_rest untuk keempat lengan, per gantry (g1 lalu g2).
# DRY (hanya saringan) kecuali argumen pertama = --move.
#   bash docs/results/p1_g24/g24b_rest.sh           # DRY
#   bash docs/results/p1_g24/g24b_rest.sh --move    # GERAK (izin operator)

MODE=${1:-}
[ -n "$MODE" ] && [ "$MODE" != "--move" ] && { echo "argumen hanya --move"; exit 1; }
ROOT=/home/user1/Documents/ceiling_arm
LOG=$ROOT/docs/results/p1_g24/g24b_rest_$([ -n "$MODE" ] && echo move || echo dry).log
source $ROOT/ros2_ws/install/setup.bash
set -u
cd $ROOT/scripts
: > "$LOG"
for ARMS in "arm_1 arm_2" "arm_3 arm_4"; do
  echo "=== $(date +%T) return_rest --arms $ARMS $MODE" | tee -a "$LOG"
  timeout 180 python3 -u return_rest.py --arms $ARMS $MODE 2>&1 \
    | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf" | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  echo "rc=$rc" | tee -a "$LOG"
  [ "$rc" -ne 0 ] && { echo "BERHENTI: rc=$rc pada $ARMS" | tee -a "$LOG"; exit "$rc"; }
done
echo "SELESAI -> $LOG"
