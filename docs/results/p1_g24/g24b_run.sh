#!/usr/bin/env bash
# G24b tahap 3: run_g22 pada instance seed 1. DRY (--dry) kecuali argumen = --move.
#   bash g24b_run.sh            # DRY: event list + prechecks, nothing sent
#   bash g24b_run.sh --move     # JADWAL PENUH (izin operator)
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
MODE=${1:-}
[ -n "$MODE" ] && [ "$MODE" != "--move" ] && { echo "argumen hanya --move"; exit 1; }
if [ -n "$MODE" ]; then DRY=(); OUT=/tmp/g24b; PFX=g24b_; else DRY=(--dry); OUT=/tmp/g24b_dry; PFX=g24b_DRY_; fi
rm -rf "$OUT"
python3 -u $ROOT/docs/results/p1_g22/run_g22.py --seed 1 \
  --plan $ROOT/docs/results/p1_g24/g24_candidates.json \
  --launch-log /tmp/g24b_t1.log --mon /tmp/g24b_step_samples.csv \
  --out-dir "$OUT" --archive $ROOT/docs/results/p1_g24 --prefix "$PFX" "${DRY[@]}" 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
exit ${PIPESTATUS[0]}
