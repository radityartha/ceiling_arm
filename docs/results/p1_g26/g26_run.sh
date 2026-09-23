#!/usr/bin/env bash
# G26 A4: run_g22 APA ADANYA pada seed S dari g26_candidates.json. DRY kecuali argumen ke-2 = --move.
#   bash g26_run.sh S            # DRY
#   bash g26_run.sh S --move     # JADWAL PENUH
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
S=$1; MODE=${2:-}
[ -n "$MODE" ] && [ "$MODE" != "--move" ] && { echo "argumen ke-2 hanya --move"; exit 1; }
if [ -n "$MODE" ]; then DRY=(); OUT=/tmp/g26_s$S; PFX=g26_s${S}_; else DRY=(--dry); OUT=/tmp/g26_s${S}_dry; PFX=g26_s${S}_DRY_; fi
rm -rf "$OUT"
python3 -u $ROOT/docs/results/p1_g22/run_g22.py --seed $S \
  --plan $ROOT/docs/results/p1_g26/g26_candidates.json \
  --launch-log /tmp/g26_t1.log --mon /tmp/g26_step_samples.csv \
  --out-dir "$OUT" --archive $ROOT/docs/results/p1_g26 --prefix "$PFX" "${DRY[@]}" 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
exit ${PIPESTATUS[0]}
