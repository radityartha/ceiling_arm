#!/usr/bin/env bash
# G32-HW A1 tahap 4/6: run_g32 APA ADANYA, seed 36, varian V. bash g32hw_run.sh V --move
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
V=$1; [ "${2:-}" = "--move" ] || { echo "argumen ke-2 harus --move"; exit 1; }
OUT=/tmp/g32hw_$V; rm -rf "$OUT"
python3 -u $ROOT/docs/results/p1_g32/run_g32.py --seed 36 --variant $V \
  --launch-log /tmp/g32hw_t1.log --mon /tmp/g32hw_step_samples.csv \
  --out-dir "$OUT" --archive $ROOT/docs/results/p1_g32hw --prefix g32hw_${V}_ 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
echo "== RC ${PIPESTATUS[0]}"
