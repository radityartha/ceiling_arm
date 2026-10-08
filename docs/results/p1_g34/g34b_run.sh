#!/usr/bin/env bash
# G34b step 3: run_g32 APA ADANYA (peta g34n + retract sadar-lingkungan), seed 36, varian V, stack NYATA.
# bash g34b_run.sh V --dry | --move     (salinan p1_g32hw/g32hw_run.sh; hanya jalur log/arsip berbeda)
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
V=$1; M=${2:-}
case "$M" in --dry) DRY=--dry;; --move) DRY=;; *) echo "argumen ke-2 harus --dry atau --move"; exit 1;; esac
OUT=/tmp/g34hw_$V${DRY:+_dry}; rm -rf "$OUT"; mkdir -p $ROOT/docs/results/p1_g34/hw
python3 -u $ROOT/docs/results/p1_g32/run_g32.py --seed 36 --variant $V $DRY \
  --launch-log /tmp/g34b_launch.log --mon /tmp/g34hw_step_samples.csv \
  --out-dir "$OUT" --archive $ROOT/docs/results/p1_g34/hw --prefix g34hw_${V}${DRY:+_dry}_ 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
echo "== RC ${PIPESTATUS[0]}"
