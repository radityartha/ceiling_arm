#!/usr/bin/env bash
# G36b: run_g32 APA ADANYA (G36: retract_check di probe + S18 branch-and-bound), seed 36, varian V, stack NYATA.
# bash g36b_run.sh V --dry | --move     (salinan p1_g35/g35b_run.sh; hanya jalur log/arsip berbeda)
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -u
V=$1; M=${2:-}
case "$M" in --dry) DRY=--dry;; --move) DRY=;; *) echo "argumen ke-2 harus --dry atau --move"; exit 1;; esac
OUT=/tmp/g36hw_$V${DRY:+_dry}; rm -rf "$OUT"; mkdir -p $ROOT/docs/results/p1_g36/hw
python3 -u $ROOT/docs/results/p1_g32/run_g32.py --seed 36 --variant $V $DRY \
  --launch-log /tmp/g36b_launch.log --mon /tmp/g36hw_step_samples.csv \
  --out-dir "$OUT" --archive $ROOT/docs/results/p1_g36/hw --prefix g36hw_${V}${DRY:+_dry}_ 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
echo "== RC ${PIPESTATUS[0]}"
