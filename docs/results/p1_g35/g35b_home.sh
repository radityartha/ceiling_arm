#!/usr/bin/env bash
# G35b pulang sesudah satu varian: return_rest g2 lalu g1 (menyaring + merencana sendiri sebelum kirim),
# lalu pose_to_g g1, g2 -> (0, 0). Berhenti pada rc != 0 pertama. bash g35b_home.sh <tag> <variant>
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -o pipefail
T=$1; V=$2; D=$ROOT/docs/results/p1_g35/hw; F="UserWarning\|buildGeomFromUrdf"
run() { local log=$D/g35hw_${T}_$1.log; shift; echo "== $(date +%T) $*"; "$@" 2>&1 | grep --line-buffered -v "$F" > $log
        local rc=$?; tail -2 $log | cut -c1-300; [ $rc -eq 0 ] || { echo "== BERHENTI rc $rc"; exit $rc; }; }
cd $ROOT/scripts
run rest_g2 python3 -u return_rest.py --arms arm_3 arm_4 --move
run rest_g1 python3 -u return_rest.py --arms arm_1 arm_2 --move
for g in 1 2; do
  run pose_g$g python3 -u $ROOT/docs/results/p1_g32/pose_to_g.py --gantry $g --seed 36 --variant $V 0 0 --move
done
echo "== PULANG OK"
