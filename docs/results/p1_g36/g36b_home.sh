#!/usr/bin/env bash
# G36b pulang sesudah satu varian: return_rest g2 lalu g1 (menyaring + merencana sendiri sebelum kirim), lalu
# pose_to_g g1 -> (G1, 0), g2 -> (0, 0). G1 = 0 di antara varian (run berikut mulai di p0 0/0), = 1.5 (PARKIR, operator
# 2026-10-08: beban plafon tidak berat sebelah) di akhir sesi. Berhenti pada rc != 0 pertama.
# bash g36b_home.sh <tag> <variant> <G1: 0 | 1.5>
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
set -o pipefail
T=$1; V=$2; G1=$3; D=$ROOT/docs/results/p1_g36/hw; F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
case "$G1" in 0|1.5) ;; *) echo "G1 harus 0 atau 1.5"; exit 1;; esac
mkdir -p $D
run() { local log=$D/g36hw_${T}_$1.log; shift; echo "== $(date +%T) $*"; "$@" 2>&1 | grep --line-buffered -v "$F" > $log
        local rc=$?; tail -2 $log | cut -c1-300; [ $rc -eq 0 ] || { echo "== BERHENTI rc $rc"; exit $rc; }; }
cd $ROOT/scripts
run rest_g2 python3 -u return_rest.py --arms arm_3 arm_4 --move
run rest_g1 python3 -u return_rest.py --arms arm_1 arm_2 --move
run pose_g1 python3 -u $ROOT/docs/results/p1_g32/pose_to_g.py --gantry 1 --seed 36 --variant $V $G1 0 --move
run pose_g2 python3 -u $ROOT/docs/results/p1_g32/pose_to_g.py --gantry 2 --seed 36 --variant $V 0 0 --move
echo "== PULANG OK (g1 $G1)"
