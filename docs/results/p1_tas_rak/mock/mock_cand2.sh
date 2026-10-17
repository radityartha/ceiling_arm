#!/usr/bin/env bash
# Ronde 2 (MOCK 77, mock sama yang masih hidup; g1 1.40, g2 0/0, REST): DRY pose_to_g tiap kandidat; yang tidak REFUSE -> --move
# -> plan-check lengan terdekat ke tas (z 1.34, 1.40) -> kembali (0,0).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1
R=/home/user1/Documents/ceiling_arm/docs/results; T=$R/p1_tas_rak; M=$T/mock; L=/tmp/p1tr_mock; cd /home/user1/Documents/ceiling_arm/scripts
step(){ echo "=== $(date +%T) $*"; }
for c in "0.7 85 arm_4" "0.85 65 arm_4" "0.65 -25 arm_3" "0.7 90 arm_4" "0.8 75 arm_4" "0.75 80 arm_4" "0.7 -25 arm_3" "1.0 45 arm_4"; do
  set -- $c; tag=r2_g2_${1}_${2}
  step "DRY g2 -> $1 $2"; python3 $R/p1_g32/pose_to_g.py --gantry 2 --seed 64 --plan $T/tas_rak_plan.json --variant TR -- $1 $2 > $M/${tag}_dry.log 2>&1
  grep -E "^G33|^S28|REFUSE" $M/${tag}_dry.log | cut -c1-200
  grep -q REFUSE $M/${tag}_dry.log && continue
  step "MOVE g2 -> $1 $2"; python3 $R/p1_g32/pose_to_g.py --gantry 2 --seed 64 --plan $T/tas_rak_plan.json --variant TR --move -- $1 $2 > $M/${tag}_to.log 2>&1
  grep -E "^akhir|REFUSE|GAGAL" $M/${tag}_to.log | cut -c1-160
  for z in 1.34 1.40; do step "plan-check $3 z $z"
    timeout 600 python3 -u reach_dwell_probe.py --arms $3 --target=0.978,-0.831,$z --approach 0 --trials 1 --tau-max 14 \
      --monitor-csv $L/mon_samples.csv --plan-check > $M/${tag}_pc_z${z}.log 2>&1
    grep -E "terketat|moveit|\[ 1\]" $M/${tag}_pc_z${z}.log | cut -c1-200 | tail -4
  done
  step "g2 -> 0 0"; python3 $R/p1_g32/pose_to_g.py --gantry 2 --seed 64 --plan $T/tas_rak_plan.json --variant TR --move 0 0 > $M/${tag}_back.log 2>&1
  grep -E "^akhir|REFUSE|GAGAL" $M/${tag}_back.log | cut -c1-160
done
step selesai
