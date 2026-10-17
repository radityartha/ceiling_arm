#!/usr/bin/env bash
# TAS-RAK luring (MOCK 77, fake HW, BUKAN data): g1 1.40, lengan REST; tiap kandidat g2 (rel, rot): pose_to_g dari (0,0) -> kandidat
# -> probe --plan-check arm_4 ke tas (z 1.34 dan 1.40) -> pose_to_g kembali (0,0). Penyaring tidak diubah.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1 DISPLAY=:1
R=/home/user1/Documents/ceiling_arm/docs/results; T=$R/p1_tas_rak; M=$T/mock; L=/tmp/p1tr_mock; mkdir -p $L; cd /home/user1/Documents/ceiling_arm/scripts
step(){ echo "=== $(date +%T) $*"; }
step mock up; setsid nohup $R/p1_cam6dof/harness/mock_up.sh > $L/mock_launch.log 2>&1 < /dev/null &
for i in $(seq 1 100); do grep -q "You can start planning now" $L/mock_launch.log && break; sleep 3; done
grep -q "You can start planning now" $L/mock_launch.log || { echo "GAGAL: move_group tidak siap"; exit 1; }
sleep 10; echo "controllers active: $(timeout 30 ros2 control list_controllers | grep -c active)"
setsid nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=$L/mon'])" > $L/mon.log 2>&1 < /dev/null &
step REST; for g in "arm_1 arm_2" "arm_3 arm_4"; do python3 return_rest.py --arms $g --move 2>&1 | grep -E "selesai|TOLAK|GAGAL"; done
step "g1 -> 1.40"; python3 $R/p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan $R/p1_g63/g63_rail_plan.json --variant G63 1.40 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
for c in "1.0 90" "1.1 70" "1.15 60" "1.2 50" "1.25 40" "1.3 30"; do set -- $c; tag=g2_${1}_${2}
  step "g2 -> $1 $2"; python3 $R/p1_g32/pose_to_g.py --gantry 2 --seed 64 --plan $T/tas_rak_plan.json --variant TR $1 $2 --move > $M/${tag}_to.log 2>&1
  grep -E "^akhir|TOLAK|REFUS|GAGAL|S28" $M/${tag}_to.log | cut -c1-220
  if grep -q "^akhir" $M/${tag}_to.log; then
    for z in 1.34 1.40; do step "plan-check arm_4 z $z"
      timeout 600 python3 -u reach_dwell_probe.py --arms arm_4 --target=0.978,-0.831,$z --approach 0 --trials 1 --tau-max 14 \
        --monitor-csv $L/mon_samples.csv --plan-check > $M/${tag}_pc_z${z}.log 2>&1
      grep -E "terketat|DITOLAK|moveit|rencana:|CLEAR|TABRAKAN|\[ 1\]" $M/${tag}_pc_z${z}.log | cut -c1-200 | tail -6
    done
  fi
  step "g2 -> 0 0"; python3 $R/p1_g32/pose_to_g.py --gantry 2 --seed 64 --plan $T/tas_rak_plan.json --variant TR 0 0 --move > $M/${tag}_back.log 2>&1
  grep -E "^akhir|TOLAK|REFUS|GAGAL" $M/${tag}_back.log | cut -c1-200
done
step selesai
