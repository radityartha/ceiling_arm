#!/bin/bash
# G57 HW, NO motion: per schedule plan-only real k=3 (g31_screen --task-cli) then run_g54 --dry.
#   bash p1hw_prep.sh S PLAN(rel. results/) V [S PLAN V ...]
cd /home/user1/Documents/ceiling_arm/docs/results
source /opt/ros/humble/setup.bash; source ../../ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
R=$PWD
while [ $# -ge 3 ]; do
  S=$1; P=$R/$2; V=$3; shift 3
  echo "=== $(date +%T) plan-only s$S $V"
  G47_GOAL_TOL_DEG=0.05 python3 -u p1_g31/g31_screen.py --plan $P --variant $V --final-retract --task-cli \
    --repeats 3 --seeds $S --out p1_g57/hw/planonly_s${S}_${V}.json --traj p1_g57/hw/planonly_s${S}_${V}.jsonl | grep -E "LOLOS|GAGAL|Traceback|Error"
  echo "=== $(date +%T) DRY s$S $V"
  python3 -u p1_g54/run_g54.py --seed $S --plan $P --variant $V --final-retract --goal-tol-deg 0.05 --tau-max 14 --dry \
    --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_dry_s${S}_$V \
    --archive p1_g57/hw --prefix p1hw_par_dry_s${S}_${V}_ > /tmp/p1hw_par_dry_s${S}_$V.log 2>&1
  echo "DRY rc $?"; grep -E "K0 LULUS|K0 TOLAK|traverse di luar" /tmp/p1hw_par_dry_s${S}_$V.log
  grep "traverse'" /tmp/p1hw_par_dry_s${S}_$V.log | sed -E "s/.*'gantry': ([12]).*'to': \(([^)]*)\).*/  g\1 -> \2/"
done
echo "=== $(date +%T) SELESAI"
