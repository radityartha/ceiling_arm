#!/bin/bash
# HW (G62+): one parallel run (run_g54) with the G61 changes, detached. = p1_g57/p1hw_run.sh + --goal-correct, archive p1_g62/hw.
#   bash p1hw_run.sh <seed> <plan rel. to results/> <variant> [--dry]
cd /home/user1/Documents/ceiling_arm/docs/results
source /opt/ros/humble/setup.bash; source ../../ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
R=$PWD; S=$1; P=$R/$2; V=$3; mkdir -p $R/p1_g62/hw
setsid nohup python3 -u $R/p1_g54/run_g54.py --seed $S --plan $P --variant $V --final-retract --goal-tol-deg 0.05 --tau-max 14 \
  --goal-correct $4 --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_s${S}_$V \
  --archive $R/p1_g62/hw --prefix p1hw_par_s${S}_${V}_ > /tmp/p1hw_par_s${S}_$V.log 2>&1 < /dev/null &
echo "start $(date +%T) s$S $V $4 log /tmp/p1hw_par_s${S}_$V.log"
