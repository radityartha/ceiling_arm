#!/bin/bash
# G57 HW: one parallel run (run_g54), detached.   bash p1hw_run.sh <seed> <plan rel. to results/> <variant>
cd /home/user1/Documents/ceiling_arm/docs/results
source /opt/ros/humble/setup.bash; source ../../ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
R=$PWD; S=$1; P=$R/$2; V=$3
setsid nohup python3 -u $R/p1_g54/run_g54.py --seed $S --plan $P --variant $V --final-retract --goal-tol-deg 0.05 --tau-max 14 \
  --launch-log /tmp/p1hw_launch.log --mon /tmp/p1hw_mon_samples.csv --out-dir /tmp/p1hw_par_s${S}_$V \
  --archive $R/p1_g57/hw --prefix p1hw_par_s${S}_${V}_ > /tmp/p1hw_par_s${S}_$V.log 2>&1 < /dev/null &
echo "start $(date +%T) s$S $V log /tmp/p1hw_par_s${S}_$V.log"
