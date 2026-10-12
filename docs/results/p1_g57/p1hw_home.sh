#!/bin/bash
# G57 HW home after a run. Step 1: g47_home DRY (route = retrace of the schedule). Step 2 (real): return_rest both arm pairs,
# then each leg with pose_to_g --move, STOP at the first REFUSE / rc != 0 (g47_home itself carries on to the next leg), K0.
# DRY legs 2+ are screened from the CURRENT pose (DRY does not move) -> their REFUSE is not meaningful; leg 1 is.
#   bash p1hw_home.sh <seed> <plan rel. to results/> <variant> <tag>
cd /home/user1/Documents/ceiling_arm/docs/results
source /opt/ros/humble/setup.bash; source ../../ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
R=$PWD; S=$1; P=$R/$2; V=$3; T=$4; L=/tmp/p1hw_home_${T}.log
bash p1_g47/g47_home.sh $S $P $V --dry > /tmp/p1hw_home_${T}_dry.log 2>&1
cp /tmp/p1hw_home_${T}_dry.log p1_g57/hw/
grep -q "tidak di pose jadwal" /tmp/p1hw_home_${T}_dry.log && { echo "DRY TOLAK"; cat /tmp/p1hw_home_${T}_dry.log; exit 1; }
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
{ cd ../../scripts
  for A in "arm_3 arm_4" "arm_1 arm_2"; do python3 -u return_rest.py --arms $A --move 2>&1 | grep -v "$F" | tail -1; done
  cd $R
  for G in 1 2; do
    LEGS=$(grep "^g$G pulang lewat:" /tmp/p1hw_home_${T}_dry.log | sed 's/.*lewat: //; s/(sudah di home)//')
    echo "g$G pulang lewat: ${LEGS:-(sudah di home)}"
    for X in $LEGS; do
      python3 -u p1_g32/pose_to_g.py --gantry $G --seed $S --plan $P --variant $V ${X%%:*} ${X##*:} --move --rot-max 125 > /tmp/p1hw_leg.log 2>&1; rc=$?
      grep "akhir:\|REFUSE\|T_cmd" /tmp/p1hw_leg.log | tail -2
      if [ $rc -ne 0 ] || grep -q REFUSE /tmp/p1hw_leg.log; then echo "BERHENTI: g$G kaki $X rc $rc"; exit 1; fi
    done
  done
  python3 ../../scripts/k0_check.py --rail1 1.5 --rail2 0 | tail -1
} > $L 2>&1; rc=$?
cp $L p1_g57/hw/
echo "NYATA rc $rc"; cut -c1-230 $L | grep -v '^{'
