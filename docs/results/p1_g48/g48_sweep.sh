#!/usr/bin/env bash
# G48 rotation sweep via pose_to_g (S12 arms REST, S13" target in g48_sweep_plan.json, S23" --rot-max, G33 env sweep, S28
# cross-gantry sweep). ONE gantry; the other stays parked. bash g48_sweep.sh <gantry> <from_leg> <to_leg> [--dry]
# On HW: ONE leg per call (from_leg = to_leg), operator asked before EVERY leg (campaign phase 6).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
H=$(cd $(dirname $0) && pwd); G=$1; A=$2; B=$3; MV=--move; [ "${4:-}" = "--dry" ] && MV=
LEGS=$(python3 -c "import json;l=json.load(open('$H/g48_sweep_legs.json'))['$G'];print(' '.join(f'{x}:{y}' for x,y in l[$A:$B+1]))")
i=$A
for L in $LEGS; do
  echo "== kaki $i g$G -> ${L%%:*} m, ${L##*:} deg  $(date +%T)"
  python3 -u $H/../p1_g32/pose_to_g.py --gantry $G --seed 48 --plan $H/g48_sweep_plan.json --variant SWEEP \
    ${L%%:*} ${L##*:} $MV --rot-max 125 2>&1 | grep -v "UserWarning\|buildGeomFromUrdf" | tail -3
  rc=${PIPESTATUS[0]}; [ $rc -eq 0 ] || { echo "== BERHENTI rc $rc"; exit $rc; }
  i=$((i+1))
done
