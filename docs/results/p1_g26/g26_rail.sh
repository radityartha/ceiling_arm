#!/usr/bin/env bash
# G26: rail_to_g APA ADANYA. bash g26_rail.sh S G GOAL [--move]
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
cd $ROOT/scripts
python3 -u $ROOT/docs/results/p1_g22/rail_to_g.py --gantry $2 --seed $1 $3 --plan $ROOT/docs/results/p1_g26/g26_candidates.json ${4:-} 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
exit ${PIPESTATUS[0]}
