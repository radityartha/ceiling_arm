#!/usr/bin/env bash
# G36 smoke (NOT data): run_g32 seed 36 on the MOCK stack (domain 77) -- the HW tool path (move_to + retract_check,
# S18 branch-and-bound) end to end. bash g36_smoke.sh <variant>
ROOT=/home/user1/Documents/ceiling_arm
source /opt/ros/humble/setup.bash
source $ROOT/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
V=$1; OUT=/tmp/g36smoke_$V; rm -rf "$OUT"
python3 -u $ROOT/docs/results/p1_g32/run_g32.py --seed 36 --variant $V \
  --launch-log /tmp/g36_mock_launch.log --mon /tmp/g36mock_step_samples.csv \
  --out-dir "$OUT" --archive $ROOT/docs/results/p1_g36/smoke --prefix g36smoke_${V}_ 2>&1 \
  | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
echo "== RC ${PIPESTATUS[0]}"
