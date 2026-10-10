#!/usr/bin/env bash
# G47 A1.2 smoke (fake HW, NOT data): run_g32 --final-retract --move --rot-max 125 on the MOCK (domain 77), then home
# (return_rest g2, g1; pose_to_g g1 -> PARK 1.5/0, g2 -> 0/0). bash g47_smoke.sh <seed> <plan json> <variant>
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ROOT=/home/user1/Documents/ceiling_arm; H=$ROOT/docs/results/p1_g47; SP=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/bf76def2-a023-4612-acce-d968fb8e9f2a/scratchpad
S=$1; PLAN=$2; V=$3; OUT=/tmp/claude-1001/g47smoke_s$S; rm -rf "$OUT"; mkdir -p $H/smoke
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
python3 -u $ROOT/docs/results/p1_g32/run_g32.py --seed $S --plan $PLAN --variant $V --final-retract --rot-max 125 --goal-tol-deg 0.05 \
  --launch-log $SP/g47_mock_launch.log --mon $SP/g47_mon_samples.csv \
  --out-dir "$OUT" --archive $H/smoke --prefix g47smoke_s${S}_ 2>&1 | grep --line-buffered -v "$F"
echo "== RC run_g32 ${PIPESTATUS[0]}"
bash $H/g47_home.sh $S $PLAN $V
echo "== SELESAI $(date +%T)"
