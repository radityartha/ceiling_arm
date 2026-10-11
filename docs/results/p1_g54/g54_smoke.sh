#!/usr/bin/env bash
# G54 A2.2-A2.4 (fake HW, NOT data): run_g54 [--dry] then home by retrace (g47_home.sh) + K0. MOCK domain 77 only.
# bash g54_smoke.sh <tag> <seed> <plan json> <variant> [extra run_g54 args...]
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ROOT=/home/user1/Documents/ceiling_arm; H=$ROOT/docs/results/p1_g54; SP=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/05bd37bd-31fc-4191-8a8f-e012117e87ee/scratchpad
TAG=$1; S=$2; PLAN=$3; V=$4; shift 4; mkdir -p $H/smoke
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
python3 -u $H/run_g54.py --seed $S --plan $PLAN --variant $V --final-retract --rot-max 125 --goal-tol-deg 0.05 \
  --launch-log /tmp/g52_mock_launch.log --mon $SP/g52_mon_samples.csv \
  --out-dir /tmp/claude-1001/g54_$TAG --archive $H/smoke --prefix ${TAG}_ "$@" 2>&1 | grep --line-buffered -v "$F"
echo "== RC run_g54 ${PIPESTATUS[0]}"
case " $* " in *" --dry "*) echo "== DRY: tidak pulang"; exit 0;; esac
bash $ROOT/docs/results/p1_g47/g47_home.sh $S $PLAN $V
echo "== SELESAI $(date +%T)"
