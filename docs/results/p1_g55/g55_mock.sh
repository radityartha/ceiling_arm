#!/usr/bin/env bash
# G55 A1.4 + G56 A2 on the mock (domain 77, monitor + recorder up, REST + PARK, K0). Fake HW, NOT data.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 G47_GOAL_TOL_DEG=0.05
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
SP=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/05bd37bd-31fc-4191-8a8f-e012117e87ee/scratchpad
export LAUNCH_LOG=/tmp/g56_mock_launch.log MON=$SP/g56_mon_samples.csv
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
P=$D/p1_g55/g55_selected.json; NEW="0 3 7 8 10 11 18"
python3 -u p1_g31/g31_screen.py --plan $P --variant R90sel --final-retract --task-cli --repeats 3 --seeds $NEW \
  --out p1_g55/g55_drycli.json --traj p1_g55/g55_drycli_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
echo "== DRY-CLI SELESAI $(date +%T)"
for S in $NEW; do
  OK=$(python3 -c "import json;r=[x for x in json.load(open('p1_g55/g55_drycli.json')) if x['seed']==$S];print(int(bool(r) and r[-1]['ok']))")
  if [ "$OK" != "1" ]; then echo "== smoke s$S DILEWATI (DRY-CLI tidak 3/3)"; continue; fi
  echo "== smoke s$S mulai $(date +%T)"
  bash p1_g54/g54_smoke.sh g55_s$S $S $P R90sel > p1_g55/g55_smoke_s$S.log 2>&1
  echo "== smoke s$S: $(grep -E 'SELESAI: |AUTO-STOP: ' p1_g55/g55_smoke_s$S.log | head -1) | $(grep -E 'K0 (LULUS|TOLAK)' p1_g55/g55_smoke_s$S.log | tail -1)"
done
echo "== G56 uji $(date +%T)"
bash p1_g56/g56_tests.sh
echo "== SEMUA SELESAI $(date +%T)"
