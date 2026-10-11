#!/usr/bin/env bash
# G50: DRY-CLI k=3 (tol 0.05) on the 11 new schedules, then smoke --move for every seed that passed 3/3.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 G47_GOAL_TOL_DEG=0.05
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
P43=$D/p1_g43/g43_candidates.json; P49=$D/p1_g49/g49_candidates.json
python3 -u p1_g31/g31_screen.py --plan $P43 --variant R180 --final-retract --task-cli --repeats 3 --seeds 2 11 14 16 17 \
  --out p1_g50/g50_drycli.json --traj p1_g50/g50_drycli_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
python3 -u p1_g31/g31_screen.py --plan $P49 --variant R180 --final-retract --task-cli --repeats 3 --seeds 4 5 6 9 12 13 \
  --out p1_g50/g50_drycli.json --traj p1_g50/g50_drycli_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
echo "== DRY-CLI SELESAI $(date +%T)"
for S in 2 11 14 16 17 4 5 6 9 12 13; do
  case $S in 2|11|14|16|17) P=$P43;; *) P=$P49;; esac
  OK=$(python3 -c "import json;r=[x for x in json.load(open('p1_g50/g50_drycli.json')) if x['seed']==$S];print(int(bool(r) and r[-1]['ok']))")
  if [ "$OK" != "1" ]; then echo "== smoke s$S DILEWATI (DRY-CLI tidak 3/3)"; continue; fi
  echo "== smoke s$S mulai $(date +%T)"
  bash p1_g50/g50_smoke.sh $S $P R180 > p1_g50/g50_smoke_s$S.log 2>&1
  echo "== smoke s$S: $(grep -E 'SELESAI: |AUTO-STOP' p1_g50/g50_smoke_s$S.log | head -1) | pulang: $(grep -E 'K0 (LULUS|TOLAK)|REFUSE' p1_g50/g50_smoke_s$S.log | tail -1 | cut -c1-80)"
done
echo "== SEMUA SELESAI $(date +%T)"
