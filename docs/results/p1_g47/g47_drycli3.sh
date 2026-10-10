#!/usr/bin/env bash
# G47 B2/B3: DRY-CLI k=3 tol 0.05 on s0 s3 s10 (R180) + s8 (G42), then smoke s0 + s7 (--goal-tol-deg 0.05).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
export G47_GOAL_TOL_DEG=0.05
python3 -u p1_g31/g31_screen.py --plan p1_g43/g43_candidates_6.json --variant R180 --final-retract --task-cli \
  --repeats 3 --seeds 0 3 10 --out p1_g47/g47_drycli3.json --traj p1_g47/g47_drycli3_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
python3 -u p1_g31/g31_screen.py --plan p1_g42/g42f_candidates.json --variant G42 --final-retract --task-cli \
  --repeats 3 --seeds 8 --out p1_g47/g47_drycli3.json --traj p1_g47/g47_drycli3_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
echo "== DRY-CLI 0.05 SELESAI $(date +%T)"
