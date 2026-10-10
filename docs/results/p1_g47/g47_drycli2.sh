#!/usr/bin/env bash
# G47 B1: round 2 -- s10 s3 (R180) + s8 (G42) k=3 with start_valid re-send; then A/B s7 s18 with task goal tol 0.05 deg.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
python3 -u p1_g31/g31_screen.py --plan p1_g43/g43_candidates_6.json --variant R180 --final-retract --task-cli \
  --repeats 3 --seeds 10 3 --out p1_g47/g47_drycli2.json --traj p1_g47/g47_drycli2_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
python3 -u p1_g31/g31_screen.py --plan p1_g42/g42f_candidates.json --variant G42 --final-retract --task-cli \
  --repeats 3 --seeds 8 --out p1_g47/g47_drycli2.json --traj p1_g47/g47_drycli2_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
echo "== A/B tol 0.05 $(date +%T)"
G47_GOAL_TOL_DEG=0.05 python3 -u p1_g31/g31_screen.py --plan p1_g43/g43_candidates_6.json --variant R180 --final-retract \
  --task-cli --repeats 3 --seeds 7 18 --out p1_g47/g47_ab005.json --traj p1_g47/g47_ab005_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
echo "== SELESAI $(date +%T)"
