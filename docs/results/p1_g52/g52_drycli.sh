#!/usr/bin/env bash
# G52 A2.5: DRY-CLI k=3 --final-retract (tol 0.05) on the 5 new R90 schedules (mock domain 77, no --move). Pattern G50.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 G47_GOAL_TOL_DEG=0.05
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
python3 -u p1_g31/g31_screen.py --plan p1_g52/g52_candidates.json --variant R90 --final-retract --task-cli --repeats 3 \
  --seeds 0 2 11 12 17 --out p1_g52/g52_drycli.json --traj p1_g52/g52_drycli_plans.jsonl 2>&1 | grep --line-buffered -v "$F"
echo "== DRY-CLI SELESAI $(date +%T)"
