#!/usr/bin/env bash
# G47 A1.1: DRY-CLI k=3 --final-retract on the 6 candidates (mock domain 77, no --move).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
python3 -u p1_g31/g31_screen.py --plan p1_g43/g43_candidates_6.json --variant R180 --final-retract --task-cli \
  --repeats 3 --seeds 0 7 10 3 18 --out p1_g47/g47_drycli.json --traj p1_g47/g47_drycli_plans.jsonl 2>&1 | grep -v "$F"
python3 -u p1_g31/g31_screen.py --plan p1_g42/g42f_candidates.json --variant G42 --final-retract --task-cli \
  --repeats 3 --seeds 8 --out p1_g47/g47_drycli.json --traj p1_g47/g47_drycli_plans.jsonl 2>&1 | grep -v "$F"
echo "== SELESAI $(date +%T)"
