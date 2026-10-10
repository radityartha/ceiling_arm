#!/usr/bin/env bash
# G47 B4: go home by RETRACING the gantry's own schedule backwards (every leg = a traverse the schedule already uses,
# screened CLEAR; same lin x rot rectangle reversed), then PARK g1 (1.5, 0) / p0 g2 (0, 0). Arms to REST first.
# A direct last-stop -> PARK sweep is NOT screened by the scheduler (G47 B4: s0 g1 (0.80,-125) -> (1.5,0) REFUSED by G33).
# bash g47_home.sh <seed> <plan json> <variant> [--dry]
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
ROOT=/home/user1/Documents/ceiling_arm; S=$1; PLAN=$2; V=$3; MV=--move; [ "${4:-}" = "--dry" ] && MV=
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
cd $ROOT/scripts
for A in "arm_3 arm_4" "arm_1 arm_2"; do python3 -u return_rest.py --arms $A $MV 2>&1 | grep -v "$F" | tail -1; done
for G in 1 2; do
  LEGS=$(python3 - "$PLAN" "$S" "$V" "$G" <<'PY'
import json, math, sys, time
import rclpy
from sensor_msgs.msg import JointState
plan, s, v, g = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
r = {x['seed']: x for x in json.load(open(plan))}[s]
st = [(round(x['rail_m'], 3), round(x['rot_deg'], 3)) for x in r[v]['schedule'][g]]
home = (1.5, 0.0) if g == '1' else (0.0, 0.0)
p0 = (round(r['p0'][g], 3), 0.0)
rclpy.init(); n = rclpy.create_node('g47_home_pose'); pos = {}
n.create_subscription(JointState, '/joint_states', lambda m: pos.update(zip(m.name, m.position)), 50)
t0 = time.time()
while time.time() - t0 < 5.0 and not all(k in pos for k in (f't{g}_linear_joint', f't{g}_rotation_joint')):
    rclpy.spin_once(n, timeout_sec=0.05)
cur = (pos[f't{g}_linear_joint'], math.degrees(pos[f't{g}_rotation_joint']))
near = lambda a: abs(cur[0] - a[0]) <= 0.002 and abs(cur[1] - a[1]) <= 0.5      # noqa: E731
if near(home):
    print('')                                    # already home: no leg
elif any(near(x) for x in st):
    k = max(i for i, x in enumerate(st) if near(x))
    legs = list(reversed(st[:k])) + [p0] + ([home] if home != p0 else [])
    print(' '.join(f'{a}:{b}' for a, b in legs))
elif near(p0):
    print(f'{home[0]}:{home[1]}')
else:
    print('TOLAK')                               # not at any planned pose: do not invent a route
PY
)
  echo "g$G pulang lewat: ${LEGS:-(sudah di home)}"
  [ "$LEGS" = "TOLAK" ] && { echo "REFUSE: g$G tidak di pose jadwal mana pun -- rute pulang tidak dikarang"; exit 1; }
  for L in $LEGS; do
    python3 -u $ROOT/docs/results/p1_g32/pose_to_g.py --gantry $G --seed $S --plan $PLAN --variant $V ${L%%:*} ${L##*:} $MV --rot-max 125 2>&1 \
      | grep "akhir:\|REFUSE\|DRY\|T_cmd" | tail -2
  done
done
python3 $ROOT/scripts/k0_check.py --rail1 1.5 --rail2 0 | tail -1
