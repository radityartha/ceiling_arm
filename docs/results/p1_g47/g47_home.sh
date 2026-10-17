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
# G64-C (docs/p1_g64c.md A0.5/A0.11): a gantry whose BOTH arms are at T1 (+-0.5 deg) is NOT retracted to REST where it
# stands (REST at a T1 stop is what failed); it retraces at T1 (pose_to_g --allow-t1) to p0, unfolds there
# (return_rest --target REST --strict), then parks. Arms neither at REST nor at T1 -> the old REST retract.
T1G=$(python3 - <<'PY'
import math, time, sys
import rclpy
from sensor_msgs.msg import JointState
sys.path.insert(0, '.')
from return_rest import T1, PREFIX, GANTRY
rclpy.init(); n = rclpy.create_node('g47_home_t1'); pos = {}
n.create_subscription(JointState, '/joint_states', lambda m: pos.update(zip(m.name, m.position)), 50)
names = [f'{p}joint_{i}' for p in PREFIX.values() for i in range(1, 7)]
t0 = time.time()
while time.time() - t0 < 5.0 and not all(k in pos for k in names):
    rclpy.spin_once(n, timeout_sec=0.05)
at = lambda a: all(abs(pos[f'{PREFIX[a]}joint_{i}'] - T1[i - 1]) < math.radians(0.5) for i in range(1, 7))  # noqa: E731
print(' '.join(str(g) for g in (1, 2) if all(at(a) for a in PREFIX if GANTRY[a] == g)))
PY
)
echo "gantry dengan lengan di T1: ${T1G:-(tidak ada)}"
for A in "arm_3 arm_4" "arm_1 arm_2"; do
  case "$A" in arm_3*) GA=2;; *) GA=1;; esac
  if [[ " $T1G " == *" $GA "* ]]; then echo "g$GA di T1: retract REST di tempat DILEWATI (buka di p0)"; continue; fi
  python3 -u return_rest.py --arms $A $MV 2>&1 | grep -v "$F" | tail -1
done
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
# rot 1.0 deg = run_g32's own guard: HW legs end up to 0.81 deg SHORT (bridge debounce 0.3 + driver deadband 0.5, G51 B4)
near = lambda a: abs(cur[0] - a[0]) <= 0.002 and abs(cur[1] - a[1]) <= 1.0      # noqa: E731
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
  T1=; [[ " $T1G " == *" $G "* ]] && T1=1
  P0=$(python3 -c "import json,sys; r={x['seed']: x for x in json.load(open(sys.argv[1]))}[int(sys.argv[2])]; print(round(r['p0'][sys.argv[3]], 3))" "$PLAN" "$S" "$G")
  for L in $LEGS; do
    python3 -u $ROOT/docs/results/p1_g32/pose_to_g.py --gantry $G --seed $S --plan $PLAN --variant $V ${L%%:*} ${L##*:} $MV --rot-max 125 \
      ${T1:+--allow-t1} 2>&1 | grep "akhir:\|REFUSE\|DRY\|T_cmd" | tail -2
    if [ -n "$T1" ] && python3 -c "import sys; sys.exit(0 if abs(float(sys.argv[1]) - float(sys.argv[2])) < 1e-6 and abs(float(sys.argv[3])) < 1e-6 else 1)" "${L%%:*}" "$P0" "${L##*:}"; then
      echo "g$G buka T1 -> REST di p0 ($P0, 0)"
      python3 -u return_rest.py --arms $([ $G = 1 ] && echo arm_1 arm_2 || echo arm_3 arm_4) --target REST --strict --no-plan $MV 2>&1 \
        | grep -v "$F" | tail -1
      T1=
    fi
  done
  if [ -n "$T1" ]; then
    echo "g$G buka T1 -> REST di tempat (tidak ada kaki ke p0; return_rest --strict menyaring)"
    python3 -u return_rest.py --arms $([ $G = 1 ] && echo arm_1 arm_2 || echo arm_3 arm_4) --target REST --strict --no-plan $MV 2>&1 \
      | grep -v "$F" | tail -1
  fi
done
python3 $ROOT/scripts/k0_check.py --rail1 1.5 --rail2 0 | tail -1
