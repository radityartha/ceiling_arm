#!/usr/bin/env bash
# DEMO-SYS tugas 2 (MOCK 77, fake HW, BUKAN data): demo_solve jadwal -> run_g54 --dry (plan-only) -> run_g54 eksekusi -> pulang.
# Keadaan awal = sel HW: lengan REST, g1 1.40, g2 0. Penyaring tidak diubah; HW-flag sama dengan runbook (--tau-max 14,
# --goal-correct, --final-retract, --goal-tol-deg 0.05, TANPA --rot-max -> amplop rot_envelope.json).
#   bash mock_sys.sh up | prep | dry <run_dir> | exec <run_dir> | home <run_dir> | down
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1 L=/tmp/p1ds_mock; mkdir -p $L
HZ=/home/user1/Documents/ceiling_arm/docs/results; D=$HZ/p1_demo_sys; SC=/home/user1/Documents/ceiling_arm/scripts
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
step(){ echo "=== $(date +%T) $*"; }
case $1 in
up)
  step mock up; setsid nohup $HZ/p1_cam6dof/harness/mock_up.sh > $L/mock_launch.log 2>&1 < /dev/null &
  echo "launch pid $!" > $L/pids
  for i in $(seq 1 100); do grep -q "You can start planning now" $L/mock_launch.log && break; sleep 3; done
  grep -q "You can start planning now" $L/mock_launch.log || { echo "GAGAL: move_group tidak siap"; exit 1; }
  sleep 10; echo "controllers active: $(timeout 30 ros2 control list_controllers | grep -c active)"
  step monitor + perekam
  setsid nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=$L/p1ds_mon'])" > $L/mon.log 2>&1 < /dev/null &
  echo "mon pid $!" >> $L/pids
  setsid nohup python3 $HZ/p1_g22/js_record.py $L/p1ds_joint_states.csv > $L/rec.log 2>&1 < /dev/null &
  echo "rec pid $!" >> $L/pids; cat $L/pids ;;
prep)
  step REST; for g in "arm_1 arm_2" "arm_3 arm_4"; do python3 $SC/return_rest.py --arms $g --move 2>&1 | grep -E "selesai|TOLAK|GAGAL|REFUSE"; done
  step "g1 -> 1.40 (rencana rel G63, = keadaan HW)"
  python3 $HZ/p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan $HZ/p1_g63/g63_rail_plan.json --variant G63 1.40 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
  python3 $SC/k0_check.py --rail1 1.40 --rail2 0 | tail -2 ;;
dry|exec)
  RD=$2; X=; [ $1 = dry ] && X=--dry
  step "run_g54 $1 $RD"
  python3 -u $HZ/p1_g54/run_g54.py --seed 0 --plan $RD/demo_candidates.json --variant DEMO --final-retract --goal-tol-deg 0.05 \
    --tau-max 14 --goal-correct $X --launch-log $L/mock_launch.log --mon $L/p1ds_mon_samples.csv --out-dir $L/run_$1 \
    --archive $RD/mock --prefix mock_$1_ 2>&1 | grep --line-buffered -v "$F"
  echo "== RC run_g54 ${PIPESTATUS[0]}" ;;
home)
  RD=$2; step "pulang (retrace) DRY lalu gerak"; bash $HZ/p1_g47/g47_home.sh 0 $RD/demo_candidates.json DEMO --dry
  bash $HZ/p1_g47/g47_home.sh 0 $RD/demo_candidates.json DEMO ;;
down)
  step "matikan per PID (anak dulu)"; cat $L/pids
  for k in rec mon; do p=$(awk -v k=$k '$1==k{print $3}' $L/pids); [ -n "$p" ] && kill -INT $p; done
  lp=$(awk '$1=="launch"{print $3}' $L/pids)
  if [ -n "$lp" ]; then kids=$(pgrep -P $lp); for c in $kids; do pgrep -P $c | xargs -r kill -INT; kill -INT $c; done; kill -INT $lp; fi
  sleep 8; echo "sisa node domain 77: $(timeout 15 ros2 node list --no-daemon 2>/dev/null | wc -l)" ;;
esac
