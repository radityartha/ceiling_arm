#!/usr/bin/env bash
# G64-C fase 4 (MOCK 77, fake HW, BUKAN data; docs/p1_g64c.md C5-C8). Penyaring tidak diubah; flag = runbook HW.
#   bash g64c_mock.sh up | down
#   bash g64c_mock.sh prep <g1 rel>                 lengan REST + g1 -> rel (1.50 = PARK, 1.40 = rencana rel G63), K0
#   bash g64c_mock.sh dryold <seed>                 C5 hidup: run_g54 --dry HEAD-sebelum-fase-3 vs sekarang (g55_selected R90sel)
#   bash g64c_mock.sh screen <seed> <plan> <V> <tag>  plan-only k=3 (g31_screen --task-cli)
#   bash g64c_mock.sh smoke <seed> <plan> <V> <tag>   run_g54 eksekusi + pulang retrace + K0 (= g54_smoke, --tau-max 14)
#   bash g64c_mock.sh tas dry|exec <tag>            TAS run_c3/demo_candidates.json seed 0 DEMO (flag = mock_sys.sh)
#   bash g64c_mock.sh fold <g> <k>                  return_rest fold T1 / unfold REST di tempat, k kali, waktu dinding
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1
ROOT=/home/user1/Documents/ceiling_arm; HZ=$ROOT/docs/results; D=$HZ/p1_g64c; SC=$ROOT/scripts
L=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/11d2133c-65ff-4529-9e37-bd2e52b6f489/scratchpad/mock; mkdir -p $L $D/mock
export LAUNCH_LOG=$L/mock_launch.log MON=$L/g64c_mon_samples.csv
SEL=$HZ/p1_g55/g55_selected.json; TAS=$D/run_c3/demo_candidates.json
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
step(){ echo "=== $(date +%T) $*"; }
case $1 in
up)
  step mock up; setsid nohup $HZ/p1_cam6dof/harness/mock_up.sh > $LAUNCH_LOG 2>&1 < /dev/null &
  echo "launch pid $!" > $L/pids
  for i in $(seq 1 100); do grep -q "You can start planning now" $LAUNCH_LOG && break; sleep 3; done
  grep -q "You can start planning now" $LAUNCH_LOG || { echo "GAGAL: move_group tidak siap"; exit 1; }
  sleep 10; echo "controllers active: $(timeout 30 ros2 control list_controllers | grep -c active)"
  step monitor + perekam
  setsid nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=$L/g64c_mon'])" > $L/mon.log 2>&1 < /dev/null &
  echo "mon pid $!" >> $L/pids
  setsid nohup python3 $HZ/p1_g22/js_record.py $L/g64c_joint_states.csv > $L/rec.log 2>&1 < /dev/null &
  echo "rec pid $!" >> $L/pids; cat $L/pids ;;
prep)
  step REST; for g in "arm_1 arm_2" "arm_3 arm_4"; do python3 $SC/return_rest.py --arms $g --move 2>&1 | grep -E "selesai|TOLAK|GAGAL|REFUSE|sudah"; done
  if [ "$2" = "1.50" ]; then
    python3 $HZ/p1_g32/pose_to_g.py --gantry 1 --seed 7 --plan $SEL --variant R90sel 1.50 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
  else
    python3 $HZ/p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan $HZ/p1_g63/g63_rail_plan.json --variant G63 $2 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
  fi
  python3 $SC/k0_check.py --rail1 $2 --rail2 0 | tail -2 ;;
dryold)
  S=$2; O=$L/before; mkdir -p $O
  [ -d $O/docs ] || git -C $ROOT archive 6edb61b docs/results/p1_g54 docs/results/p1_g31 docs/results/p1_g32 docs/results/p1_g22 \
      docs/results/p1_g29 docs/results/p1_g48 | tar -x -C $O --exclude='*.jsonl' --exclude='*.npz' --exclude='*.gz'
  for W in before:$O/docs/results now:$HZ; do
    IFS=: read T B <<< "$W"
    python3 -u $B/p1_g54/run_g54.py --seed $S --plan $SEL --variant R90sel --final-retract --goal-tol-deg 0.05 --tau-max 14 --dry \
      --launch-log $LAUNCH_LOG --mon $MON --out-dir $L/dry_${T}_s$S --archive $L --prefix dry_${T}_s${S}_ > $L/dry_${T}_s$S.log 2>&1
    echo "DRY $T s$S rc $?: $(grep -cE '  perintah g' $L/dry_${T}_s$S.log) perintah | $(grep -E 'K0 (LULUS|TOLAK)' $L/dry_${T}_s$S.log | tail -1)"
    grep -E '  perintah g' $L/dry_${T}_s$S.log | sed -E 's/^[0-9.]+ +//; s#'"$O"'/docs/results#'"$HZ"'#g' > $L/dry_${T}_s$S.argv
  done
  diff <(sed "s/dry_before_s$S/OUT/" $L/dry_before_s$S.argv) <(sed "s/dry_now_s$S/OUT/" $L/dry_now_s$S.argv) > /dev/null && echo "C5 s$S: argv SAMA ($(wc -l < $L/dry_now_s$S.argv) baris)" \
    || { echo "C5 s$S: argv BEDA"; diff $L/dry_before_s$S.argv $L/dry_now_s$S.argv | head -20; } ;;
screen)
  S=$2; P=$3; V=$4; T=$5
  G47_GOAL_TOL_DEG=0.05 python3 -u $HZ/p1_g31/g31_screen.py --plan $P --variant $V --final-retract --task-cli --repeats 3 \
    --seeds $S --out $D/mock/planonly_$T.json --traj $L/planonly_$T.jsonl 2>&1 | grep --line-buffered -v "$F" ;;
smoke)
  S=$2; P=$3; V=$4; T=$5
  python3 -u $HZ/p1_g54/run_g54.py --seed $S --plan $P --variant $V --final-retract --goal-tol-deg 0.05 --tau-max 14 \
    --launch-log $LAUNCH_LOG --mon $MON --out-dir $L/run_$T --archive $D/mock --prefix ${T}_ 2>&1 | grep --line-buffered -v "$F"
  echo "== RC run_g54 ${PIPESTATUS[0]}"
  bash $HZ/p1_g47/g47_home.sh $S $P $V
  echo "== SELESAI $(date +%T)" ;;
tas)
  X=; [ $2 = dry ] && X=--dry; T=$3
  python3 -u $HZ/p1_g54/run_g54.py --seed 0 --plan $TAS --variant DEMO --final-retract --goal-tol-deg 0.05 --tau-max 14 \
    --goal-correct $X --launch-log $LAUNCH_LOG --mon $MON --out-dir $L/run_$T --archive $D/mock --prefix ${T}_ 2>&1 \
    | grep --line-buffered -v "$F"
  echo "== RC run_g54 ${PIPESTATUS[0]}" ;;
fold)
  G=$2; K=$3; A="arm_1 arm_2"; [ $G = 2 ] && A="arm_3 arm_4"; cd $SC
  for k in $(seq 1 $K); do
    for X in T1 REST; do
      t0=$(date +%s.%N); python3 -u return_rest.py --arms $A --move --no-plan --target $X --strict > $L/fold_g${G}_${X}_$k.log 2>&1
      rc=$?; t1=$(date +%s.%N)
      echo "fold g$G k$k -> $X rc $rc wall $(python3 -c "print(f'{$t1-$t0:.2f}')") s | $(grep -E 'selesai|TUCK|MENOLAK' $L/fold_g${G}_${X}_$k.log | tail -1)"
      [ $rc = 0 ] || { echo "BERHENTI: rc $rc"; exit 1; }
    done
  done ;;
down)
  step "matikan per PID (anak dulu)"; cat $L/pids
  for k in rec mon; do p=$(awk -v k=$k '$1==k{print $3}' $L/pids); [ -n "$p" ] && kill -INT $p; done
  lp=$(awk '$1=="launch"{print $3}' $L/pids)
  if [ -n "$lp" ]; then kids=$(pgrep -P $lp); for c in $kids; do pgrep -P $c | xargs -r kill -INT; kill -INT $c; done; kill -INT $lp; fi
  sleep 8; echo "sisa node domain 77: $(timeout 15 ros2 node list --no-daemon 2>/dev/null | wc -l)" ;;
esac
