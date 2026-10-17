#!/usr/bin/env bash
# G64-C fase 5 HW NYATA (domain 0; docs/p1_g64c.md C9). Pola = ../g64c_mock.sh tanpa domain 77. Satu gerak per izin operator.
#   bash g64c_hw.sh up | down | k0 <rel1> <rel2>
#   bash g64c_hw.sh fold <T1|REST> dry|move <arm> [<arm>]   return_rest --no-plan --target X --strict (+ tahan 15 s + torsi)
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1 DISPLAY=:1
ROOT=/home/user1/Documents/ceiling_arm; HZ=$ROOT/docs/results; D=$HZ/p1_g64c; H=$D/hw; SC=$ROOT/scripts
L=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/59cfa971-0483-4e1e-96e6-a4c1bf0fde18/scratchpad/hw; mkdir -p $L
export LAUNCH_LOG=$L/hw_launch.log MON=$L/g64c_hw_mon_samples.csv REC=$L/g64c_hw_joint_states.csv
HOLD=15; TAS=${PLAN:-$D/run_c3_cfold44.83_c3/demo_candidates.json}; G=${TAG:-tas}   # PLAN/TAG: jadwal lain (botol lemari)
F="UserWarning|buildGeomFromUrdf|self.geom = |V = np.array"
step(){ echo "=== $(date +%T) $*"; }
case $1 in
up)
  step stack up; setsid nohup $HZ/p1_demo_hw/harness/stack_up.sh > $LAUNCH_LOG 2>&1 < /dev/null &
  echo "launch pid $!" > $L/pids
  for i in $(seq 1 100); do grep -q "You can start planning now" $LAUNCH_LOG && break; sleep 3; done
  grep -q "You can start planning now" $LAUNCH_LOG || { echo "GAGAL: move_group tidak siap"; exit 1; }
  sleep 15; timeout 30 ros2 control list_controllers
  grep -E "ARMED|NOT armed|not initialized|No response|exception|failed" $LAUNCH_LOG | cut -c1-200 | tail -8
  step monitor + perekam
  setsid nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=$L/g64c_hw_mon'])" > $L/mon.log 2>&1 < /dev/null &
  echo "mon pid $!" >> $L/pids
  setsid nohup python3 $HZ/p1_g22/js_record.py $REC > $L/rec.log 2>&1 < /dev/null &
  echo "rec pid $!" >> $L/pids; cat $L/pids
  sleep 5; ls -la $REC $MON 2>&1 ;;
k0)
  python3 $SC/k0_check.py --rail1 $2 --rail2 $3 | tail -3 ;;
fold)
  X=$2; M=; [ $3 = move ] && M=--move; shift 3; A="$@"; T=$(echo $A | tr ' ' _)_$X; P=
  for a in $A; do case $a in arm_1) P="$P t1_a1_";; arm_2) P="$P t1_a2_";; arm_3) P="$P t2_a1_";; arm_4) P="$P t2_a2_";; esac; done
  cd $SC; t0=$(date +%s.%N)
  python3 -u return_rest.py --arms $A $M --no-plan --target $X --strict > $L/fold_${T}_${M:-dry}.log 2>&1; rc=$?; t1=$(date +%s.%N)
  grep -vE "UserWarning|buildGeomFromUrdf|self.geom = |V = np.array" $L/fold_${T}_${M:-dry}.log | tail -8
  echo "fold $A -> $X ${M:-DRY} rc $rc wall $(python3 -c "print(f'{$t1-$t0:.2f}')") s  t0 $t0 t1 $t1"
  [ -z "$M" ] && exit $rc
  echo "tahan $HOLD s (tanpa gerak) ..."; sleep $HOLD; th=$(date +%s.%N)
  python3 $H/g64c_tq.py $REC $t0 $t1 $th $P | tee $L/tq_${T}.txt; trc=${PIPESTATUS[0]}
  echo "$T rc $rc t0 $t0 t1 $t1 th $th tq_rc $trc" >> $L/moves.txt
  exit $(( rc || trc )) ;;
trav)
  # g2 traverse dengan lengan T1 (= argv traverse_cmd run_g54 + --allow-t1); torsi kedua lengan g2 selama sapuan + tahan.
  LIN=$2; ROT=$3; M=; [ $4 = move ] && M=--move; T=trav_${LIN}_${ROT}
  t0=$(date +%s.%N)
  python3 -u $HZ/p1_g32/pose_to_g.py --gantry 2 --seed 0 --plan $D/run_c3_cfold44.83_c3/demo_candidates.json --variant DEMO \
    $LIN $ROT $M --allow-t1 > $L/${T}_${M:-dry}.log 2>&1; rc=$?; t1=$(date +%s.%N)
  grep -vE "UserWarning|buildGeomFromUrdf|self.geom = |V = np.array|^\{" $L/${T}_${M:-dry}.log | tail -8 | cut -c1-220
  echo "trav g2 -> ($LIN, $ROT) ${M:-DRY} rc $rc wall $(python3 -c "print(f'{$t1-$t0:.2f}')") s  t0 $t0 t1 $t1"
  [ -z "$M" ] && exit $rc
  echo "tahan $HOLD s (tanpa gerak) ..."; sleep $HOLD; th=$(date +%s.%N)
  python3 $H/g64c_tq.py $REC $t0 $t1 $th t2_a1_ t2_a2_ | tee $L/tq_${T}.txt; trc=${PIPESTATUS[0]}
  echo "$T rc $rc t0 $t0 t1 $t1 th $th tq_rc $trc" >> $L/moves.txt
  exit $(( rc || trc )) ;;
prep)
  # langkah 5, NOL gerak: plan-only nyata k=3 (g31_screen --task-cli) + run_g54 --dry (= p1_g57/p1hw_prep.sh, TAS DEMO s0)
  step plan-only k=3
  G47_GOAL_TOL_DEG=0.05 python3 -u $HZ/p1_g31/g31_screen.py --plan $TAS --variant DEMO --final-retract --task-cli --repeats 3 \
    --seeds 0 --out $H/planonly_$G.json --traj $L/planonly_$G.jsonl 2>&1 | grep -vE "$F" > $L/planonly_$G.log; echo "rc ${PIPESTATUS[0]}"
  grep -E "LOLOS|GAGAL|PLANNED|Traceback|Error|TUCK|START-JUMP" $L/planonly_$G.log | tail -12 | cut -c1-200
  step DRY run_g54
  python3 -u $HZ/p1_g54/run_g54.py --seed 0 --plan $TAS --variant DEMO --final-retract --goal-tol-deg 0.05 --tau-max 14 \
    --goal-correct --dry --launch-log $LAUNCH_LOG --mon $MON --out-dir $L/run_${G}_dry --archive $H --prefix ${G}_dry_ > $L/${G}_dry.log 2>&1
  echo "DRY rc $?"; grep -E "K0 (LULUS|TOLAK)|perintah g|TOLAK|REFUS" $L/${G}_dry.log | sed -E 's#'"$HZ"'/##g' | cut -c1-260 ;;
tas)
  # langkah 5 NYATA: satu run_g54 lepas (setsid nohup), log baris-per-baris
  setsid nohup python3 -u $HZ/p1_g54/run_g54.py --seed 0 --plan $TAS --variant DEMO --final-retract --goal-tol-deg 0.05 \
    --tau-max 14 --goal-correct --launch-log $LAUNCH_LOG --mon $MON --out-dir $L/run_$G --archive $H --prefix ${G}_ \
    > $L/$G.log 2>&1 < /dev/null &
  echo "start $(date +%T) log $L/$G.log (pid via ps, setsid fork)" ;;
home)
  # pulang = g47_home.sh retrace (gantry T1 -> --allow-t1 ke p0 -> buka -> PARK); $2 = dry | move
  X=; [ $2 = dry ] && X=--dry
  bash $HZ/p1_g47/g47_home.sh 0 $TAS DEMO $X 2>&1 | grep -vE "$F" | tee $L/home_${G}_$2.log | cut -c1-220 | tail -30 ;;
down)
  step "matikan per PID (anak dulu)"; cat $L/pids
  for k in rec mon; do p=$(awk -v k=$k '$1==k{print $3}' $L/pids); [ -n "$p" ] && { pgrep -P $p | xargs -r kill -INT; kill -INT $p; }; done
  lp=$(awk '$1=="launch"{print $3}' $L/pids)
  if [ -n "$lp" ]; then kids=$(pgrep -P $lp); for c in $kids; do pgrep -P $c | xargs -r kill -INT; kill -INT $c; done; kill -INT $lp; fi
  sleep 25; echo "sisa node domain 0: $(timeout 15 ros2 node list --no-daemon 2>/dev/null | wc -l)" ;;
esac
