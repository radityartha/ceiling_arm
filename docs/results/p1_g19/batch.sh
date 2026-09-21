#!/bin/bash
# G19 step 4 runner: docs/p1_g19_hw.md A2. One trial = RETRACT -> TRAVERSE -> EXTEND+TASK.
#   ./batch.sh FROM TO        rows of g19_plan.txt ("i rail a1xyz a2xyz"); i = 0 is task 0 (no move)
# Auto-stop (S17) BEFORE the next step on any trip. Reports only; never loosens a screen.
# From g18 batch.sh; PIDs are FOUND, not hard-coded, and every trial is archived at once.
FROM=${1:?FROM}; TO=${2:?TO}
D=/home/user1/Documents/ceiling_arm/docs/results/p1_g19
source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash >/dev/null 2>&1
cd /home/user1/Documents/ceiling_arm/scripts
LOG=/tmp/g19_batch.log; LAUNCHLOG=${LAUNCHLOG:-/tmp/g18_t3.log}; REC=/tmp/g19_js.csv; MON=/tmp/g19_step4_samples.csv
FAULT_RE='ARMSTATE_IN_FAULT|Kortex exception|KDetailedException|Failed to create Kortex|ros2_control_node-[0-9]+\]: process has died|\[ERROR\] \[ros2_control_node|bridge: table1 (REJECTED|NOT armed)'
say(){ echo "$*" | tee -a $LOG; }
stop(){ say "AUTO-STOP before $1: $2"; exit 3; }
pids(){ for p in /proc/[0-9]*; do c=$(tr '\0' ' ' < $p/cmdline 2>/dev/null); case "$c" in *"$1"*) case "$c" in *bash*|*grep*) ;; *) echo ${p#/proc/};; esac;; esac; done; }
PMON=$(pids 'lib/reachability_gng/reach_dwell_monitor' | head -1); PREC=$(pids 'js_record.py' | head -1); PCTL=$(pids 'controller_manager/ros2_control_node' | head -1)
[ -n "$PMON" ] && [ -n "$PREC" ] && [ -n "$PCTL" ] || { say "REFUSE: monitor=$PMON recorder=$PREC ros2_control=$PCTL"; exit 2; }
say "pids monitor $PMON recorder $PREC ros2_control_node $PCTL; launch log $LAUNCHLOG"
alive(){ [ -d /proc/$PMON ] && [ -d /proc/$PREC ] && [ -d /proc/$PCTL ]; }
base_faults=$(grep -cE "$FAULT_RE" $LAUNCHLOG)
peak_since(){ python3 - "$1" "$REC" <<'PY'
import csv,sys
t0=float(sys.argv[1]); pk,pj=0.0,''
for r in csv.DictReader(open(sys.argv[2])):
    if float(r['t'])>=t0 and r['joint'].startswith('t1_a') and r['eff']:
        e=abs(float(r['eff']))
        if e>pk: pk,pj=e,r['joint']
print(f'{pk:.3f} {pj}')
PY
}
while read i rail a1 a2; do
  [ "$i" -lt "$FROM" ] || [ "$i" -gt "$TO" ] && continue
  alive || stop "trial $i" "monitor/recorder/ros2_control_node not alive"
  nf=$(grep -cE "$FAULT_RE" $LAUNCHLOG); [ "$nf" -gt "$base_faults" ] && stop "trial $i" "launch log: $(grep -E "$FAULT_RE" $LAUNCHLOG | tail -1 | cut -c1-160)"
  T0=$(date +%s.%N); say "===== TRIAL $i start $T0 rail->$rail a1=$a1 a2=$a2"
  if [ "$i" -gt 0 ]; then
    python3 -u return_rest.py --arms arm_1 arm_2 --move > /tmp/g19_rest$i.log 2>&1; rc=$?; cp /tmp/g19_rest$i.log $D/
    say "  RETRACT rc=$rc: $(grep -E 'selesai:|sudah di rest|MENOLAK|DITOLAK|tidak ada' /tmp/g19_rest$i.log | tail -1)"
    [ $rc -ne 0 ] && stop "traverse $i" "return_rest rc=$rc"
    tp=$(grep -oE "torsi puncak [0-9.]+" /tmp/g19_rest$i.log | awk '{print $3}'); [ -n "$tp" ] && awk "BEGIN{exit !($tp>13.5)}" && stop "traverse $i" "return_rest torque $tp > 13.5"
    T1=$(date +%s.%N)
    python3 -u $D/rail_to.py $rail --move > /tmp/g19_rail$i.log 2>&1; rc=$?; cp /tmp/g19_rail$i.log $D/
    say "  TRAVERSE rc=$rc: $(grep -E '^akhir:|REFUSE|DITOLAK' /tmp/g19_rail$i.log | tail -1)"
    [ $rc -ne 0 ] && stop "task $i" "rail_to rc=$rc (1 = refused/failed, 3 = S17 drift/error)"
    nf=$(grep -cE "$FAULT_RE" $LAUNCHLOG); [ "$nf" -gt "$base_faults" ] && stop "task $i" "launch log after traverse: $(grep -E "$FAULT_RE" $LAUNCHLOG | tail -1 | cut -c1-160)"
  else T1=$T0; fi
  T2=$(date +%s.%N)
  rm -f /tmp/g17_step3.json
  timeout 600 python3 -u reach_dwell_probe.py --dual --approach 0 --trials 1 --tau-max 12.0 --target $a1 --target2 $a2 --move --monitor-csv $MON > /tmp/g19_trial$i.log 2>&1; rc=$?
  cp /tmp/g19_trial$i.log $D/; cp /tmp/g17_step3.json $D/g19_trial$i.json 2>/dev/null
  T3=$(date +%s.%N); pk=$(peak_since $T2)
  say "  TASK probe rc=$rc | $(grep -E 'moveit ' /tmp/g19_trial$i.log | sed 's/^ *//' | tr '\n' ' ')| $(grep -E '^\s+\[ ?[0-9]+\] ' /tmp/g19_trial$i.log | tail -1 | sed 's/^ *//')"
  say "  measured peak |tau| during task: $pk N.m"
  echo "$i $rail $T0 $T1 $T2 $T3" >> $D/g19_windows.txt
  grep -q "TIDAK VALID (mesin):" /tmp/g19_trial$i.log && stop "trial $((i+1))" "trial $i INVALID: $(grep 'TIDAK VALID (mesin):' /tmp/g19_trial$i.log | head -1 | cut -c1-120)"
  inv=$(grep -oE "di LUAR penyebut\): .*" /tmp/g19_trial$i.log | sed 's/di LUAR penyebut): //')
  case "$inv" in "tidak ada"|"{'TORQUE-ABORT': 1}"|"{'HALTED': 1}"|"") ;; *) stop "trial $((i+1))" "trial $i INVALID bucket: $inv";; esac
  case "$inv" in "{'TORQUE-ABORT': 1}"|"{'HALTED': 1}") say "  NOTE: probe bucket $inv -- scored as an outcome (g18 B2.1/B2.2), not a stop";; esac
  awk "BEGIN{exit !(${pk%% *}>14.0)}" && stop "trial $((i+1))" "measured torque ${pk} > 14 N.m"
done < $D/g19_plan.txt
cp $LOG $D/; say "BATCH $FROM..$TO DONE"
