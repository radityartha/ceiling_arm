#!/bin/bash
# G18 batch: pairs 2..10, auto-stop BEFORE the next pair on any trip. Reports only; never loosens a screen.
SP=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/d95e352d-d30a-4acc-99ad-efa21db00d59/scratchpad
source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash >/dev/null 2>&1
cd /home/user1/Documents/ceiling_arm/scripts
LOG=/tmp/g18_batch.log; LAUNCHLOG=/tmp/g18_t3.log; REC=/tmp/g18_js_pair1.csv
FAULT_RE='ARMSTATE_IN_FAULT|Kortex exception|KDetailedException|Failed to create Kortex|ros2_control_node-[0-9]+\]: process has died|\[ERROR\] \[ros2_control_node'
base_faults=$(grep -cE "$FAULT_RE" $LAUNCHLOG)
say(){ echo "$*" | tee -a $LOG; }
stop(){ say "AUTO-STOP before pair $1: $2"; exit 3; }
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
alive(){ [ -d /proc/875854 ] && [ -d /proc/875837 ] && [ -d /proc/872083 ]; }
while read i a1 a2; do
  [ "$i" -lt 7 ] && continue
  alive || stop $i "monitor/recorder/ros2_control_node not alive"
  nf=$(grep -cE "$FAULT_RE" $LAUNCHLOG); [ "$nf" -gt "$base_faults" ] && stop $i "fault/exception in launch log: $(grep -E "$FAULT_RE" $LAUNCHLOG | tail -1 | cut -c1-160)"
  T0=$(date +%s.%N); say "===== PAIR $i start $T0  a1=$a1 a2=$a2"
  python3 -u return_rest.py --arms arm_1 arm_2 --move > /tmp/g18_rest$i.log 2>&1; rc=$?
  rr=$(grep -E "selesai:|sudah di rest|MENOLAK|DITOLAK|tidak ada" /tmp/g18_rest$i.log | tail -1); say "  return_rest rc=$rc: $rr"
  [ $rc -ne 0 ] && stop $i "return_rest rc=$rc"
  tp=$(grep -oE "torsi puncak [0-9.]+" /tmp/g18_rest$i.log | awk '{print $3}'); [ -n "$tp" ] && awk "BEGIN{exit !($tp>13.5)}" && stop $i "return_rest torque $tp > 13.5"
  T1=$(date +%s.%N)
  timeout 600 python3 -u reach_dwell_probe.py --dual --approach 0 --trials 1 --tau-max 12.0 --target $a1 --target2 $a2 --move --monitor-csv /tmp/g18_step3_samples.csv > /tmp/g18_trial$i.log 2>&1; rc=$?
  cp /tmp/g17_step3.json /tmp/g18_trial$i.json 2>/dev/null
  T2=$(date +%s.%N)
  v=$(grep -E "^\s+\[ ?[0-9]+\] " /tmp/g18_trial$i.log | tail -1 | sed 's/^ *//'); mv=$(grep -E "moveit " /tmp/g18_trial$i.log | sed 's/^ *//' | tr '\n' ' ')
  pk=$(peak_since $T1)
  say "  probe rc=$rc | $mv| $v"
  say "  measured peak |tau| during trial: $pk N.m   window $T1 .. $T2"
  echo "$i $T0 $T1 $T2" >> /tmp/g18_windows.txt
  # A5 machine cause only. The probe also files TORQUE-ABORT under "TIDAK VALID"; per g16 B5.1 that is
  # scored on A1 (torque reported separately), so it is logged, not a stop.
  grep -q "TIDAK VALID (mesin):" /tmp/g18_trial$i.log && stop $((i+1)) "pair $i INVALID: $(grep 'TIDAK VALID (mesin):' /tmp/g18_trial$i.log | head -1 | cut -c1-120)"
  inv=$(grep -oE "di LUAR penyebut\): .*" /tmp/g18_trial$i.log | sed 's/di LUAR penyebut): //')
  # HALTED (feasibility, in the denominator -- g17 B3) and TORQUE-ABORT (observer, g16 B5.1) are OUTCOMES.
  case "$inv" in "tidak ada"|"{'TORQUE-ABORT': 1}"|"{'HALTED': 1}"|"") ;; *) stop $((i+1)) "pair $i INVALID bucket: $inv";; esac
  case "$inv" in "{'TORQUE-ABORT': 1}"|"{'HALTED': 1}") say "  NOTE: probe bucket $inv -- scored as an outcome, not a stop";; esac
  awk "BEGIN{exit !(${pk%% *}>14.0)}" && stop $((i+1)) "measured torque ${pk} > 14 N.m"
done < $SP/b2_pairs.txt
say "BATCH DONE"
