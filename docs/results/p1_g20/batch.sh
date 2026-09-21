#!/bin/bash
# G20 step 5 runner: docs/p1_g20_hw.md A2. One trial = 4-arm TASK -> score -> RETRACT gantry 2, then gantry 1.
#   ./batch.sh FROM TO        rows of g20_plan.txt ("i a1xyz a2xyz a3xyz a4xyz")
# Rails are NEVER commanded (S20). Auto-stop (S22) BEFORE the next step on any trip. Reports only.
# From p1_g19/batch.sh; PIDs FOUND, not hard-coded; every trial archived at once.
FROM=${1:?FROM}; TO=${2:?TO}
D=/home/user1/Documents/ceiling_arm/docs/results/p1_g20
source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash >/dev/null 2>&1
cd /home/user1/Documents/ceiling_arm/scripts
LOG=/tmp/g20_batch.log; LAUNCHLOG=${LAUNCHLOG:-/tmp/g20_t1.log}; REC=/tmp/g20_js.csv; MON=/tmp/g20_step5_samples.csv
FAULT_RE='ARMSTATE_IN_FAULT|Kortex exception|KDetailedException|Failed to create Kortex|ros2_control_node-[0-9]+\]: process has died|\[ERROR\] \[ros2_control_node|bridge: table[12] (REJECTED|NOT armed)'
say(){ echo "$*" | tee -a $LOG; }
stop(){ say "AUTO-STOP before $1: $2"; cp $LOG $D/; exit 3; }
pids(){ for p in /proc/[0-9]*; do c=$(tr '\0' ' ' < $p/cmdline 2>/dev/null); case "$c" in *"$1"*) case "$c" in *bash*|*grep*) ;; *) echo ${p#/proc/};; esac;; esac; done; }
PMON=$(pids 'lib/reachability_gng/reach_dwell_monitor' | head -1); PREC=$(pids 'p1_g20/js_record.py' | head -1); PCTL=$(pids 'controller_manager/ros2_control_node' | head -1)
[ -n "$PMON" ] && [ -n "$PREC" ] && [ -n "$PCTL" ] || { say "REFUSE: monitor=$PMON recorder=$PREC ros2_control=$PCTL"; exit 2; }
say "pids monitor $PMON recorder $PREC ros2_control_node $PCTL; launch log $LAUNCHLOG"
alive(){ [ -d /proc/$PMON ] && [ -d /proc/$PREC ] && [ -d /proc/$PCTL ]; }
base_faults=$(grep -cE "$FAULT_RE" $LAUNCHLOG)
# S22/S23: rails not commanded -> must not move; rotations stay 0. Baseline read from the recorder.
rails(){ python3 - "$REC" <<'PY'
import csv,sys,collections
last={}
for r in csv.DictReader(open(sys.argv[1])):
    if r['joint'] in ('t1_linear_joint','t2_linear_joint','t1_rotation_joint','t2_rotation_joint'): last[r['joint']]=float(r['pos'])
print(' '.join(f"{last.get(k,float('nan')):.6f}" for k in ('t1_linear_joint','t2_linear_joint','t1_rotation_joint','t2_rotation_joint')))
PY
}
read R1 R2 _ _ <<< "$(rails)"; say "rail baseline t1 $R1 t2 $R2"
rail_ok(){ read a b c d <<< "$(rails)"; python3 -c "import math,sys;a,b,c,d,r1,r2=map(float,sys.argv[1:]);sys.exit(0 if abs(a-r1)<=0.002 and abs(b-r2)<=0.002 and abs(math.degrees(c))<=0.5 and abs(math.degrees(d))<=0.5 else 1)" $a $b $c $d $R1 $R2 || { echo "$a $b $c $d"; return 1; }; }
at_rest(){ python3 - "$REC" <<'PY'
import csv,sys,math,time
REST=[-0.46352,0.10710,0.12916,-1.38653,-0.17648,1.73885]; last={}; t_end=None
for r in csv.DictReader(open(sys.argv[1])): last[r['joint']]=(float(r['t']),float(r['pos']))
now=max(t for t,_ in last.values())
bad=[(j,round(math.degrees(abs(last[j][1]-REST[int(j[-1])-1])),2)) for j in last if '_a' in j and (abs(last[j][1]-REST[int(j[-1])-1])>math.radians(0.5) or now-last[j][0]>0.5)]
print(bad) if bad else None; sys.exit(1 if bad or len([j for j in last if '_a' in j])<24 else 0)
PY
}
peak_since(){ python3 - "$1" "$REC" <<'PY'
import csv,sys
t0=float(sys.argv[1]); pk={}
for r in csv.DictReader(open(sys.argv[2])):
    j=r['joint']
    if float(r['t'])>=t0 and '_a' in j and r['eff']:
        a=j[:6]; e=abs(float(r['eff']))
        if e>pk.get(a,(0,''))[0]: pk[a]=(e,j)
print(' '.join(f'{v[0]:.3f}:{v[1]}' for k,v in sorted(pk.items())) or '0.000:none')
PY
}
retract(){ # $1 = tag. Gantry 2 first (moved last), then gantry 1. Each call screened (S18).
  for g in "arm_3 arm_4" "arm_1 arm_2"; do
    t=$(echo $g | tr -d ' _'); python3 -u return_rest.py --arms $g --move > /tmp/g20_rest$1_$t.log 2>&1; rc=$?; cp /tmp/g20_rest$1_$t.log $D/
    say "  RETRACT $g rc=$rc: $(grep -E 'selesai:|sudah di rest|MENOLAK|DITOLAK|tidak ada' /tmp/g20_rest$1_$t.log | tail -1)"
    [ $rc -ne 0 ] && stop "next ($1)" "return_rest $g rc=$rc"
    tp=$(grep -oE "torsi puncak [0-9.]+" /tmp/g20_rest$1_$t.log | awk '{print $3}'); [ -n "$tp" ] && awk "BEGIN{exit !($tp>13.5)}" && stop "next ($1)" "return_rest $g torque $tp > 13.5"
  done; }
while read i a1 a2 a3 a4; do
  [ "$i" -lt "$FROM" ] || [ "$i" -gt "$TO" ] && continue
  alive || stop "trial $i" "monitor/recorder/ros2_control_node not alive"
  nf=$(grep -cE "$FAULT_RE" $LAUNCHLOG); [ "$nf" -gt "$base_faults" ] && stop "trial $i" "launch log: $(grep -E "$FAULT_RE" $LAUNCHLOG | tail -1 | cut -c1-160)"
  x=$(rail_ok) || stop "trial $i" "rail/rotation moved (t1 t2 r1 r2 = $x)"
  x=$(at_rest) || stop "trial $i" "A2: not all four arms at REST (<0.5 deg, fresh <0.5 s): $x"
  T0=$(date +%s.%N); say "===== TRIAL $i start $T0 a1=$a1 a2=$a2 a3=$a3 a4=$a4"
  rm -f /tmp/g20_step5.json
  timeout 900 python3 -u reach_dwell_probe.py --quad --approach 0 --trials 1 --tau-max 12.0 --target=$a1 --target2=$a2 --target3=$a3 --target4=$a4 --move --monitor-csv $MON > /tmp/g20_trial$i.log 2>&1; rc=$?
  cp /tmp/g20_trial$i.log $D/; cp /tmp/g20_step5.json $D/g20_trial$i.json 2>/dev/null
  T1=$(date +%s.%N); pk=$(peak_since $T0)
  # rc != 0 = the probe never ran a trial (g20 B0: argparse ate a negative x). Silent continue = lost trial.
  [ $rc -ne 0 ] && stop "retract $i" "probe rc=$rc: $(grep -E 'error:|Traceback' /tmp/g20_trial$i.log | tail -1 | cut -c1-140)"
  say "  TASK probe rc=$rc | $(grep -E 'moveit ' /tmp/g20_trial$i.log | sed 's/^ *//' | tr '\n' ' ')| $(grep -E '^\s+\[ ?[0-9]+\] ' /tmp/g20_trial$i.log | tail -1 | sed 's/^ *//')"
  say "  measured peak |tau| per arm during task: $pk"
  echo "$i $T0 $T1" >> $D/g20_windows.txt
  for v in $pk; do awk "BEGIN{exit !(${v%%:*}>14.0)}" && stop "retract $i" "measured torque $v > 14 N.m (arms left where they are: operator decides recovery)"; done
  grep -q "UNSCREENED" /tmp/g20_trial$i.log && stop "retract $i" "screen could not run (S9/S18)"
  grep -q "TIDAK VALID (mesin):" /tmp/g20_trial$i.log && stop "retract $i" "trial $i INVALID: $(grep 'TIDAK VALID (mesin):' /tmp/g20_trial$i.log | head -1 | cut -c1-120)"
  inv=$(grep -oE "di LUAR penyebut\): .*" /tmp/g20_trial$i.log | sed 's/di LUAR penyebut): //')
  case "$inv" in "tidak ada"|"{'TORQUE-ABORT': 1}"|"{'HALTED': 1}"|"") ;; *) stop "retract $i" "trial $i INVALID bucket: $inv";; esac
  case "$inv" in "{'TORQUE-ABORT': 1}"|"{'HALTED': 1}") say "  NOTE: probe bucket $inv -- scored as an outcome (g18 B2.1/B2.2), not a stop";; esac
  nf=$(grep -cE "$FAULT_RE" $LAUNCHLOG); [ "$nf" -gt "$base_faults" ] && stop "retract $i" "launch log after task: $(grep -E "$FAULT_RE" $LAUNCHLOG | tail -1 | cut -c1-160)"
  retract $i
done < $D/g20_plan.txt
cp $LOG $D/; say "BATCH $FROM..$TO DONE"
