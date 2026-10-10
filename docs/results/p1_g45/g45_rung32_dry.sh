#!/usr/bin/env bash
# G45 A2: run_g32 --dry on ROS_DOMAIN_ID=78 with fake_js (F0 match / F1 arm_3 +1 deg). Sends nothing.
# monitor + ros2_control_node pids come from the mock (domain 77): run_g32 only checks they are alive.
export ROS_DOMAIN_ID=78
H=$(cd $(dirname $0) && pwd); R=/home/user1/Documents/ceiling_arm/docs/results
LL=$1   # mock launch log
python3 $R/p1_g22/js_record.py /tmp/claude-1001/g45_rec78.csv > /dev/null 2>&1 & rp=$!
for c in "F0" "F1 --off-deg 1.0 --arms arm_3"; do
  set -- $c; lab=$1; shift
  python3 $H/fake_js.py --r1 1.50 --r2 0.00 "$@" > /dev/null 2>&1 & fp=$!; sleep 2
  python3 $R/p1_g32/run_g32.py --seed 0 --plan $R/p1_g42/g42f_candidates.json --variant G42 --final-retract --dry \
    --launch-log $LL --mon /tmp/claude-1001/g44_mon_samples.csv --out-dir /tmp/claude-1001/g45_dry_$lab \
    --archive $H --prefix g45_dry_${lab}_ > $H/g45_rung32_dry_$lab.log 2>&1
  echo "$lab rc $?: $(grep -E 'K0 (LULUS|TOLAK)|AUTO-STOP|DRY:' $H/g45_rung32_dry_$lab.log | head -3 | tr '\n' ' ' | cut -c1-220)"
  kill $fp; wait $fp 2>/dev/null
done
kill $rp
