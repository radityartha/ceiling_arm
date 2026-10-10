#!/usr/bin/env bash
# G45 A2 fake_js cases F0-F6 on ROS_DOMAIN_ID=78 (no stack there). Observer only.
export ROS_DOMAIN_ID=78
H=$(cd $(dirname $0) && pwd); K=/home/user1/Documents/ceiling_arm/scripts/k0_check.py
run() {  # label expect fake_js-args...
  lab=$1; want=$2; shift 2
  python3 $H/fake_js.py --r1 1.50 --r2 0.00 "$@" > /dev/null 2>&1 &
  fp=$!; sleep 2
  python3 $K --rail1 1.50 --rail2 0.00 --seconds 1.0 --json $H/g45_$lab.json > $H/g45_$lab.log 2>&1; rc=$?
  kill $fp; wait $fp 2>/dev/null
  got=$([ $rc -eq 0 ] && echo LULUS || echo TOLAK)
  echo "$lab harap $want dapat $got (rc $rc) -- $(tail -1 $H/g45_$lab.log | cut -c1-160)"
}
run F0 LULUS
run F1 TOLAK --off-deg 1.0 --arms arm_3
run F2 TOLAK --r1 1.503
run F3 TOLAK --rot2 1.0
run F4 TOLAK --finger 0.05
run F5 TOLAK --drop t2_a2_joint_4
run F6 LULUS --off-deg 0.4 --arms arm_1
