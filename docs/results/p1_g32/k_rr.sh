#!/bin/bash
# K-RR0/p/m/n (docs/p1_g32_rot_exec.md A6). DRY only, synthetic /joint_states, domain 77, no stack.
export ROS_DOMAIN_ID=77
S=/home/user1/Documents/ceiling_arm/scripts
H=$(dirname $(readlink -f $0))
run() {  # label, fake_js args...
  lab=$1; shift
  python3 $H/fake_js.py "$@" 2>/dev/null & F=$!; sleep 2
  echo "=== $lab   fake_js $*"
  for v in pre post; do
    if [ $v = pre ]; then f=$H/return_rest_pre_g32.py; else f=$S/return_rest.py; fi
    (cd $S && PYTHONPATH=$S:$PYTHONPATH python3 $f --arms arm_3 arm_4 2>&1 | grep -v -i warn | sed "s/^/  [$v] /"; echo "  [$v] rc=${PIPESTATUS[0]}")
  done
  kill -INT $F; wait $F 2>/dev/null
}
run K-RR0 --off-deg 1.0 --arms arm_3 arm_4
run K-RRp --off-deg 1.0 --arms arm_3 arm_4 --r1 0.9 --r2 0.6 --rot1 -10 --rot2 -10
run K-RRm --off-deg 1.0 --arms arm_3 arm_4 --drop t1_rotation_joint
python3 $H/fake_js.py --off-deg 1.0 --arms arm_3 arm_4 2>/dev/null & F=$!; sleep 2
echo "=== K-RRn   model antar-gantry buta t1_rotation_joint"
(cd $S && python3 $H/k_rrn_wrap.py --arms arm_3 arm_4 2>&1 | grep -v -i warn | sed 's/^/  [post] /'; echo "  [post] rc=${PIPESTATUS[0]}")
kill -INT $F; wait $F 2>/dev/null
