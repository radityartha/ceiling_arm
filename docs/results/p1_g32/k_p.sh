#!/bin/bash
# K-P (docs/p1_g32_rot_exec.md A6): pose_to_g DRY on synthetic /joint_states, domain 77, no stack.
export ROS_DOMAIN_ID=77
H=$(dirname $(readlink -f $0))
case_() {  # label, "fake args", pose_to_g args...
  lab=$1; fa=$2; shift 2
  python3 $H/fake_js.py $fa 2>/dev/null & F=$!; sleep 2
  echo "=== $lab   fake_js $fa   pose_to_g $*"
  python3 $H/pose_to_g.py --seed 36 "$@" 2>&1 | grep -v -i "warn\|buildGeom" | sed 's/^/  /'
  echo "  rc=${PIPESTATUS[0]}"
  kill -INT $F; wait $F 2>/dev/null
}
case_ a  ""                                   --gantry 2 0.60 -10
case_ b  "--r1 0.9"                           --gantry 1 1.35 0
case_ c  "--off-deg 1.0 --arms arm_3"         --gantry 2 0.60 -10
case_ d  ""                                   --gantry 2 0.50 0
case_ e1 ""                                   --gantry 2 0.60 15
case_ e2 "--rot1 12"                          --gantry 2 0.60 -10
case_ f  "--drop t2_rotation_joint"           --gantry 2 0.60 -10
