#!/bin/bash
# G63b CADANGAN JARI di MOCK 77 (fake HW, BUKAN data): plan-check G63 kedua lengan + uji tangkap R0 (papan polos, --arm finger).
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1
B=/home/user1/Documents/ceiling_arm/docs/results/p1_g63/g63b; G=$B/..; cd $B
step(){ echo "=== $(date +%T) $*"; }
step mock up; setsid nohup $G/mock/g63_mock_up.sh > mock/fj_mock_launch.log 2>&1 < /dev/null &
for i in $(seq 1 100); do grep -q "You can start planning now" mock/fj_mock_launch.log && break; sleep 3; done
grep -q "You can start planning now" mock/fj_mock_launch.log || { echo "GAGAL: move_group tidak siap"; exit 1; }
sleep 10; timeout 30 ros2 control list_controllers | grep -c active
step monitor; setsid nohup $G/mock/g63_mon_up.sh > mock/fj_mon.log 2>&1 < /dev/null & sleep 10
timeout 20 ros2 topic info /reach_dwell/target/arm_2 | grep Subscription
step REST; for g in "arm_1 arm_2" "arm_3 arm_4"; do python3 /home/user1/Documents/ceiling_arm/scripts/return_rest.py --arms $g --move 2>&1 | grep -E "selesai|TOLAK|GAGAL"; done
step "g1 -> 1.40 (rencana rel G63)"; python3 $G/../p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan $G/g63_rail_plan.json --variant G63 1.40 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
ros2 param get /robot_state_publisher robot_description | sed '1s/^String value is: //' > mock/fj_live.urdf
python3 $G/g63_urdf.py --in mock/fj_live.urdf --mount mount_finger.json --out mock/fj_payload.urdf --check 2>&1 | grep -E "OK|GAGAL"
python3 g63b_attach.py --mount mount_finger.json --boards A B | tail -1
for arm in arm_2 arm_3; do step "plan-check $arm"
  python3 -u $G/g63_plancheck.py --poses poses_finger_v1.json --payload mock/fj_payload.urdf --arm $arm --out mock/fj_pc_$arm.json --extra-env $G/s0c_extra_env.npz | tail -1
done
step "uji tangkap R0 (fakecam finger)"
setsid nohup python3 -u mock/g63b_fakecam.py --arm finger --mount mount_finger.json --truth-out mock/fj_truth.json > mock/fj_fakecam.log 2>&1 < /dev/null & sleep 8
S=mock/fj_session; mkdir -p $S
for pr in "arm_2 A" "arm_3 B"; do set -- $pr
  python3 $G/g63_move.py --session $S --poses poses_finger_v1.json --payload mock/fj_payload.urdf --extra-env $G/s0c_extra_env.npz --arm $1 --pose $2-R0 | grep -E "rc |MOVED"
  python3 g63b_capture.py capture --arm finger --session $S --pose-id $2-R0 --board $2 --role R0 --stage P0 --mount mount_finger.json | grep -E "papan|diam|GAGAL"
  python3 $G/g63_move.py --session $S --poses poses_finger_v1.json --payload mock/fj_payload.urdf --extra-env $G/s0c_extra_env.npz --arm $1 --pose REST | grep -E "rc |MOVED"
done
python3 g63b_capture.py merge --arm finger --session $S --square-mm A=42.0 B=42.0 --out mock/fj_ds.json --poses poses_finger_v1.json | tail -1
step shutdown
for p in $(ps -eo pid,args | awk '/g63b_fakecam|reach_dwell_monitor|my_workcell.launch/ && !/awk/ {print $1}'); do kill -INT $p; done; sleep 15
timeout 20 ros2 node list --no-daemon | wc -l
echo SELESAI
