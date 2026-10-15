#!/bin/bash
# G63b: plan-check v4 kedua fase di MOCK 77 (fake HW, BUKAN data). Satu job, log berurutan.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1
B=/home/user1/Documents/ceiling_arm/docs/results/p1_g63/g63b; G=$B/..; cd $B
step(){ echo "=== $(date +%T) $*"; }
step mock up; setsid nohup $G/mock/g63_mock_up.sh > mock/v4b_mock_launch.log 2>&1 < /dev/null &
for i in $(seq 1 100); do grep -q "You can start planning now" mock/v4b_mock_launch.log && break; sleep 3; done
grep -q "You can start planning now" mock/v4b_mock_launch.log || { echo "GAGAL: move_group tidak siap"; exit 1; }
sleep 10; timeout 30 ros2 control list_controllers | grep -c active
step monitor; setsid nohup $G/mock/g63_mon_up.sh > mock/v4b_mon.log 2>&1 < /dev/null & sleep 10
timeout 20 ros2 topic info /reach_dwell/target/arm_2 | grep Subscription
step REST; for g in "arm_1 arm_2" "arm_3 arm_4"; do python3 /home/user1/Documents/ceiling_arm/scripts/return_rest.py --arms $g --move 2>&1 | grep -E "selesai|TOLAK|GAGAL"; done
step "g1 -> 1.30"; python3 $G/../p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan g63b_rail_plan.json --variant G63b 1.30 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
ros2 param get /robot_state_publisher robot_description | sed '1s/^String value is: //' > mock/v4_live.urdf
for ph in 2 3; do python3 $G/g63_urdf.py --in mock/v4_live.urdf --mount mount_g63b_arm_$ph.json --out mock/v4_payload_arm_$ph.urdf --check 2>&1 | grep -E "OK|GAGAL"; done
step "pindah rakitan A -> B (gladi S3)"; python3 g63b_attach.py --mount mount_g63b_arm_2.json --boards A | tail -1
python3 g63b_attach.py --mount mount_g63b_arm_2.json --boards A --remove | tail -2
python3 g63b_attach.py --mount mount_g63b_arm_3.json --boards B | tail -1
python3 -u g63b_plancheck.py --board B --poses poses_g63b_v4.json --payload mock/v4_payload_arm_3.urdf --arm arm_3 --out mock/v4_pc_arm_3.json --extra-env $G/s0c_extra_env.npz
step shutdown
for p in $(ps -eo pid,args | awk '/reach_dwell_monitor|my_workcell.launch/ && !/awk/ {print $1}'); do kill -INT $p; done; sleep 15
timeout 20 ros2 node list --no-daemon | wc -l
echo SELESAI
