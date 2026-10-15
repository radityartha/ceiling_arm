#!/bin/bash
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77 PYTHONUNBUFFERED=1
cd "$(dirname "$0")/.."
python3 -u g63b_plancheck.py --board A --poses poses_g63b_v3.json --payload mock/payload_arm_2.urdf --arm arm_2 \
  --out mock/pc_arm_2.json --extra-env ../s0c_extra_env.npz
echo "rc=$?"; echo SELESAI
