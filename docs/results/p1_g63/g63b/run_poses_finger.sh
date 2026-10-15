#!/bin/bash
# G63b CADANGAN JARI: pose satu muka per lengan (A arm_2, B arm_3), g1 1.40 (G63 A8.13), R0 <= 40 deg
cd "$(dirname "$0")"
python3 -u g63b_poses.py --single-face --j4-min 0 --urdf ../live_urdf_20261011.urdf --mount mount_finger.json \
  --out poses_finger_v1.json --rail1 1.40 --samples 8000 > poses_finger_v1.log 2>&1
echo "rc=$?"; echo SELESAI
