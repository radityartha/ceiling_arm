#!/bin/bash
# G63b pose v4: dudukan NYATA (mount_g63b.json), g1 1.30, R0 <= 40 deg (operator 2026-10-15), aturan j4 torsi mati
cd "$(dirname "$0")"
python3 -u g63b_poses.py --j4-min 0 --urdf ../live_urdf_20261011.urdf --mount mount_g63b.json \
  --out poses_g63b_v4.json --rail1 1.30 --samples 8000 > poses_g63b_v4.log 2>&1
echo "rc=$?"; echo SELESAI
