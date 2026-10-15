#!/bin/bash
# G63b: pembangkit pose dudukan NYATA v3 (mount_g63b.json), g1 1.30, tanpa aturan j4 torsi (tak layak, §A8.8)
cd "$(dirname "$0")"
python3 -u g63b_poses.py --j4-min 0 --urdf ../live_urdf_20261011.urdf --mount mount_g63b.json \
  --out poses_g63b_v3.json --rail1 1.30 --samples 8000 > poses_g63b_v3.log 2>&1
echo "rc=$?"; echo SELESAI
