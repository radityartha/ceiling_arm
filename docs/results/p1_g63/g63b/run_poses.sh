#!/bin/bash
# G63b: pembangkit pose dudukan TEBAKAN, dua varian aturan j4 (0 = mati, 1.5 N.m)
cd "$(dirname "$0")"
for j in 0 1.5; do
  python3 -u g63b_poses.py --j4-min $j --urdf ../live_urdf_20261011.urdf --mount mount_g63b_guess.json \
    --out poses_g63b_guess_v2_j4_$j.json --rail1 1.30 --samples 8000 > poses_g63b_guess_v2_j4_$j.log 2>&1
  echo "j4=$j rc=$?"
done
echo SELESAI
