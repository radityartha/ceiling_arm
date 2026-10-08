#!/usr/bin/env bash
# G34 A1: one calibration leg = pose_to_g DRY -> --move (rc 0 both) -> optional capture.
# usage: leg.sh <gantry> <lin> <name> [capture_out]
set -o pipefail
D=/home/user1/Documents/ceiling_arm/docs/results/p1_g34
P="python3 -u $D/../p1_g32/pose_to_g.py --gantry $1 --seed 34 --plan $D/g34_calib_plan.json --variant CAL $2 0"
$P 2>&1 | grep -v -i "warn\|buildGeom" > $D/g34hw_$3_dry.log; rc=$?
tail -3 $D/g34hw_$3_dry.log | cut -c1-200
grep -q '"dry": true' $D/g34hw_$3_dry.log || { echo "DRY GAGAL"; exit 1; }
$P --move 2>&1 | grep -v -i "warn\|buildGeom" > $D/g34hw_$3.log
tail -2 $D/g34hw_$3.log | cut -c1-400
python3 -c "import json,sys; r=json.loads(open('$D/g34hw_$3.log').read().strip().splitlines()[-1]); sys.exit(0 if r.get('jtc_error_code')=='0' and abs(r['err_mm'])<2 and r['arm_drift_deg']<0.5 else 3)" || { echo "GERAK GAGAL / di luar batas"; exit 3; }
if [ -n "$4" ]; then
  python3 -u $D/../p1_g33/capture_cloud.py --joints --seconds 20 --out $D/$4.npz > $D/$4.log 2>&1 || { cat $D/$4.log; echo "TANGKAP GAGAL"; exit 2; }
  tail -4 $D/$4.log
fi
echo LEG_OK
