#!/usr/bin/env bash
# G36 C regression, 16 workers: plans 10 parts, v28 4 parts, controls, hw.
cd /home/user1/Documents/ceiling_arm/docs/results/p1_g36
for i in $(seq 0 9); do python3 -u g36_regress.py --source plans --part $i/10 --out g36_regress_plans_$i.json > g36_regress_plans_$i.log 2>&1 & done
for i in 0 1 2 3; do python3 -u g36_regress.py --source v28 --part $i/4 --out g36_regress_v28_$i.json > g36_regress_v28_$i.log 2>&1 & done
python3 -u g36_regress.py --source controls --out g36_regress_controls.json > g36_regress_controls.log 2>&1 &
python3 -u g36_regress.py --source hw --out g36_regress_hw.json > g36_regress_hw.log 2>&1 &
wait
echo "== SELESAI $(date +%T)" > g36_regress_done.txt
