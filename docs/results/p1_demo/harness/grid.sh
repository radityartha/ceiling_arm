#!/usr/bin/env bash
source $(dirname $0)/env.sh; cd /home/user1/Documents/ceiling_arm/scripts
for dx in -0.05 0 0.05; do for dy in -0.05 0 0.05; do for dz in -0.03 0.03; do
 x=$(python3 -c "print(round(1.75+$dx,3))"); y=$(python3 -c "print(round(0.55+$dy,3))"); z=$(python3 -c "print(round(1.22+$dz,3))")
 v=$(timeout 300 python3 -u reach_dwell_probe.py --arms arm_1 --target=$x,$y,$z --approach 0 --trials 1 --tau-max 12.0 --plan-check --monitor-csv /tmp/demo_off_mon_samples.csv 2>&1 | grep -oE "rencana: \S+|terketat \S+ [0-9]+ %|antar-lengan minimum[^(]*|lingkungan minimum[^(]*" | tr '\n' ' ')
 echo "$x,$y,$z $v"
done; done; done
echo DONE
