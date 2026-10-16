#!/usr/bin/env bash
# Rekam satu tanda 15 s (mentah + terkoreksi + tolak bersamaan) lalu skor §A2.   L=/tmp/p1dzl live_rec.sh C3 [durasi]
M=$1; T=${2:-15}; L=${L:-/tmp/p1dzl}; H=/home/user1/Documents/ceiling_arm/docs/results
source $H/p1_demo_hw/harness/env.sh; export ROS_DOMAIN_ID=79; TM=$H/p1_arm1_zone/track_median.py
python3 $TM $T $L/${M}_raw.jsonl /loc_rgbd2/tracks > $L/${M}_raw.txt 2>&1 &
python3 $TM $T $L/${M}_cor.jsonl /demo_zone/tracks > $L/${M}_cor.txt 2>&1 &
python3 $TM $T $L/${M}_rej.jsonl /demo_zone/rejected > $L/${M}_rej.txt 2>&1 &
wait; cat $L/${M}_raw.txt; python3 $H/p1_demo_zone/live/live_score.py $M $L
