#!/usr/bin/env bash
# Z2: = live/live_rec.sh di domain 0 (env.sh). Rekam 15 s mentah + terkoreksi + tolak -> $L/<tag>/ lalu skor L1-L3 (live_score.py).
#   rec.sh M1 m1_pre [durasi]
M=$1; TAG=${2:?tag}; T=${3:-15}; source $(dirname $0)/env.sh; D=$L/$TAG; mkdir -p $D; TM=$H/p1_arm1_zone/track_median.py
python3 $TM $T $D/${M}_raw.jsonl /loc_rgbd2/tracks > $D/${M}_raw.txt 2>&1 &
python3 $TM $T $D/${M}_cor.jsonl /demo_zone/tracks > $D/${M}_cor.txt 2>&1 &
python3 $TM $T $D/${M}_rej.jsonl /demo_zone/rejected > $D/${M}_rej.txt 2>&1 &
wait; cat $D/${M}_raw.txt; python3 $H/p1_demo_zone/live/live_score.py $M $D
