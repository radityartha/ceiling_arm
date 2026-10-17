#!/usr/bin/env bash
# Rekam 15 s fusi + rack_box (diterima/tolak) -> $L/<tag>/ ; cetak median per track.   rec.sh <tag> [durasi]
TAG=${1:?tag}; T=${2:-15}; source $(dirname $0)/env.sh; D=$L/$TAG; mkdir -p $D; TM=$H/p1_arm1_zone/track_median.py
python3 $TM $T $D/fused.jsonl /detected_objects/tracks > $D/fused.txt 2>&1 &
python3 $TM $T $D/rack.jsonl /rack_box/tracks > $D/rack.txt 2>&1 &
python3 $TM $T $D/rej.jsonl /rack_box/rejected > $D/rej.txt 2>&1 &
wait; echo "--- fusi"; cat $D/fused.txt; echo "--- rack_box diterima"; cat $D/rack.txt
