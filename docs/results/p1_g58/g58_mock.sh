#!/usr/bin/env bash
# G58 A1 on the mock (domain 77, monitor + recorder up, REST + PARK, K0). Fake HW, NOT data.
# Interleaved old/new parallel smokes (run_g54 + retrace home + K0), one job. Resumable: a run whose log already says
# SELESAI is skipped.
source /opt/ros/humble/setup.bash; source /home/user1/Documents/ceiling_arm/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D
SP=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/c032767e-b00c-429e-bddd-e55e3eb2950d/scratchpad
export LAUNCH_LOG=/tmp/g58_mock_launch.log MON=$SP/g58_mon_samples.csv
G43=$D/p1_g43/g43_candidates.json; G42=$D/p1_g42/g42f_candidates.json; G52=$D/p1_g52/g52_candidates.json
SEL=$D/p1_g55/g55_selected.json
RUNS="s3L:3:$G43:R180 s3B:3:$SEL:R90sel s8B:8:$SEL:R90sel s8L:8:$G42:G42 s18L:18:$G43:R180 s18B:18:$SEL:R90sel
s12:12:$G52:R90 s7L:7:$G43:R180 s7B:7:$SEL:R90sel s10B:10:$SEL:R90sel s10L:10:$G42:G42"
for R in $RUNS; do
  IFS=: read TAG S P V <<< "$R"
  L=p1_g58/g58_smoke_$TAG.log
  if [ -f $L ] && grep -q "== SELESAI" $L; then echo "== $TAG sudah selesai"; continue; fi
  echo "== $TAG (s$S $V) mulai $(date +%T)"
  bash p1_g54/g54_smoke.sh g58_$TAG $S $P $V > $L 2>&1
  echo "== $TAG: $(grep -E 'SELESAI: |AUTO-STOP: ' $L | head -1 | cut -c1-160) | $(grep -E 'K0 (LULUS|TOLAK)' $L | tail -1 | cut -c1-80) $(date +%T)"
  grep -q "K0 LULUS" <(grep -E 'K0 (LULUS|TOLAK)' $L | tail -1) || { echo "== BERHENTI: K0 sesudah $TAG tidak LULUS"; exit 3; }
done
echo "== SEMUA SELESAI $(date +%T)"
