#!/usr/bin/env bash
# G54 A2.3 + A2.4: remaining smokes, then the negative control. Each run homes + K0 (g54_smoke.sh).
H=/home/user1/Documents/ceiling_arm/docs/results/p1_g54; R=/home/user1/Documents/ceiling_arm/docs/results; cd $H
run() { echo "== $1 mulai $(date +%T)"; bash g54_smoke.sh "$@" > g54_smoke_$1.log 2>&1
        echo "== $1: $(grep -E 'SELESAI: |AUTO-STOP: ' g54_smoke_$1.log | head -1) | $(grep -E 'K0 (LULUS|TOLAK)' g54_smoke_$1.log | tail -1)"; }
run s7r35 7 $R/p1_g42/g42f_candidates.json G42
run s10r180 10 $R/p1_g43/g43_candidates.json R180
run s10r35 10 $R/p1_g42/g42f_candidates.json G42
run s0r90 0 $R/p1_g52/g52_candidates.json R90
run neg_s7r180 7 $R/p1_g43/g43_candidates.json R180 --inject-stop 2:4
echo "== BATCH SELESAI $(date +%T)"
