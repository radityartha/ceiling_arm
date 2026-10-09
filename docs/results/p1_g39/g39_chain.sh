#!/usr/bin/env bash
# G39 -> G38 chain (2026-10-09, operator offline): wait for the table build, validate it, then re-run G38 with the
# EXACT oracle (g38_make.py unchanged; caches reused). Nothing here moves hardware.
cd /home/user1/Documents/ceiling_arm/docs/results
BUILD=$1
while [ -d /proc/$BUILD ]; do sleep 60; done
echo "== build selesai $(date +%T)"
cd p1_g39 && python3 -u g39_table.py validate 2>&1 | grep --line-buffered -v Warning > g39_validate.log
echo "== validate rc ${PIPESTATUS[0]} $(date +%T)"
cd ../p1_g38 && python3 -u g38_make.py run 2>&1 | grep --line-buffered -v "Warning\|buildGeom\|self.geom" > g38_make.log
echo "== g38 rc ${PIPESTATUS[0]} $(date +%T)"
