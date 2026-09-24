#!/bin/bash
# G29: after K, true-hull map rebuild -> analyse -> (d) sample -> report -> tip speed
cd "$(dirname "$0")"
while pgrep -f 'g29_oracle_rot.py k' >/dev/null; do sleep 10; done
echo "chain start $(date +%T)"
python3 g29_map.py build > g29_map_build.log 2>&1 && echo "map built $(date +%T)"
python3 g29_map.py analyse > g29_map_analyse.log 2>&1 && echo "map analysed $(date +%T)"
python3 g29_oracle_rot.py sample > g29_oracle_sample.log 2>&1 && echo "sample done $(date +%T)"
python3 g29_oracle_rot.py report > g29_oracle_report.log 2>&1 && echo "report done $(date +%T)"
python3 g29_tipspeed.py > g29_tipspeed.log 2>&1 && echo "tipspeed done $(date +%T)"
echo "chain end $(date +%T)"
