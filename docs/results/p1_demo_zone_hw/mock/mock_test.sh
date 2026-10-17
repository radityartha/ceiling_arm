#!/usr/bin/env bash
# DEMO-ZONE-HW tugas 0c (MOCK 77, fake HW, BUKAN data): rekaman mentah live 10-17 -> zone node -> target_cli (remap) -> runner
# --plan-only --tracks-topic /demo_zone/tracks. Jalan 1: C3, ARMS default, '#0' via fifo. Jalan 2: M1, ARMS=arm_1, POST teks.
# Lengan REST, g1 1.40 (rencana rel G63), g2 0 = keadaan sel HW. Penyaring tidak diubah.
export DEMO_DOMAIN=77 L=/tmp/p1dzh_mock; mkdir -p $L
HZ=/home/user1/Documents/ceiling_arm/docs/results; HH=$HZ/p1_demo_zone_hw/harness; M=$HZ/p1_demo_zone_hw/mock; LV=$HZ/p1_demo_zone/live/hw_20261017
source $HH/env.sh; cd $M
step(){ echo "=== $(date +%T) $*"; }
pid(){ ps -eo pid,args | awk -v p="$1" '$0 ~ p && !/awk/ {print $1; exit}'; }
step mock up; setsid nohup $HZ/p1_cam6dof/harness/mock_up.sh > $L/mock_launch.log 2>&1 < /dev/null &
for i in $(seq 1 100); do grep -q "You can start planning now" $L/mock_launch.log && break; sleep 3; done
grep -q "You can start planning now" $L/mock_launch.log || { echo "GAGAL: move_group tidak siap"; exit 1; }
sleep 10; echo "controllers active: $(timeout 30 ros2 control list_controllers | grep -c active)"
step REST; for g in "arm_1 arm_2" "arm_3 arm_4"; do python3 /home/user1/Documents/ceiling_arm/scripts/return_rest.py --arms $g --move 2>&1 | grep -E "selesai|TOLAK|GAGAL"; done
step "g1 -> 1.40"; python3 $HZ/p1_g32/pose_to_g.py --gantry 1 --seed 63 --plan $HZ/p1_g63/g63_rail_plan.json --variant G63 1.40 0 --move 2>&1 | grep -E "^akhir|TOLAK|REFUS|GAGAL"
step monitor; setsid nohup $HH/mon_up.sh > $L/mon.log 2>&1 < /dev/null &
step "zone + replay C3"; setsid nohup $HZ/p1_demo_zone/zone_up.sh -p object_height:=0.07 > $L/zone.log 2>&1 < /dev/null &
setsid nohup python3 $HH/replay.py $LV/c3/C3_raw.jsonl > $L/replay.log 2>&1 < /dev/null &
step view; $HH/view_up.sh
setsid nohup $HZ/p1_demo_hw/harness/vwu_up.sh > $L/vwu.log 2>&1 < /dev/null &
step "cli + runner (plan-only)"; setsid nohup $HH/fifo_hold.sh > /dev/null 2>&1 < /dev/null &
sleep 1; setsid nohup $HH/cli_up.sh > $L/cli.log 2>&1 < /dev/null &
EXTRA=--plan-only OUT=$M/run setsid nohup $HH/runner_up.sh > $L/runner1.log 2>&1 < /dev/null &
sleep 15; timeout 10 ros2 topic echo --once /demo_zone/tracks | head -3
waitrun(){ for i in $(seq 1 120); do [ -f $M/run/run$1.json ] && return 0; sleep 5; done; echo "GAGAL: run$1.json tidak muncul"; }
step "jalan 1: #0 + g"; echo '#0' > $L/cli.fifo; sleep 3; echo g > $L/cli.fifo; waitrun 01
step "runner ARMS=arm_1, replay M1"; kill -INT $(pid 'demo_runner.py') $(pid 'replay.py'); sleep 3
setsid nohup python3 $HH/replay.py $LV/m1/M1_raw.jsonl > $L/replay2.log 2>&1 < /dev/null &
ARMS=arm_1 EXTRA=--plan-only OUT=$M/run setsid nohup $HH/runner_up.sh > $L/runner2.log 2>&1 < /dev/null &
sleep 10
step "jalan 2: POST teks + g"; curl -sk -X POST https://127.0.0.1:18080/api/transcript -H 'Content-Type: application/json' -d '{"text":"please bring a bottle"}'; echo
sleep 4; echo g > $L/cli.fifo; waitrun 02
DISPLAY=:1 scrot $L/view_run2.png
step selesai
