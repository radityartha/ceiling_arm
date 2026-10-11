#!/usr/bin/env bash
# G59 A1 on the mock (domain 77; mock + monitor up, REST, g1 PARK). Fake perception + fake voice, fake HW. NOT data.
# Order: J1 (+ r1) J2 J3 J5 J6 J7 J8 (r2) J9 (r3), then runner restarted --plan-only for J4.
H=$(cd $(dirname $0) && pwd); source $H/env.sh
J=/home/user1/Documents/ceiling_arm/docs/results/p1_g22/js_record.py
up() { setsid nohup "$@" < /dev/null & }
killp() { for p in $(ps -eo pid,args | awk -v k="$1" '$0 ~ k && !/awk/ {print $1}'); do kill -INT $p; done; }
rm -f $H/cli.fifo; mkfifo $H/cli.fifo
setsid nohup bash -c "exec 3>$H/cli.fifo; sleep 86400" > /dev/null 2>&1 < /dev/null &
up $H/vwu_up.sh > $H/vwu.log 2>&1
up $H/cli_up.sh > $H/cli.log 2>&1
export EXTRA=; up $H/runner_up.sh > $H/runner.log 2>&1
timeout 60 bash -c "until grep -q 'demo_runner up' $H/runner.log && curl -sk https://127.0.0.1:18080/ > /dev/null; do sleep 2; done" \
  || { echo "BERHENTI: runner/voice tidak naik"; exit 3; }
echo "== naik $(date +%T)"
# J1 + r1: second 'g' 15 s into the run
# r1a: a second 'g' typed into target_cli 15 s into J1 -> must be DISCARDED by target_cli (G59 B fix), no run 2
( sleep 25; echo g > $H/cli.fifo; echo "$(date +%T) r1a: g kedua diketik" ) & R1=$!
$H/run_pos.sh 201 1.75,0.55,0.97; wait $R1
sleep 20; echo "r1a: run sesudah J1 = $(grep -c 'run [0-9]*: #' $H/runner.log) (harus 1); cli: $(grep -c DISCARDED $H/cli.log) DISCARDED"
# r1b: an execute published straight to the runner 15 s into J2 -> runner busy guard "(r1)"
( sleep 25; ros2 topic pub --once /reach_fusion/execute std_msgs/msg/Empty "{}" > /dev/null; echo "$(date +%T) r1b: execute langsung" ) & R1=$!
$H/run_pos.sh 202 1.75,0.55,0.97; wait $R1
echo "r1b: runner DITOLAK (r1) baris: $(grep -c '(r1)' $H/runner.log)"
$H/run_pos.sh 203 1.75,0.55,0.97
$H/run_pos.sh 205 2.15,-0.15,0.85
$H/run_pos.sh 206 1.20,0.85,1.25
$H/run_pos.sh 207 1.25,0.55,0.97
# J8 r2: a second subscriber on /reach_fusion/execute
up ros2 topic echo /reach_fusion/execute std_msgs/msg/Empty > $H/r2_echo.log 2>&1
sleep 5; ros2 topic info /reach_fusion/execute | grep -i "subscription count"
$H/run_pos.sh 208 1.75,0.55,0.97
killp "topic echo /reach_fusion/execute"; sleep 3
# J9 r3: the track disappears between selection and 'g'
killp "fake_tracks.py"; sleep 2
n0=$(grep -c STATUS $H/runner.log)
up python3 $H/../../p1_demo/fake_tracks.py --tid 209 --label bottle --xyz 1.75,0.55,0.97 > $H/fake209.log 2>&1
sleep 4; echo "" > $H/cli.fifo; sleep 2
curl -sk -X POST https://127.0.0.1:18080/api/transcript -H 'Content-Type: application/json' -d '{"text":"please bring a bottle"}' > /dev/null
sleep 4; killp "fake_tracks.py"; sleep 2
ros2 topic pub --once --qos-durability transient_local /detected_objects/tracks std_msgs/msg/String "{data: '[]'}" > /dev/null
sleep 3; echo g > $H/cli.fifo
timeout 120 bash -c "until [ \$(grep -c STATUS $H/runner.log) -gt $n0 ]; do sleep 2; done"
echo "$(date +%T) r3: $(grep STATUS $H/runner.log | tail -n 1)"
# J4 plan-only: runner restarted
killp "demo_runner.py"; sleep 4
mv $H/runner.log $H/runner_exec.log
export EXTRA=--plan-only; up $H/runner_up.sh > $H/runner.log 2>&1
timeout 60 bash -c "until grep -q 'demo_runner up' $H/runner.log; do sleep 2; done"
python3 $J $H/j4_js_before.csv > /dev/null 2>&1 & JP=$!; sleep 3; kill -INT $JP
$H/run_pos.sh 204 1.75,0.55,0.97
python3 $J $H/j4_js_after.csv > /dev/null 2>&1 & JP=$!; sleep 3; kill -INT $JP
killp "demo_runner.py"; killp "target_cli"; killp "voice_web_ui"; killp "fake_tracks.py"; killp "sleep 86400"
echo "== SELESAI $(date +%T)"
