#!/usr/bin/env bash
# run_pos.sh TID X,Y,Z : swap the fake track, speak, 'g', wait for the runner's STATUS.
SP=$(dirname $0); source $SP/env.sh
for p in $(ps -eo pid,args | awk '/fake_tracks.py/ && !/awk/ && !/fake_up/ {print $1}'); do kill -INT $p; done
sleep 2
setsid nohup $SP/fake_up.sh $1 $2 > $SP/fake$1.log 2>&1 < /dev/null &
sleep 4; echo "" > $SP/cli.fifo; sleep 2
n0=$(grep -c STATUS $SP/runner.log)
curl -sk -X POST https://127.0.0.1:18080/api/transcript -H 'Content-Type: application/json' -d '{"text":"please bring a bottle"}' > /dev/null
sleep 3; echo g > $SP/cli.fifo
echo "$(date +%T) sent g for #$1 ($2)"
timeout 1500 bash -c "until [ \$(grep -c STATUS $SP/runner.log) -gt $n0 ]; do sleep 3; done"
echo "$(date +%T) rc=$? $(grep STATUS $SP/runner.log | tail -n 1)"
