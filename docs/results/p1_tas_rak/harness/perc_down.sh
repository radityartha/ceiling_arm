#!/usr/bin/env bash
# = p1_demo_zone_hw/harness/perc_down.sh untuk $L/pids_perc.txt (rack + loc, lalu anak launch per PID, lalu launch). Tanpa pkill -f.
source $(dirname $0)/env.sh; P=$L/pids_perc.txt
for k in rack loc; do p=$(awk -v k=$k '$1==k{print $2}' $P); [ -n "$p" ] && kill -INT $p; done; sleep 2
LP=$(awk '$1=="launch"{print $2}' $P); C=$(ps -o pid= --ppid $LP | tr -s ' \n' ' ')
echo "launch $LP anak: $C"; [ -n "${C// }" ] && kill -INT $C
for i in $(seq 30); do n=0; for p in $LP $C; do ps -p $p > /dev/null && n=$((n+1)); done; [ $n -le 1 ] && break; sleep 1; done
ps -p $LP > /dev/null && kill -INT $LP; sleep 2; ps -p $LP > /dev/null && kill -TERM $LP
ps -eo pid,args | awk '/[r]ealsense2_camera_node|[s]eg_router|[o]bject_localizer|[r]ack_box_node|[c]onvert_metric_node|topic_tools\/[r]elay/'
