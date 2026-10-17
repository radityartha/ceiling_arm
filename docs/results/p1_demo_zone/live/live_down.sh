#!/usr/bin/env bash
# SIGINT per PID dari $L/pids.txt: node zona + localizer, lalu ANAK launch (ps --ppid), lalu launch. Tanpa pkill -f.
# 2026-10-17 10:08: launch bisa mewarisi SIGINT diabaikan (SigIgn 0x...7) dan SIGTERM ke launch mematikannya SEKETIKA tanpa
# menutup anak (11 yatim) -> anak harus disinyal sendiri.
L=${L:-/tmp/p1dzl}
for k in zone loc; do p=$(awk -v k=$k '$1==k{print $2}' $L/pids.txt); [ -n "$p" ] && kill -INT $p; done; sleep 2
LP=$(awk '$1=="launch"{print $2}' $L/pids.txt); C=$(ps -o pid= --ppid $LP | tr -s ' \n' ' ')
echo "launch $LP anak: $C"; [ -n "${C// }" ] && kill -INT $C
for i in $(seq 30); do n=0; for p in $LP $C; do ps -p $p > /dev/null && n=$((n+1)); done; [ $n -le 1 ] && break; sleep 1; done
ps -p $LP > /dev/null && kill -INT $LP; sleep 2; ps -p $LP > /dev/null && kill -TERM $LP
ps -eo pid,args | awk '/[r]ealsense2_camera_node|[s]eg_router|[s]tatic_transform_publisher|[o]bject_localizer|[z]one_correct_node|[c]onvert_metric_node|topic_tools\/[r]elay/'
source /opt/ros/humble/setup.bash; echo "node domain 79: $(ROS_DOMAIN_ID=79 timeout 15 ros2 node list --no-daemon | wc -l)"
