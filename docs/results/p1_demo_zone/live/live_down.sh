#!/usr/bin/env bash
# SIGINT per PID dari $L/pids.txt (node dulu, lalu launch), tunggu <= 30 s, cek sisa + domain 79 kosong. Tanpa pkill -f.
L=${L:-/tmp/p1dzl}
for k in zone loc; do p=$(awk -v k=$k '$1==k{print $2}' $L/pids.txt); [ -n "$p" ] && kill -INT $p; done; sleep 2
p=$(awk '$1=="launch"{print $2}' $L/pids.txt); kill -INT $p
for i in $(seq 30); do ps -p $p > /dev/null || break; sleep 1; done; echo "launch $p hilang ${i}s"
ps -eo pid,args | awk '/[r]ealsense2_camera_node|[s]eg_router|[s]tatic_transform_publisher|[o]bject_localizer|[z]one_correct_node/' 
source /opt/ros/humble/setup.bash; echo "node domain 79: $(ROS_DOMAIN_ID=79 timeout 15 ros2 node list --no-daemon | wc -l)"
