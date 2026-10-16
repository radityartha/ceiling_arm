#!/usr/bin/env bash
# DEMO-ZONE-LIVE: kamera (incumbent+G34, imgsz 1280) + localizer rgbd2 + node zona, domain 79, stack MATI. PID -> $L/pids.txt
# (PID sebenarnya dicari dari ps, bukan $!: setsid bisa fork). Matikan: live_down.sh.    L=/tmp/p1dzl live_up.sh
set -u; L=${L:-/tmp/p1dzl}; mkdir -p $L; H=/home/user1/Documents/ceiling_arm/docs/results; export DEMO_DOMAIN=79
setsid nohup $H/p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280 > $L/cam.log 2>&1 < /dev/null &
sleep 5
setsid nohup $H/p1_cam6dof/harness/loc_cam.sh rgbd2 > $L/loc.log 2>&1 < /dev/null &
setsid nohup $H/p1_demo_zone/zone_up.sh -p object_height:=0.07 > $L/zone.log 2>&1 < /dev/null &
sleep 5
ps -eo pid,args | awk '/[r]os2 launch reachability_gng rgbd_perception/ {print "launch", $1}
  /lib\/reachability_gng\/[o]bject_localizer/ {print "loc", $1} /[z]one_correct_node.py/ {print "zone", $1}' | tee $L/pids.txt
grep SigIgn /proc/$(awk '$1=="launch"{print $2}' $L/pids.txt)/status
