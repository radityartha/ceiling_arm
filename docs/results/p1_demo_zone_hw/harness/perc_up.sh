#!/usr/bin/env bash
# Kamera (incumbent + G34, imgsz 1280) + localizer rgbd2 saja + node zona (botol 7 cm) di domain 0 (= live_up.sh, domain beda).
# PID dicari dari ps -> $L/pids_perc.txt. Matikan: perc_down.sh.
set -u; source $(dirname $0)/env.sh
setsid nohup $H/p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280 > $L/cam.log 2>&1 < /dev/null &
sleep 5
setsid nohup $H/p1_cam6dof/harness/loc_cam.sh rgbd2 > $L/loc.log 2>&1 < /dev/null &
setsid nohup $H/p1_demo_zone/zone_up.sh -p object_height:=0.07 > $L/zone.log 2>&1 < /dev/null &
sleep 5
ps -eo pid,args | awk '/[r]os2 launch reachability_gng rgbd_perception/ {print "launch", $1}
  /lib\/reachability_gng\/[o]bject_localizer/ {print "loc", $1} /[z]one_correct_node.py/ {print "zone", $1}' | tee $L/pids_perc.txt
grep SigIgn /proc/$(awk '$1=="launch"{print $2}' $L/pids_perc.txt)/status
