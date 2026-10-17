#!/usr/bin/env bash
# TAMPILAN demo di DISPLAY :1 (noVNC): rqt_image_view /rgbd2/seg/debug_image (deteksi YOLOE) + RViz demo (robot, kotak zona,
# track mentah vs terkoreksi, p_cmd, peta env, gambar deteksi) + penerbit marker. Hanya baca/tampil, NOL gerak.
# PID -> $L/pids_view.txt; matikan: kill -INT per PID dari berkas itu.
source $(dirname $0)/env.sh; D=$(dirname $(readlink -f $0))
setsid nohup python3 $D/zone_markers.py > $L/markers.log 2>&1 < /dev/null &
setsid nohup ros2 run rqt_image_view rqt_image_view /rgbd2/seg/debug_image > $L/rqt.log 2>&1 < /dev/null &
setsid nohup rviz2 -d $D/demo_zone.rviz > $L/rviz_demo.log 2>&1 < /dev/null &
sleep 4
ps -eo pid,args | awk '/[z]one_markers.py/ {print "markers", $1} /lib\/rqt_image_view\/[r]qt_image_view/ {print "rqt", $1}
  /[r]viz2 -d .*demo_zone.rviz/ {print "rviz", $1}' | tee $L/pids_view.txt
