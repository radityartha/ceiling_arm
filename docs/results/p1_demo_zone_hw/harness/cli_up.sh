#!/usr/bin/env bash
# target_cli atas track TERKOREKSI (remap; rclpy.init() membaca sys.argv). 'g' hanya lewat $L/cli.fifo, ditulis SESUDAH izin
# operator untuk SATU jalan. Catatan: peringatan ">1 localizer" CLI memakai count_publishers (TIDAK ikut remap) -> menghitung
# /detected_objects/tracks asli (0 penerbit di harness ini), bukan /demo_zone/tracks.
source $(dirname $0)/env.sh; trap - INT
[ -p $L/cli.fifo ] || mkfifo $L/cli.fifo
exec ros2 run reachability_gng target_cli --ros-args -p voice_topic:=/voice/transcript -p status_timeout:=900.0 \
  -r /detected_objects/tracks:=/demo_zone/tracks < $L/cli.fifo
