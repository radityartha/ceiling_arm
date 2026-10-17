#!/usr/bin/env bash
source $(dirname $0)/env.sh
exec python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('ros2',['ros2','run','reachability_gng','reach_dwell_monitor','--ros-args','-p','csv_log:=$L/p1tr_mon'])"
