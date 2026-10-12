#!/usr/bin/env bash
source $(dirname $0)/env.sh; trap - INT
exec ros2 run sayai_voice_sim voice_web_ui --ros-args -p port:=18080
