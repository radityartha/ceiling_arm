#!/usr/bin/env bash
# One-terminal wrapper for the voice/web control demo: table controller +
# MoveIt/RViz (all 4 real arms) + the voice/web pipeline. Ctrl+C tears
# everything down.
#
# Equivalent to running these 3 commands in 3 separate terminals:
#   1) ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false
#   2) ./scripts/start_single_rviz.sh
#   3) ros2 launch sayai_voice_sim workcell_voice.launch.py
#
# Usage:  ./scripts/start_voice_demo.sh
set -e

WS="$HOME/Documents/ceiling_arm/ros2_ws"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

LOG_DIR="$(mktemp -d /tmp/voice_demo_logs.XXXXXX)"
echo "Logs: $LOG_DIR"

PIDS=()
cleanup() {
    echo
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    pkill -f single_rviz_workcell 2>/dev/null || true
    pkill -f "rviz2 -d" 2>/dev/null || true
    pkill -f workcell_voice.launch 2>/dev/null || true
    pkill -f dual_table_controller 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[1/3] Starting table controller..."
ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false \
    > "$LOG_DIR/table_controller.log" 2>&1 &
PIDS+=($!)

echo "Waiting for /move_dual_table service..."
for i in $(seq 1 20); do
    if ros2 service list 2>/dev/null | grep -q "^/move_dual_table$"; then
        echo "Table controller is up."
        break
    fi
    sleep 1
done

echo "[2/3] Starting MoveIt + RViz (all 4 arms, real hardware)..."
"$(dirname "$0")/start_single_rviz.sh" > "$LOG_DIR/moveit_rviz.log" 2>&1 &
PIDS+=($!)

echo "Waiting for /move_group..."
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "^/move_group$"; then
        echo "move_group is up."
        break
    fi
    sleep 1
done
sleep 3

echo "[3/3] Starting voice + web control pipeline..."
ros2 launch sayai_voice_sim workcell_voice.launch.py \
    > "$LOG_DIR/voice_web.log" 2>&1 &
PIDS+=($!)

sleep 2
# Prefer a LAN IP reachable from a phone/browser; 192.168.2.x is the arm-only
# subnet and won't be reachable from anything but this PC.
LAN_IP="$(hostname -I | tr ' ' '\n' | grep -v '^192\.168\.2\.' | grep -E '^[0-9]+\.' | head -1)"
echo
echo "All 3 subsystems running."
echo "Web UI:  https://${LAN_IP:-<this-PC-LAN-IP>}:8080"
echo "Logs:    tail -f $LOG_DIR/*.log"
echo "Ctrl+C to stop everything."
echo

tail -f "$LOG_DIR"/*.log
