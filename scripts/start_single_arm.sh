#!/usr/bin/env bash
# One-terminal bringup: arm + tables + move_group, then RViz with MoveIt.
#
# Why a script instead of launch_rviz:=true — when RViz is spawned by a
# TimerAction inside single_arm_tables.launch.py, the included kortex/move_group
# sub-launches starve the launch event loop and the timer never fires, so no
# window appears. This script sidesteps that: it runs the launch (no rviz) in
# the background, waits for /move_group, then execs the SYSTEM rviz2 (env
# stripped so the broken rviz2_ws/moveit2_ws overlays can't shadow it) in the
# foreground. Ctrl+C tears everything down.
#
# Usage:  ./scripts/start_single_arm.sh [robot_ip]
#   robot_ip defaults to 192.168.2.10 (arm 4). Use 192.168.2.11 for arm 3.
set -e

ROBOT_IP="${1:-192.168.2.10}"
WS="$HOME/Documents/ceiling_arm/ros2_ws"

# strip the broken custom overlays from this shell's environment
strip() { echo "$1" | tr ':' '\n' | grep -vE 'rviz2_ws|moveit2_ws' | paste -sd: -; }
export PATH="$(strip "$PATH")"
export AMENT_PREFIX_PATH="$(strip "${AMENT_PREFIX_PATH:-}")"
export LD_LIBRARY_PATH="$(strip "${LD_LIBRARY_PATH:-}")"
export PYTHONPATH="$(strip "${PYTHONPATH:-}")"
export CMAKE_PREFIX_PATH="$(strip "${CMAKE_PREFIX_PATH:-}")"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "Using rviz2: $(command -v rviz2)"          # expect /opt/ros/humble/bin/rviz2
echo "Robot IP:    $ROBOT_IP"

# --- start arm + tables + move_group (NO rviz) in the background ---
ros2 launch workcell_moveit_config single_arm_tables.launch.py \
    robot_ip:="$ROBOT_IP" \
    launch_rviz:=false &
LAUNCH_PID=$!

# tear everything down on exit / Ctrl+C
cleanup() {
    echo
    echo "Shutting down…"
    kill "$LAUNCH_PID" 2>/dev/null || true
    pkill -f single_arm_tables 2>/dev/null || true
    pkill -f "rviz2 -d" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- wait for /move_group to come up (max ~60 s) ---
echo "Waiting for move_group…"
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "^/move_group$"; then
        echo "move_group is up."
        break
    fi
    sleep 1
done

# small settle so the planning scene / controllers are ready
sleep 3

# --- RViz in the foreground (full MoveIt config → interactive marker works) ---
RVIZ_CONFIG="$WS/install/kinova_gen3_lite_moveit_config/share/kinova_gen3_lite_moveit_config/config/moveit.rviz"
echo "Launching RViz…"
ros2 launch kinova_gen3_lite_moveit_config moveit_rviz.launch.py \
    rviz_config:="$RVIZ_CONFIG"
