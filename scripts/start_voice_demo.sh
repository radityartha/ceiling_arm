#!/usr/bin/env bash
# One-terminal wrapper for the voice/web control demo: Whisper STT driver +
# table controller + MoveIt/RViz (all 4 real arms) + the voice/web pipeline.
# Ctrl+C tears everything down.
#
# Equivalent to running these 4 commands in 4 separate terminals:
#   1) ros2 launch whisper_bringup whisper.launch.py
#   2) ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false
#   3) ./scripts/start_single_rviz.sh
#   4) ros2 launch sayai_voice_sim workcell_voice.launch.py
#
# Requires a working microphone (see /dev/snd permissions, `audio` group).
# Without a real mic, the Whisper node still starts but /whisper/transcription
# never gets meaningful text -- the web UI's mic button works independently
# via the browser's Web Speech API.
#
# Whisper is SKIPPED BY DEFAULT (see SKIP_WHISPER below) -- a previous
# audio_capturer_node got stuck in kernel D-state (uninterruptible, survives
# even SIGKILL -- a sign of a wedged sof-hda-dsp driver) and won't clear until
# reboot, so defaulting to "on" would block every startup with the stray-node
# check below until that's fixed.
#
# Usage:  ./scripts/start_voice_demo.sh                  # Whisper skipped
#         SKIP_WHISPER=0 ./scripts/start_voice_demo.sh   # include Whisper
set -e
SKIP_WHISPER="${SKIP_WHISPER:-1}"

WS="$HOME/Documents/ceiling_arm/ros2_ws"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

LOG_DIR="$(mktemp -d /tmp/voice_demo_logs.XXXXXX)"
echo "Logs: $LOG_DIR"

# Nodes spawned by workcell_voice.launch.py. They must be matched separately
# from the launch process: killing `ros2 launch` does not always take its
# children with it, and their command lines do not contain "workcell_voice".
VOICE_NODES="sayai_voice_sim/(voice_web_ui|real_robot_task_server|voice_command_manager|whisper_transcript_bridge)"

# Same for the MoveIt/ros2_control side started by start_single_rviz.sh:
# pkill'ing the launch wrapper leaves these behind. A second move_group means
# two /move_action servers, which makes arm goals fail with CONTROL_FAILED.
MOVEIT_NODES="(moveit_ros_move_group/move_group|controller_manager/ros2_control_node)"

# Same for whisper_bringup's whisper.launch.py: whisper_ros nodes + the
# audio_common capturer, which otherwise keep holding the mic device open.
WHISPER_NODES="(whisper_ros/(whisper_node|whisper_server_node|silero_vad_node)|audio_common/audio_capturer_node)"

# Refuse to start on top of a previous run. Two real_robot_task_servers each
# enforce "one sequence at a time" only within their own process, so a single
# button press can start the same sequence twice on real hardware.
if [ "$SKIP_WHISPER" = "1" ]; then
    STRAY="$(pgrep -f "$VOICE_NODES" || true; pgrep -f "$MOVEIT_NODES" || true)"
else
    STRAY="$(pgrep -f "$VOICE_NODES" || true; pgrep -f "$MOVEIT_NODES" || true; pgrep -f "$WHISPER_NODES" || true)"
fi
if [ -n "$STRAY" ]; then
    echo "ERROR: nodes from a previous run are still running:" >&2
    ps -o pid,lstart,cmd -p $(echo "$STRAY" | tr '\n' ',' | sed 's/,$//') >&2
    echo >&2
    echo "Starting a second copy would double-trigger sequences on real hardware," >&2
    echo "and a duplicate move_group makes arm goals fail with CONTROL_FAILED." >&2
    echo "Stop them first:" >&2
    echo "  pkill -f \"$VOICE_NODES\"" >&2
    echo "  pkill -f \"$MOVEIT_NODES\"" >&2
    echo "  pkill -f \"$WHISPER_NODES\"" >&2
    exit 1
fi

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
    # ...and the nodes both launches spawned, which survive their launch process.
    pkill -f "$VOICE_NODES" 2>/dev/null || true
    pkill -f "$MOVEIT_NODES" 2>/dev/null || true
    pkill -f "$WHISPER_NODES" 2>/dev/null || true
    pkill -f whisper.launch 2>/dev/null || true
    pkill -f dual_table_controller 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$SKIP_WHISPER" = "1" ]; then
    echo "[1/4] SKIP_WHISPER=1 -- skipping Whisper STT driver."
else
    echo "[1/4] Starting Whisper STT driver (mic capture + transcription)..."
    ros2 launch whisper_bringup whisper.launch.py \
        > "$LOG_DIR/whisper_driver.log" 2>&1 &
    PIDS+=($!)

    echo "Waiting for /whisper/whisper_node (model load can take a while)..."
    for i in $(seq 1 90); do
        if ros2 node list 2>/dev/null | grep -q "^/whisper/whisper_node$"; then
            echo "Whisper node is up."
            break
        fi
        sleep 1
    done
fi

echo "[2/4] Starting table controller..."
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

echo "[3/4] Starting MoveIt + RViz (all 4 arms, real hardware)..."
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

echo "[4/4] Starting voice + web control pipeline..."
ros2 launch sayai_voice_sim workcell_voice.launch.py \
    > "$LOG_DIR/voice_web.log" 2>&1 &
PIDS+=($!)

sleep 2
# Prefer a LAN IP reachable from a phone/browser; 192.168.2.x is the arm-only
# subnet and won't be reachable from anything but this PC.
LAN_IP="$(hostname -I | tr ' ' '\n' | grep -v '^192\.168\.2\.' | grep -E '^[0-9]+\.' | head -1)"
echo
if [ "$SKIP_WHISPER" = "1" ]; then
    echo "3/4 subsystems running (Whisper skipped)."
else
    echo "All 4 subsystems running."
fi
echo "Web UI:  https://${LAN_IP:-<this-PC-LAN-IP>}:8080"
echo "Logs:    tail -f $LOG_DIR/*.log"
echo "Ctrl+C to stop everything."
echo

tail -f "$LOG_DIR"/*.log
