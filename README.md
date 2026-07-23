beri guidance untk # Moonshot Workcell Project

A ROS 2 workspace for controlling an automated workcell consisting of two ceiling-mounted motorized tables, four Kinova Gen3 Lite robotic arms, four 2-finger grippers, and a Livox Mid360 3D LIDAR sensor — all integrated with MoveIt 2 for motion planning.

---

## System Architecture

```
Workcell
├── Table 1  (linear + rotational, ceiling-mounted)
│   ├── Arm 1  — Kinova Gen3 Lite 6DOF + 2F gripper  @ 192.168.2.13  (left mount)
│   └── Arm 2  — Kinova Gen3 Lite 6DOF + 2F gripper  @ 192.168.2.12  (right mount)
│
├── Table 2  (linear + rotational, ceiling-mounted)
│   ├── Arm 3  — Kinova Gen3 Lite 6DOF + 2F gripper  @ 192.168.2.11  (left mount)
│   └── Arm 4  — Kinova Gen3 Lite 6DOF + 2F gripper  @ 192.168.2.10  (right mount)
│
└── Livox Mid360 LIDAR  (overhead, world frame: x=2.3 y=0 z=1.9)
    └── Object detection & MoveIt octomap collision avoidance
```

Each table is driven by **3 Oriental Motor stepper motors** (2 linear + 1 rotational) over **Modbus RTU** serial.  
Each arm connects to the Kinova API over Ethernet.

---

## Repository Layout

```
ceiling_arm/
├── Dockerfile                    # ROS 2 Humble image build
├── docker-compose.yml
├── scripts/
│   ├── build.sh                  # Build Docker image
│   └── run.sh                    # Run Docker container (X11, host network, USB devices)
├── dependencies/
│   └── kortex_api-2.6.0.post3-py3-none-any.whl   # Kinova Python API wheel
└── ros2_ws/
    ├── build_all.sh              # Local colcon build script
    └── src/
        ├── workcell_description/         # URDF/XACRO + LIDAR nodes + utility scripts
        ├── workcell_moveit_config/       # MoveIt 2 config for all groups
        ├── moving_table_pkg/             # Dual table controller (Modbus RTU)
        ├── moving_table_interfaces/      # ROS 2 service definition for table control
        ├── kinova_gen3_lite_control/     # (placeholder — superseded by ros2_kortex)
        ├── lidar_integration/            # (placeholder — handled in workcell_description)
        ├── ros2_kortex/                  # Kinova official ROS 2 driver (submodule)
        └── livox_ros_driver2/            # Livox LIDAR ROS 2 driver (submodule)
```

---

## ROS 2 Packages

### `workcell_description`
Defines the complete robot model and provides sensing/utility nodes.

**URDF/XACRO:**
- `urdf/workcell.urdf.xacro` — top-level model: 2 tables + 4 arms + 4 grippers
- `urdf/moving_table.urdf.xacro` — table model with linear and rotation joints

**Launch files:**
| File | Purpose |
|------|---------|
| `workcell_bringup.launch.py` | Full system: RSP, table controller, joint state GUI, RViz |
| `lidar_filter.launch.py` | LIDAR crop-box filter node (4×4×1.8 m workspace) |
| `workcell_view.launch.py` | Visualization only |

**Scripts/Nodes:**
| Script | What it does |
|--------|-------------|
| `lidar_filter.py` | Voxel downsampling → DBSCAN clustering → bounding-box object detection; publishes `/livox/filtered` and `/detected_object_pose` |
| `lidar_processor.py` | Alternative LIDAR processor feeding object poses into MoveIt |
| `move_arm_commander.py` | Example MoveIt Commander node for arm trajectory execution |
| `save_pcd.py` | Records raw LIDAR clouds to `~/lidar_dataset/*.pcd` |
| `get_pose.py` | Utility for reading current end-effector pose |

---

### `workcell_moveit_config`
MoveIt 2 configuration for the entire workcell.

**Planning groups** (defined in `config/trailer_workcell.srdf`):
- `arm_1` … `arm_4` — individual 6-DOF arm groups
- `gripper_1` … `gripper_4` — individual gripper groups
- `table_1`, `table_2` — per-table compound groups
- `table_1_with_arm`, `table_2_with_arm` — arm+table compound groups

**Key config files:**
| File | Purpose |
|------|---------|
| `moveit_controllers.yaml` | FollowJointTrajectory (tables + arms) + GripperCommand controllers |
| `kinematics.yaml` | KDL IK solver for all groups |
| `sensors_3d.yaml` | Octomap integration with LIDAR point cloud |
| `joint_limits.yaml` / `pilz_cartesian_limits.yaml` | Safety limits |
| `initial_positions.yaml` | Default home joint positions |

**Main launch file:**
```bash
ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false
```
This spawns: `joint_state_broadcaster`, 2 table controllers, 4 arm controllers, Move Group node, RViz, LIDAR static TF, and the Livox driver.

---

### `moving_table_pkg`
Controls both ceiling tables over Modbus RTU serial.

**Node:** `dual_table_controller`

**Service:** `move_dual_table` (`moving_table_interfaces/srv/MovingTable`)

| Field | Type | Description |
|-------|------|-------------|
| `table_id` | string | `"table1"` or `"table2"` |
| `distance_mm` | float | Linear displacement in mm |
| `angle_deg` | float | Rotation in degrees |
| `linear_speed` | int | Motor speed for linear axes |
| `rotate_speed` | int | Motor speed for rotation axis |
| `operation_type` | int | 0 = move, 1 = rotate, 2 = both |
| `success` (resp) | bool | Whether the command succeeded |
| `message` (resp) | string | Status/error message |

**Key parameters:**
```yaml
table1_port: /dev/ttyUSB0
table2_port: /dev/ttyUSB1
baud_rate: 115200
fake_hardware: false   # set true for simulation without physical motors
```

Publishes `JointState` for joints: `t1_linear`, `t1_rotation`, `t2_linear`, `t2_rotation`.

---

### `moving_table_interfaces`
Defines the `MovingTable.srv` service type used by `moving_table_pkg`.

---

### `ros2_kortex` (submodule)
Official Kinova ROS 2 driver. Provides:
- `kortex_driver` — C++ hardware interface for arm communication over Ethernet
- `kortex_description` — URDF/XACRO for Gen3 Lite 6DOF + 2F gripper
- `kortex_bringup` — launch files (`gen3_lite.launch.py`, `gen3_dual.launch.py`)
- `kortex_moveit_config` — standalone MoveIt configs per arm

---

### `livox_ros_driver2` (submodule)
Official Livox ROS 2 driver for the **Mid360** sensor.

Publishes:
- `/livox/points` — standard `sensor_msgs/PointCloud2`
- `/livox/lidar` — Livox custom format

Key launch files:
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py    # driver only
ros2 launch livox_ros_driver2 rviz_MID360_launch.py   # driver + RViz
```

Config file: `livox_ros_driver2/config/MID360_config.json`

---

## Prerequisites

| Dependency | Version |
|-----------|---------|
| ROS 2 | Humble (Ubuntu 22.04) |
| MoveIt 2 | Humble |
| Open3D | latest |
| PyModbus | ≥3.0 |
| PySerial | latest |
| Kinova Kortex API | 2.6.0 (wheel in `dependencies/`) |

---

## Build

### Option A — Local build (without Docker)

```bash
# Source workspace dependencies first
source /opt/ros/humble/setup.bash
source ~/rviz2_ws/install/setup.bash       # if separate rviz2 ws
source ~/moveit2_ws/install/setup.bash     # if separate moveit2 ws

cd ~/Documents/ceiling_arm/ros2_ws
./build_all.sh
source install/setup.bash
```

`build_all.sh` handles the Livox driver separately (it uses its own `build.sh`) then builds the rest with colcon.

### Option B — Docker

```bash
cd ~/Documents/ceiling_arm

# Build image (requires kortex_api wheel in dependencies/)
./scripts/build.sh

# Run container (X11 forwarding, host network, USB device access)
./scripts/run.sh
```

---

## Running the System

### Full workcell with MoveIt (real hardware)

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/ceiling_arm/ros2_ws/install/setup.bash

ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false
```

### LIDAR only

```bash
# Start Livox driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Start filtered point cloud node
ros2 launch workcell_description lidar_filter.launch.py
```

### Visualization only (no hardware)

```bash
ros2 launch workcell_moveit_config demo.launch.py use_sim_time:=false
```

### Move an arm (example)

```bash
python3 ros2_ws/src/workcell_description/scripts/move_arm_commander.py
```

---

## Sequence Demos

All sequence launches are one-shot runners. They require two things to already be running before you launch them:

**Terminal 1 — MoveIt + RViz** (curtain/bottle use `single_rviz_workcell`, bag uses `my_workcell`):
```bash
# curtain + bottle demos
ros2 launch workcell_moveit_config single_rviz_workcell.launch.py
# take-bag demo
ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false
```

**Terminal 2 — Table controller** (only one instance ever):
```bash
ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false
```

**Terminal 3 — Run the sequence:**

### Close curtain
```bash
ros2 launch workcell_moveit_config close_curtain_demo.launch.py

# With compliant pull on arm_1 (yields under load, requires patched Kortex driver)
ros2 launch workcell_moveit_config close_curtain_demo.launch.py use_compliant_pull:=true

# Fake tables (no hardware)
ros2 launch workcell_moveit_config close_curtain_demo.launch.py start_table_controller:=true use_fake_tables:=true
```

### Open curtain
```bash
ros2 launch workcell_moveit_config open_curtain_demo.launch.py

# Fake tables
ros2 launch workcell_moveit_config open_curtain_demo.launch.py start_table_controller:=true use_fake_tables:=true
```

### Take bag
```bash
ros2 launch workcell_moveit_config take_bag_demo.launch.py

# Fake tables
ros2 launch workcell_moveit_config take_bag_demo.launch.py start_table_controller:=true use_fake_tables:=true
```

### Take bottle
```bash
ros2 launch workcell_moveit_config take_bottle_demo.launch.py

# Fake tables
ros2 launch workcell_moveit_config take_bottle_demo.launch.py start_table_controller:=true use_fake_tables:=true
```

### Common tuning arguments (all 4 demos)

| Argument | Default | Notes |
|---|---|---|
| `vel_scale` | 0.1 (0.2 for take_bag) | Arm velocity scaling 0–1 |
| `acc_scale` | 0.1 (0.2 for take_bag) | Arm acceleration scaling 0–1 |
| `gripper_grip_deg` | 40.0 (45.0 for bag/bottle) | Grip angle in degrees |
| `skip_grippers` | false | Skip all gripper steps |
| `linear_speed` | 3000 | Table linear speed (pulses/s) |
| `rotate_speed` | 1000 | Table rotation speed (pulses/s) |
| `startup_delay_s` | 3.0 | Wait (s) before first command |
| `planning_time` | 10.0 | MoveIt planning timeout (s) |

> **Never run two `dual_table_controller` nodes at once** — the second cannot acquire the serial port lock and comes up with `table1/table2 = None`, causing the runner to abort immediately.

---

## Voice & Web Control

Package [`sayai_voice_sim`](ros2_ws/src/sayai_voice_sim/) triggers the four sequence demos above from spoken commands and/or a local HTTPS web page — no need to type `ros2 launch` by hand. It depends on the `whisper_ros` and `audio_common` submodules (speech-to-text, TTS, audio capture).

### Quick start (1 terminal)

```bash
./scripts/start_voice_demo.sh
```
Runs all 3 pieces below as background jobs with logs under `/tmp/voice_demo_logs.*`, tailing
them to this terminal. `Ctrl+C` stops everything (table controller, MoveIt/RViz, voice pipeline).
Use the manual 3-terminal setup instead when you need to restart just one piece (e.g. after the
`voice_web_ui` wedge issue below) without tearing down the others.

### Setup (3 terminals)

```bash
# Terminal 1 — base/table controller (same as any sequence demo)
ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false

# Terminal 2 — MoveIt + RViz, all 4 arms on real hardware
./scripts/start_single_rviz.sh
```
> **Use `start_single_rviz.sh` / `single_rviz_workcell.launch.py` here, not `my_workcell.launch.py`.**
> `my_workcell.launch.py` defaults to `use_fake_hardware:=true`, and even when overridden its
> default `arm1_ip..arm4_ip` mapping is reversed against the confirmed real IPs (see
> [Pre-flight check](#1-pre-flight-check--verify-everything-is-wired-and-online)). Since a voice/web command can fire any of
> the four sequences — each of which drives all four arms — the wrong mapping means the
> software group `arm_1` could end up commanding the wrong physical arm.

```bash
# Terminal 3 — voice + web pipeline
ros2 launch sayai_voice_sim workcell_voice.launch.py
# dry-run the speech/web layer without moving the robot:
ros2 launch sayai_voice_sim workcell_voice.launch.py use_mock:=true
```

This starts 4 nodes: `whisper_transcript_bridge` (wake-word gate), `voice_command_manager`
(phrase → `/task/*` Trigger service), `real_robot_task_server` (runs the matching
`*_demo.launch.py` as a subprocess), and `voice_web_ui` (the web page below).

### Web UI

Open `https://<PC-LAN-IP>:8080` from a phone or browser on the same network (find the IP with
`ip -4 -brief addr show`; use the WiFi/Ethernet interface's `192.168.0.x` address, **not** the
`192.168.2.x` arm subnet). The page serves a self-signed cert, so click through the browser's
"not secure" warning once.

The page has: task buttons (Open/Close Curtain, Bring Bag, Bring Bottle, Stop), a browser mic
button (Web Speech API — Chrome/Chromium only, sends recognized text straight to
`/api/transcript`), and a text box for typing a command directly. All three paths land on the
same `/voice/transcript` topic that `voice_command_manager` listens to.

### Voice / typed command phrases

From [`config/voice_commands.yaml`](ros2_ws/src/sayai_voice_sim/config/voice_commands.yaml):

| Say / type | Triggers |
|---|---|
| `open`, `open curtain` | `open_curtain_demo.launch.py` |
| `close`, `close curtain` | `close_curtain_demo.launch.py` |
| `bag`, `bring bag` | `take_bag_demo.launch.py` |
| `bottle`, `bring bottle` | `take_bottle_demo.launch.py` |
| `stop`, `cancel`, `halt` | SIGINT the currently running sequence |

Only one sequence runs at a time — a command that arrives while another is still executing is
rejected (say "stop" first). If audio comes through the real Whisper mic pipeline (not the
browser mic or text box), spoken commands are ignored unless preceded by a wake word (default
`require_wake_word:=true`) — see `whisper_transcript_bridge.py` for the wake-word list and the
30 s listening window.

### Troubleshooting

- **Web page unreachable even though the node is running** — the bundled `voice_web_ui.py`
  wraps Python's stdlib `http.server` with SSL by hand; a client that opens a TCP connection
  without completing the TLS handshake (a raw port probe, or `http://` instead of `https://`)
  can wedge its single-threaded `accept()` forever, silently queuing out every future
  connection. Symptom: `ss -tlnp | grep 8080` still shows `LISTEN`, but every request times out.
  Fix: find the PID (`ss -tlnp | grep 8080`), `kill -9 <pid>`, then
  `ros2 run sayai_voice_sim voice_web_ui` again — the other 3 nodes don't need restarting.
- **A sequence silently "succeeds" in the UI but the arms didn't move / an ABORTING line is in
  the log** — `real_robot_task_server` only reports failure if the `*_demo.launch.py` subprocess
  exits non-zero; check each `run_*.py` script actually does `sys.exit(main())`, not a bare
  `main()` call, or a failed sequence will still report `finished OK`.

---

## Hardware Check & Manual Control

Helper scripts in [scripts/](scripts/) for bringing the system up safely.

### 1. Pre-flight check — verify everything is wired and online

```bash
python3 scripts/hardware_check.py --preflight \
    --arm-ips 192.168.2.13 192.168.2.12 192.168.2.11 192.168.2.10
```

Verifies USB serial ports (`/dev/ttyUSB0`, `/dev/ttyUSB1`) and pings all 4 Kinova arms.

**Confirmed arm IPs:** Arm 1 = `192.168.2.13`, Arm 2 = `192.168.2.12`, Arm 3 = `192.168.2.11`, Arm 4 = `192.168.2.10`.

**Note on arm IPs:** The arms live on subnet `192.168.2.x`. If your PC's Ethernet is on a different subnet (e.g. `192.168.2.100`), they're directly reachable. If on `192.168.1.x`, add a secondary IP:
```bash
sudo ip addr add 192.168.1.100/24 dev enp1s0
```

### 2. Keyboard remote control — drive the tables by hand

A terminal-based "remote control" for the two motorized tables. Used for manual positioning, homing, and verifying motor health.

**Start the table controller** (one-time, in its own terminal):
```bash
source ~/Documents/ceiling_arm/ros2_ws/install/setup.bash
ros2 run moving_table_pkg dual_table_controller --ros-args -p use_fake_hardware:=false
```

**Run the keyboard remote** (in a second terminal):
```bash
source ~/Documents/ceiling_arm/ros2_ws/install/setup.bash
python3 scripts/table_keyboard.py
```

**Controls:**

| Key | Action |
|-----|--------|
| **Hold W** | Linear forward (continuous, stops on release) |
| **Hold S** | Linear backward |
| **Hold D** | Rotate clockwise |
| **Hold A** | Rotate counter-clockwise |
| `1` / `2` | Switch active table (Table 1 / Table 2) |
| `]` / `[` | Increase / decrease motor speed |
| `Z` | Zero **display** for active table (visual offset only — encoder unchanged) |
| `X` | Zero **display** for both tables |
| **`H`** | **HOME — set hardware encoder origin to 0 at current physical position. Persists across power cycles.** |
| `Q` | Quit (sends stop) |

**How it works:**
- Motors run in continuous-velocity mode while a key is held. The instant you release, the script sends `motor.stop()` over Modbus — the motor halts within ~5 ms.
- The HUD shows live linear (mm) and rotation (°) position of the active table, read from `/joint_states`.
- `H` calls Oriental Motor's `ppreset` command on all 3 motors of the active table — the AZ-series absolute encoder retains this origin even after powering down.

**Typical homing workflow:**
1. Drive Table 1 to your desired home position with W/S/A/D
2. Press `1` then `H` → `🏠 table1 home set at current position`
3. Press `2`, drive Table 2 to its home, press `H`
4. Both tables now read `+0.0 mm / +0.0 °` and will keep this origin permanently.

### 3. Single-arm MoveIt bringup (standalone test)

Test one arm + the tables + MoveIt RViz in **one terminal** — does **not** require the
full workcell launch.

```bash
./scripts/start_single_arm.sh 192.168.2.10     # arm 4 (table 2, right)
./scripts/start_single_arm.sh 192.168.2.11     # arm 3 (table 2, left)
```

The script:
1. Strips the broken `rviz2_ws` / `moveit2_ws` overlays so the **system** rviz2 is used
   (prints `Using rviz2: /opt/ros/humble/bin/rviz2`).
2. Runs `single_arm_tables.launch.py` (arm + tables + move_group) in the background.
3. Polls until `/move_group` is up.
4. Launches RViz in the foreground with the full MoveIt config (interactive marker works).

`Ctrl+C` tears the whole thing down.

In RViz: **MotionPlanning** panel → **Planning Group**: `arm` → drag the interactive
marker to a goal → **Plan** → **Execute**.

Drive the tables at the same time from a second terminal:
```bash
python3 scripts/table_keyboard.py
```

> **Why a script and not `ros2 launch ... launch_rviz:=true`?** RViz is on a `TimerAction`
> inside the combined launch, but the included kortex/move_group sub-launches starve the
> launch event loop so the timer never fires and no window appears. The script polls for
> readiness instead, which is reliable. (`launch_rviz` defaults to `false` for this reason.)

**Troubleshooting:**

- **Arm not reachable** — `ping 192.168.2.10`. If it fails, check power/Ethernet; the arm
  takes ~60 s to boot. PC must be on `192.168.2.x` (e.g. `192.168.2.100`).
- **`plan & execute` silently aborts** —
  `ros2 param set /move_group moveit_manage_controllers false`, then retry.
- **Planning fails instantly** — a joint is out of bounds. Send the arm to a safe pose first
  (clear the workspace!):
  ```bash
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.0, 0.28, -1.57, 0.0, -1.05, 0.0], time_from_start: {sec: 8, nanosec: 0}}]}}"
  ```
- **RViz won't open / wrong rviz2** — confirm `which rviz2` shows `/opt/ros/humble/bin/rviz2`.
  A new terminal self-heals via `.bashrc`; if not, the script strips the overlay itself.

---

### 4. Full automated hardware check (tables + arms + grippers)

```bash
python3 scripts/hardware_check.py --tables    # move each table ±50 mm
python3 scripts/hardware_check.py --arms      # move each arm to home via MoveIt
python3 scripts/hardware_check.py --grippers  # open/close each gripper
```

(Arms and grippers tests require `my_workcell.launch.py` to be running with `use_fake_hardware:=false`.)

---

## Named Joint Positions

Defined in `workcell_moveit_config/config/trailer_workcell.srdf`:

| Pose name | Description |
|-----------|-------------|
| `arm_1_home` | `[0, -0.2792, 1.309, 0, -1.0471, 0]` |
| `arm_1_packaging` | `[0, 2.583, 2.583, 1.5708, -2.4434, 0]` |
| (similar for arm_2–4) | |

---

## TF Frame Layout

| Frame | Parent | Notes |
|-------|--------|-------|
| `world` | — | Global origin |
| `table_1_base` | `world` | y = +0.36, z = 2.05 |
| `table_2_base` | `world` | y = −0.36, z = 2.05 |
| `arm_1_base_link` | `table_1_mount_left` | |
| `arm_2_base_link` | `table_1_mount_right` | |
| `arm_3_base_link` | `table_2_mount_left` | |
| `arm_4_base_link` | `table_2_mount_right` | |
| `livox_frame` | `world` | x=2.3, z=1.9, pointing down |

TF tree PDF snapshots are saved in `ros2_ws/frames_*.pdf`.

---

## External Resources

- Kinova Gen3 Lite docs: [github.com/Kinovarobotics/ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex)
- Livox Mid360 driver: [github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- MoveIt 2 Humble: [moveit.picknik.ai](https://moveit.picknik.ai)
