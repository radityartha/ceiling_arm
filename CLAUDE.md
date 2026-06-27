# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# From ros2_ws/
cd ros2_ws
./build_all.sh                 # builds livox driver first, then the rest with colcon
source install/setup.bash
```

`build_all.sh` builds `livox_ros_driver2` via its own `build.sh humble`, then runs `colcon build --symlink-install` on the remaining packages (Livox SDK is ignored to avoid double-build).

Docker alternative:
```bash
./scripts/build.sh             # build image (needs dependencies/kortex_api-*.whl)
./scripts/run.sh               # X11 forwarding, host network, USB pass-through
```

## Test

```bash
# From ros2_ws/
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

There is no formal test suite in this repo yet. Hardware/integration checks are run via [scripts/hardware_check.py](scripts/hardware_check.py) — see "Hardware Check" in [README.md](README.md).

## Architecture Overview

ROS 2 Humble workspace for an automated workcell: **2 ceiling-mounted motorized gantries**, **4 Kinova Gen3 Lite 6-DOF arms** (2 per gantry) each with a 2-finger gripper, and a **Livox Mid360 3D LIDAR** for collision sensing — all unified under MoveIt 2.

> **Naming note (renamed 2026-06-26):** the ceiling-mounted moving platforms are called **gantry** (not "table") to disambiguate from the workpiece **work table**. The rename covers MoveIt planning groups, controllers, kinematics, SRDF, and TF/frame docs (`gantry_1`, `gantry_2`, `gantry_1_with_arm`, `gantry_1_controller`, …). **Intentionally still named "table"** (hardware/driver layer, out of scope): the `moving_table_pkg` / `moving_table_interfaces` packages, the `MovingTable` service + its `table_id` field + `"table1"`/`"table2"` id strings, the URDF joint/link prefixes `t1_`/`t2_`, `dual_table_controller`, `move_dual_table`, and the `--tables` CLI flag.

### Hardware topology

```
world
├── gantry_1_base  (linear + rotation, Modbus RTU on /dev/ttyUSB0)
│   ├── arm_1   (Kinova Gen3 Lite, Ethernet @ 192.168.2.13)
│   └── arm_2   (Kinova Gen3 Lite, Ethernet @ 192.168.2.12)
├── gantry_2_base  (linear + rotation, Modbus RTU on /dev/ttyUSB1)
│   ├── arm_3   (Kinova Gen3 Lite, Ethernet @ 192.168.2.11)
│   └── arm_4   (Kinova Gen3 Lite, Ethernet @ 192.168.2.10)
└── livox_frame    (Mid360 LIDAR, overhead, x=2.3 z=1.9)
```

Each gantry is driven by 3 Oriental Motor stepper motors (2 linear + 1 rotational) over Modbus RTU. Each arm speaks the Kinova Kortex API over Ethernet on subnet `192.168.2.x`.

### Package dependency flow

```
workcell_description            (URDF/Xacro + LIDAR processing nodes)
         ↓
moving_table_interfaces         (MovingTable.srv definition)
         ↓
moving_table_pkg                (dual_table_controller — Modbus RTU)
         ↓
workcell_moveit_config          (SRDF, kinematics, controllers, sensors_3d)
         ↑
ros2_kortex (submodule)         (Kinova arm driver + URDF + bringup)
livox_ros_driver2 (submodule)   (Livox Mid360 driver)
```

`kinova_gen3_lite_control` and `lidar_integration` are placeholder packages — their functionality lives in `ros2_kortex` and `workcell_description` respectively.

### [ros2_ws/src/workcell_description/](ros2_ws/src/workcell_description/)

Robot model and perception nodes.

- `urdf/workcell.urdf.xacro` — top-level: 2 tables + 4 arms + 4 grippers + LIDAR
- `urdf/moving_table.urdf.xacro` — single table with linear + rotation joints
- Launch: `workcell_bringup.launch.py` (full system), `lidar_filter.launch.py` (4×4×1.8 m crop-box), `workcell_view.launch.py` (visualization only)
- Scripts: `lidar_filter.py` (voxel downsample → DBSCAN → bounding-box detection, publishes `/livox/filtered` and `/detected_object_pose`), `lidar_processor.py` (alt processor feeding MoveIt), `move_arm_commander.py` (example MoveIt Commander), `save_pcd.py`, `get_pose.py`

### [ros2_ws/src/workcell_moveit_config/](ros2_ws/src/workcell_moveit_config/)

MoveIt 2 config for the whole workcell.

**Planning groups** (in `config/trailer_workcell.srdf`):
- `arm_1` … `arm_4` — single 6-DOF arm
- `gripper_1` … `gripper_4` — single gripper
- `gantry_1`, `gantry_2` — gantry-only
- `gantry_1_with_arm`, `gantry_2_with_arm` — coupled arm + gantry

**Primary launch:** [launch/my_workcell.launch.py](ros2_ws/src/workcell_moveit_config/launch/my_workcell.launch.py) spawns `joint_state_broadcaster`, 2 table controllers, 4 arm controllers, Move Group, RViz, LIDAR static TF, and the Livox driver.

```bash
ros2 launch workcell_moveit_config my_workcell.launch.py use_sim_time:=false
```

Key config files:
- `moveit_controllers.yaml` — FollowJointTrajectory (tables + arms) + GripperCommand
- `kinematics.yaml` — KDL per group
- `sensors_3d.yaml` — octomap from LIDAR cloud
- `joint_limits.yaml`, `pilz_cartesian_limits.yaml` — safety limits
- `initial_positions.yaml` — home joints

### [ros2_ws/src/moving_table_pkg/](ros2_ws/src/moving_table_pkg/)

Node `dual_table_controller` ([moving_table_pkg/dual_table_controller.py](ros2_ws/src/moving_table_pkg/moving_table_pkg/dual_table_controller.py)) drives both tables over Modbus RTU and publishes `JointState` for `t1_linear`, `t1_rotation`, `t2_linear`, `t2_rotation`.

Service `move_dual_table` (`moving_table_interfaces/srv/MovingTable`):
- `table_id` (`"table1"` | `"table2"`), `distance_mm`, `angle_deg`, `linear_speed`, `rotate_speed`
- `operation_type`: 0 = move, 1 = rotate, 2 = both

Parameters:
```yaml
table1_port: /dev/ttyUSB0
table2_port: /dev/ttyUSB1
baud_rate: 115200
use_fake_hardware: false
```

`H` in [scripts/table_keyboard.py](scripts/table_keyboard.py) calls Oriental Motor's `ppreset` on all 3 motors — sets the AZ-series absolute encoder origin, persists across power cycles.

### Submodules

- [ros2_ws/src/ros2_kortex/](ros2_ws/src/ros2_kortex/) — official Kinova ROS 2 driver: `kortex_driver` (HW interface), `kortex_description` (Gen3 Lite + 2F gripper URDF), `kortex_bringup`, `kortex_moveit_config`.
- [ros2_ws/src/livox_ros_driver2/](ros2_ws/src/livox_ros_driver2/) — Livox Mid360 driver. Publishes `/livox/points` (`sensor_msgs/PointCloud2`) and `/livox/lidar` (Livox custom). Config: `config/MID360_config.json`. Built separately by `build_all.sh`.

### Coordinate frames

| Frame | Parent | Notes |
|-------|--------|-------|
| `world` | — | global origin |
| `gantry_1_base` | `world` | y = +0.36, z = 2.05 |
| `gantry_2_base` | `world` | y = −0.36, z = 2.05 |
| `arm_1_base_link` | `gantry_1_mount_left` | |
| `arm_2_base_link` | `gantry_1_mount_right` | |
| `arm_3_base_link` | `gantry_2_mount_left` | |
| `arm_4_base_link` | `gantry_2_mount_right` | |
| `livox_frame` | `world` | x=2.3, z=1.9, pointing down |

### Helper scripts ([scripts/](scripts/))

- `hardware_check.py` — `--preflight` (USB + arm ping), `--tables` (±50 mm sweep), `--arms` (MoveIt home), `--grippers` (open/close). Requires `my_workcell.launch.py` running for `--arms`/`--grippers`.
- `table_keyboard.py` — hold W/S/A/D to drive active table; `1`/`2` switch table; `[`/`]` speed; `Z`/`X` zero display; `H` sets hardware encoder origin via `ppreset`; `Q` quit.

### Docker

[docker-compose.yml](docker-compose.yml) and [Dockerfile](Dockerfile) build from `ros:humble` with MoveIt 2, ros2_control, the Kortex API wheel, and host-network/X11/USB pass-through for hardware access.

### Subnet note

Arms live on `192.168.2.x`. If the PC is on `192.168.1.x`, add a secondary IP:
```bash
sudo ip addr add 192.168.1.100/24 dev enp1s0
```

## Working Rules

These rules apply to every task unless explicitly overridden.
Bias: caution over speed on non-trivial work. Use judgment on trivial tasks.

---

### Primary Rules — always active

### Rule 1 — Think Before Coding
State assumptions explicitly. If uncertain, ask rather than guess.
Present multiple interpretations when ambiguity exists.
Push back when a simpler approach exists.
Stop when confused. Name what's unclear.

### Rule 2 — Simplicity First
Minimum code that solves the problem. Nothing speculative.
No features beyond what was asked. No abstractions for single-use code.
Test: would a senior engineer say this is overcomplicated? If yes, simplify.

### Rule 3 — Surgical Changes
Touch only what you must. Clean up only your own mess.
Don't "improve" adjacent code, comments, or formatting.
Don't refactor what isn't broken. Match existing style.

### Rule 4 — Goal-Driven Execution
Define success criteria. Loop until verified.
Don't follow steps. Define success and iterate.
Strong success criteria let you loop independently.

### Rule 7 — Surface conflicts, don't average them
If two patterns contradict, pick one (more recent / more tested).
Explain why. Flag the other for cleanup.
Don't blend conflicting patterns.

### Rule 8 — Read before you write
Before adding code, read exports, immediate callers, shared utilities.
"Looks orthogonal" is dangerous. If unsure why code is structured a way, ask.

### Rule 10 — Checkpoint after every significant step
Summarize what was done, what's verified, what's left.
Don't continue from a state you can't describe back.
If you lose track, stop and restate.

### Rule 11 — Match the codebase's conventions, even if you disagree
Conformance > taste inside the codebase.
If you genuinely think a convention is harmful, surface it. Don't fork silently.

### Rule 12 — Fail loud
"Completed" is wrong if anything was skipped silently.
"Tests pass" is wrong if any were skipped.
Default to surfacing uncertainty, not hiding it.

---

### Secondary Rules — apply only when condition is met

### Rule 5 — Use the model only for judgment calls
Condition: when designing or reviewing code that calls an AI/LLM inside a pipeline.
Use Claude for: classification, drafting, summarization, extraction.
Do NOT use Claude for: routing, retries, deterministic transforms.
If code can answer, code answers.

### Rule 6 — Token budgets are not advisory
Condition: when a task is multi-step, open-ended, or has been running long.
Per-task: 4,000 tokens. Per-session: 30,000 tokens.
If approaching budget, summarize and start fresh.
Surface the breach. Do not silently overrun.

### Rule 9 — Tests verify intent, not just behavior
Condition: when writing or reviewing tests.
Tests must encode WHY behavior matters, not just WHAT it does.
A test that can't fail when business logic changes is wrong.
