# Moonshot Workcell Project

A ROS 2 workspace for controlling an automated workcell consisting of two ceiling-mounted motorized tables, four Kinova Gen3 Lite robotic arms, four 2-finger grippers, and a Livox Mid360 3D LIDAR sensor — all integrated with MoveIt 2 for motion planning.

---

## System Architecture

```
Workcell
├── Table 1  (linear + rotational, ceiling-mounted)
│   ├── Arm 1  — Kinova Gen3 Lite 6DOF + 2F gripper
│   └── Arm 2  — Kinova Gen3 Lite 6DOF + 2F gripper
│
├── Table 2  (linear + rotational, ceiling-mounted)
│   ├── Arm 3  — Kinova Gen3 Lite 6DOF + 2F gripper
│   └── Arm 4  — Kinova Gen3 Lite 6DOF + 2F gripper
│
└── Livox Mid360 LIDAR  (overhead, world frame: x=2.3 y=0 z=1.9)
    └── Object detection & MoveIt octomap collision avoidance
```

Each table is driven by **3 Oriental Motor stepper motors** (2 linear + 1 rotational) over **Modbus RTU** serial.  
Each arm connects to the Kinova API over Ethernet.

---

## Repository Layout

```
moonshot_project/
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

cd ~/Documents/moonshot_project/ros2_ws
./build_all.sh
source install/setup.bash
```

`build_all.sh` handles the Livox driver separately (it uses its own `build.sh`) then builds the rest with colcon.

### Option B — Docker

```bash
cd ~/Documents/moonshot_project

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
source ~/Documents/moonshot_project/ros2_ws/install/setup.bash

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
