# Simulation & GNG Topological Mapping

Additive Gazebo Fortress simulation and a GNG (Growing Neural Gas) topological
mapping layer for the ceiling-robot workcell. Everything here lives in **two new
packages** and **does not modify** any existing real-hardware description,
config, or launch file.

> Branch: `feat/sim-and-gng`
> Real-hardware path is byte-for-byte unchanged — see [What stays untouched](#what-stays-untouched).

---

## Why this exists

The real workcell (4 Kinova Gen3 Lite arms on 2 ceiling-mounted motorized tables
+ Livox Mid360) already builds and runs. This work adds, alongside it:

1. **`cell_gazebo_sim`** — a Gazebo Fortress simulation of the same cell, so you
   can develop perception/planning without the hardware.
2. **`cell_gng`** — a GNG-U2-style topological mapping node that consumes a
   `PointCloud2` (sim LiDAR now, real Mid360 later with no code change) and
   builds a node/edge map, plus a sim-only ground-truth validation harness.

---

## Architecture

```
                         ┌─────────────────────────────┐
                         │  cell_gazebo_sim (Fortress)  │
                         │                              │
   world ── tables ──────┤  gz_ros2_control            │
        (prismatic+rev)  │   • TableSimSystem (t1/t2)  │
            │            │   • 4 arms, 4 grippers       │
        arm bases        │  gpu_lidar  → /livox/points  │
            │            │  depth/rgb  → /camera/*      │
        Gen3 Lite ×4     └──────────────┬───────────────┘
                                        │ ros_gz_bridge
                                        ▼
                         ┌─────────────────────────────┐
                         │  cell_gng                    │
   /livox/points ───────▶│  gng_node                    │──▶ /gng/graph  (MarkerArray)
   (PointCloud2)         │   GNG-U growing network      │──▶ /gng/nodes  (PointCloud2)
                         │                              │
   /world/.../pose/info ▶│  gng_validation_node (SIM)   │──▶ centroid-vs-truth errors
   (→ PoseArray bridge)  └─────────────────────────────┘
```

The arm bases are children of the table-top links, so commanding a table joint
moves the arms with it (the "reconfiguration substrate").

---

## Prerequisites

- ROS 2 Humble
- **Gazebo Sim 6.x (Fortress)** — `ign gazebo --version` → 6.x
- `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`
- Python: `numpy`, `scipy`, `scikit-learn`, `sensor_msgs_py`

### Workspace note (important on this machine)

There are two `moonshot_project` trees and three `kortex_description` installs.
The **Fortress-correct** `kortex_description` (the one using
`gz_ros2_control/GazeboSimSystem`) is at `/home/mobi/ros2_ws/install/kortex_description`.
`cell_sim.launch.py` prepends it to `AMENT_PREFIX_PATH` automatically, so you
normally don't have to think about it. Work in the
`/home/mobi/Documents/ceilingrobot/moonshot_project` tree (this repo).

---

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Both new packages build with the rest of the workspace. The existing
`build_all.sh` flow is unchanged.

---

## Running

### 1. Simulation only

```bash
ros2 launch cell_gazebo_sim cell_sim.launch.py
# headless:
ros2 launch cell_gazebo_sim cell_sim.launch.py gz_gui:=false
```

Starts Gazebo + the world, spawns the cell, brings up `robot_state_publisher`,
the `ros_gz_bridge`, `joint_state_broadcaster`, and the table controllers.

Command a table (moves the arms mounted on it):

```bash
ros2 topic pub --once /table_1_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['t1_linear_joint','t1_rotation_joint'],
    points: [{positions: [0.5, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### 2. GNG over the simulation

```bash
# terminal 1
ros2 launch cell_gazebo_sim cell_sim.launch.py
# terminal 2
ros2 launch cell_gng gng_sim.launch.py
```

In RViz add a `MarkerArray` display on `/gng/graph` and a `PointCloud2` on
`/livox/points`; the GNG network grows over the live cloud.

### 3. GNG on the real Mid360 (later)

`cell_gng` is sensor-agnostic. Point it at the real driver topic — **no code
change**:

```bash
ros2 launch cell_gng gng.launch.py input_cloud_topic:=/livox/points
```

Use `gng.launch.py` (not `gng_sim.launch.py`) on hardware — the sim sandbox
launch pulls in the pose bridge and the sim-only validation node, which have no
meaning on the real robot.

---

## The GNG node

### Input contract

| | |
|---|---|
| Topic | `input_cloud_topic` parameter (default `/livox/points`) |
| Type | `sensor_msgs/msg/PointCloud2` |
| Required fields | `x`, `y`, `z` (float32), metres |
| Optional fields | `rgb` / `intensity` (read-tolerant, currently ignored) |
| Frame | the cloud's `header.frame_id`; markers publish in the same frame |

### Outputs

| Topic | Type | Contents |
|---|---|---|
| `/gng/graph` | `visualization_msgs/MarkerArray` | nodes (SPHERE_LIST) + edges (LINE_LIST) |
| `/gng/nodes` | `sensor_msgs/PointCloud2` | node positions (structured, downstream-consumable) |

### Parameters

All declared in [gng_node.py](ros2_ws/src/cell_gng/cell_gng/gng_node.py),
overridable via [config/gng_params.yaml](ros2_ws/src/cell_gng/config/gng_params.yaml):
`voxel_size`, `max_nodes`, `samples_per_cloud`, `eps_b`, `eps_n`, `max_age`,
`lambda_insert`, `alpha`, `beta`, `utility_k`, `roi_bounds`, `publish_rate_hz`.

> **Do NOT auto-tune these against the sim LiDAR.** The Fortress `gpu_lidar` is a
> generic **raster** approximation — it cannot reproduce the Mid360's
> non-repetitive scan pattern. Tuning on the sim cloud overfits to a simulator
> artifact. Tune on the real sensor. (This note is repeated in the code.)

### Algorithm — STUB

No existing GNG/GNG-U2 code was found in the workspace, so
[gng_core.py](ros2_ws/src/cell_gng/cell_gng/gng_core.py) is a **minimal GNG-U
(Fritzke) placeholder**, clearly marked for replacement by the lab's real
GNG-U2. The ROS node only depends on its small public API (`step`, `nodes`,
`edges`), so swapping the algorithm needs no changes to the node.

---

## Validation harness (simulation only)

`gng_validation_node` clusters the GNG nodes (DBSCAN), computes per-object
centroids, and compares them to the **true** model poses from Gazebo.

- Ground truth: Gazebo publishes `gz.msgs.Pose_V` on
  `/world/workcell/pose/info`; [config/pose_bridge.yaml](ros2_ws/src/cell_gng/config/pose_bridge.yaml)
  bridges it to `geometry_msgs/msg/PoseArray` on `/gng/ground_truth_poses`.
- It prints `error = <distance> m` per object every few seconds.

Sample output:

```
--- GNG centroid-vs-truth (STATIC truth, 197 nodes, 29 clusters) ---
  validation_box_1  truth=(+1.00,+0.00,+0.25)  nearest=(+0.96,+0.01,+0.23)  error=0.046 m
  validation_box_2  truth=(-1.00,+0.50,+0.20)  nearest=(-1.00,+0.50,+0.19)  error=0.013 m
```

**This node is sim-only — never launch it on hardware** (there is no
ground-truth pose source on the real robot).

---

## GPU / headless caveat

On some Intel integrated GPUs, Gazebo Fortress's **sensor rendering**
(`gpu_lidar`, `depth_camera`) crashes the Ogre render context. Physics,
`gz_ros2_control`, and pose info all still work headless — only the camera/LiDAR
rendering is affected. This is a GPU-driver limitation, not a code issue.

To exercise the full GNG pipeline without working sensor rendering, publish a
synthetic cloud on the same topic/type:

```bash
ros2 launch cell_gng gng_sim.launch.py use_fake_cloud:=true
```

`fake_cloud_publisher` emits a `PointCloud2` around the same box poses defined in
the world. On a machine with a working GPU render context, drop the flag and the
real sim sensors publish instead — no other change.

Required environment for the sim launch (set automatically by the launch file):

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_RESOURCE_PATH=/home/mobi/ros2_ws/install/kortex_description/share
```

---

## Package layout

```
ros2_ws/src/
├── cell_gazebo_sim/
│   ├── urdf/cell_sim.urdf.xacro       # sim top-level (arms sim_gazebo, tables, sensors)
│   ├── worlds/cell.sdf                # Fortress world + validation boxes
│   ├── config/sim_ros2_controllers.yaml
│   ├── config/bridge.yaml             # PointCloud2 / Image / CameraInfo / Clock
│   └── launch/cell_sim.launch.py
└── cell_gng/
    ├── cell_gng/gng_core.py           # GNG-U algorithm (STUB; numpy only)
    ├── cell_gng/gng_node.py           # ROS node (PointCloud2 → MarkerArray + PointCloud2)
    ├── cell_gng/gng_validation_node.py# SIM-ONLY centroid-vs-truth harness
    ├── cell_gng/fake_cloud_publisher.py # TEST-ONLY synthetic cloud
    ├── config/gng_params.yaml
    ├── config/pose_bridge.yaml
    ├── launch/gng.launch.py           # sensor-agnostic (use on real robot)
    └── launch/gng_sim.launch.py       # sim sandbox (gng + pose bridge + validation)
```

---

## What stays untouched

No existing file was modified. The real-hardware description
(`workcell_description`, `workcell_moveit_config`), the table driver
(`moving_table_pkg`), and every existing launch file are byte-for-byte
unchanged. The real-hardware top-level xacro still flattens and `check_urdf`-
validates. `git status` shows changes only under the two new package
directories.
