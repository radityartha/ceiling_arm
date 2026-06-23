# reachability_gng

Growing Neural Gas (GNG) **reachability + capability map** for the redundant
**arm + table** system, used to **seed MoveIt IK**. It is **arm-list-driven**:
one separate map is built per arm. `table_1` currently carries `arm_1` and
`arm_2` (each 8 DOF = 2 table + 6 arm joints); adding `arm_3`/`arm_4` later is a
one-line config addition. Target deliverable: Year-1 conference paper
(IEEE SMC / IAS / SII / AROB).

## Idea

Each GNG node holds a vector `[task | q]`:

- `task` = end-effector pose features (xyz, optionally + quaternion)
- `q`    = the 8-DOF config `[t1_linear, t1_rotation, t1_a1_joint_1..6]`

BMU search uses only the `task` dims (so nodes tile the reachable workspace),
while adaptation moves the whole vector (so each node's `q` becomes a
representative IK seed for its workspace cell). The table joints are **first-class
DOF**: the map shows how table motion *extends* reachability, and distinct table
placements for the same pose separate into different nodes — keeping seeds valid.

Node coloring in RViz encodes **manipulability** `w = sqrt(det(J Jᵀ))`
(Yoshikawa index): blue = low (near-singular / workspace edge), red = high
(dexterous). This is the "capability" layer on top of binary reachability.

## Environment / how to run

- **No Isaac Sim needed.** This whole package is kinematic: `data_gen`/`train`
  use Pinocchio FK; `seed_ik`/`eval ik` use MoveIt `/compute_ik`; `visualize`
  is plain RViz. Isaac is only for *executing* motions under physics (a later
  step), not for any result here.
- **Run from the repo root** — the config's `urdf:` path is repo-root-relative.
- **Make the module importable**, either by sourcing the colcon install:
  ```bash
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  ```
  or, for the ROS-free offline scripts, with `PYTHONPATH`:
  ```bash
  PYTHONPATH=ros2_ws/src/reachability_gng python3 -m reachability_gng.data_gen ...
  ```
- **Dependencies:** `data_gen` needs `pinocchio` (`pip install pin`); `train`
  needs only NumPy; the ROS nodes need a sourced ROS 2 Humble + this package
  built (`colcon build --packages-select reachability_gng --symlink-install`).

## Pipeline

```
data_gen.py   sample q -> FK (Pinocchio) -> [pose, q, manip]   ->  dataset.npz
train.py      build [task|q], GNG.fit, annotate hits+manip     ->  model.npz (+_stats)
seed_ik.py    PoseStamped -> GNG seed -> /compute_ik            ->  ik_solution
visualize.py  load model; nodes/edges                          ->  RViz MarkerArray
eval.py       volume comparison + IK benchmark                 ->  paper numbers
```

### 1. Flattened URDF (already present)
`data_gen` reads `workcell_description/urdf/workcell_full.urdf`. It already
exists with the table rail set to **0–3.0 m** and Gen3 Lite mesh paths rewritten
to `package://kortex_description/...`. Regenerating it with `xacro` requires the
`ros2_kortex` submodule populated (it currently isn't), so prefer editing the
flattened file directly for now.

### 2. Sample FK data + train (offline, no ROS) — recommended dense recipe

**One map per arm, in one command** — loops over the arm configs and writes
`/tmp/<name>_model.npz` (+ `_stats.npz`) for each. Adding `arm_3`/`arm_4` later
is a one-line addition to the `ARMS` array in the script (plus a config):
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2_ws/src/reachability_gng/build_maps.sh        # -> /tmp/arm1_model.npz, /tmp/arm2_model.npz
# override the dense recipe via env vars, e.g.:  N=20000 LAM=120 build_maps.sh
```

Or a single arm by hand (the script just loops this):
```bash
python3 -m reachability_gng.data_gen \
    --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml \
    --out /tmp/arm1_dataset.npz --n 80000
python3 -m reachability_gng.train --dataset /tmp/arm1_dataset.npz \
    --out /tmp/arm1_model.npz --task pos --max-nodes 3000 --lam 60 --epochs 2
# -> ~2668 nodes; node hull reaches close to true arm+table reach
```

**Node count is set by `lam`, not `max-nodes` alone:**
```
nodes ≈ min( max_nodes , n × epochs / lam )
```
e.g. `--max-nodes 1500` with default `--lam 200`, `n=50000`, `epochs=2` gives
only ~500 nodes (cap never reached, cloud looks short). To hit a target node
count `N`, set `lam ≈ n × epochs / N`.

#### Tuning: change sample count / node density
`build_maps.sh` takes the knobs as env vars (defaults `N=80000 LAM=60
MAX_NODES=3000 EPOCHS=2` → ~2668 nodes):
```bash
N=200000 ros2_ws/src/reachability_gng/build_maps.sh    # more FK samples (denser data, slower)
LAM=160  ros2_ws/src/reachability_gng/build_maps.sh    # ~1000 nodes (sparser cloud)
```
**Fewer nodes but same coverage:** keep `N` high (samples define the covered
area) and *raise* `LAM` — GNG still spreads the nodes across the whole sampled
region, so the cloud only gets sparser, not smaller. Pick `LAM ≈ N×EPOCHS /
(target nodes)`, e.g. `LAM=110`→~1450, `LAM=160`→~1000, `LAM=320`→~500.
Do **not** lower `N` to reduce nodes — that shrinks coverage and ragged-ifies the
boundary. After rebuilding, reload the clouds (`pkill -9 -f
lib/reachability_gng/visualize` then relaunch `gng_clouds.launch.py` /
`launch_workcell.sh`).

### 3a. Quick cloud-only look in RViz (no MoveIt)
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch reachability_gng view_gng.launch.py model_path:=/tmp/arm1_model.npz
# color_by:=hits to color nodes by reachability density instead of manipulability
```
Launches `robot_state_publisher` + `joint_state_publisher_gui` (drag the
sliders to pose the arm/table; the table rotation rests at its **90° home**) +
the GNG `MarkerArray` + RViz. The cloud is **static** — it's the union over
*all* configs (incl. the full 3 m rail and ±180° table rotation), so it does
not change with the sliders; the robot is just one pose inside it.

### 3b. Both clouds + MoveIt MotionPlanning in one RViz (fake hardware)
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch reachability_gng gng_moveit.launch.py     # uses /tmp/arm{1,2}_model.npz
```
Brings up `move_group` + a ros2_control **mock** joint source + the workcell
controllers on **fake hardware** (no real arms, no Isaac Sim, no LIDAR), one
`visualize` node **per arm** (arm_1 **green**, arm_2 **orange**, nodes colored
by manipulability), and RViz with the **MotionPlanning** plugin. Pick group
`table_1_with_arm_1` or `table_1_with_arm_2` to **Plan/Execute** while both
reachability clouds are shown. There is one RViz and one `/joint_states` source
(the broadcaster) — do **not** also start `joint_state_publisher_gui`.

The `robot_description` is the **flattened `workcell_full.urdf`** (not
`trailer_workcell.urdf.xacro`): the `ros2_kortex` submodule is incomplete here
so the xacro fails to expand, and the flattened urdf is the source of truth
(same model the GNG clouds use). For controller execution it carries `mock_components`
ros2_control with **unique** per-arm/gripper hardware names plus a `table_system`
block for the 4 table joints, so the fake controllers load and Plan/Execute
works without the xacro.

### 3c. Both clouds over the **Isaac Sim** digital twin
Plan/Execute against the Isaac-driven robot while the clouds are shown. One
command runs Isaac bridge + move_group (topic_based) + clouds + GNG RViz:
```bash
./isaac_sim/launch_workcell.sh         # default 'gng' mode: TABLE_1-ONLY view
```
The default **`gng`** mode uses a **table_1-only** model (`isaac_sim/workcell/ros/`
`table1_isaac.urdf` + `trailer_table1.srdf`, made by `make_table1_model.py`) and
hides the `t2_*` prims in Isaac, so **table_2/arm_3/arm_4 are gone in both Isaac
and RViz**. Use `./isaac_sim/launch_workcell.sh full` (or `demo`) for the original
4-arm cell. Build the maps first (`build_maps.sh` → `/tmp/arm{1,2}_model.npz`).
Clouds are in the `world` frame so they line up with the ceiling-mounted arms;
planning `table_1_with_arm_1`/`_2` executes split across the `arm_1`+`table_1`
controllers. (Everything shares `ROS_DOMAIN_ID=42` + `rmw_cyclonedds_cpp`.)
See **README.html** for the short quick-start.

### 4. GNG-seeded MoveIt IK (Phase 2)
Needs `move_group` running so `/compute_ik` exists (e.g. `gng_moveit.launch.py`
above), plus the per-arm SRDF group. `seed_ik` is generic — point it at an
arm via `model_path` / `group` / `ee_frame`:
```bash
# arm_1
ros2 run reachability_gng seed_ik --ros-args -p model_path:=/tmp/arm1_model.npz \
    -p group:=table_1_with_arm_1 -p ee_frame:=t1_a1_tool_frame -p use_gng_seed:=true
# arm_2 (same node, different params)
ros2 run reachability_gng seed_ik --ros-args -p model_path:=/tmp/arm2_model.npz \
    -p group:=table_1_with_arm_2 -p ee_frame:=t1_a2_tool_frame -p use_gng_seed:=true
# set use_gng_seed:=false for the no-seed baseline.
# then publish a target on /gng_seed_ik/target_pose (geometry_msgs/PoseStamped)
```
Logs IK success/solve-time, republishes the solution on `/gng_seed_ik/ik_solution`.
`build_ik_request` / `solve_ik` are reused by `eval.py`.

### 5. Evaluation (Phase 3 — the paper's numbers)
```bash
# offline reachable-volume comparison (table-locked vs table-active datasets)
python3 -m reachability_gng.eval volume --datasets locked.npz active.npz --res 0.05
# IK benchmark (needs move_group)
python3 -m reachability_gng.eval ik --model /tmp/model.npz --dataset /tmp/dataset.npz \
    --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml \
    --methods gng none random --n 500 --csv results.csv
```
`--config` enables the KDL random-restart baseline and manipulability scoring.

## Status

**Phase 1 (done):** GNG core (`gng.py`, unit-tested; incremental adjacency
index so it scales to thousands of nodes), FK sampler (`data_gen.py`), trainer
(`train.py`), seed lookup (`seed_server.py`), RViz viz (`visualize.py`).

**Phase 2 (done):** GNG-seeded MoveIt IK via `/compute_ik` (`seed_ik.py`,
`use_gng_seed` A/B switch). SRDF group `table_1_with_arm_1` (table_1 + arm_1,
8 DOF) and its KDL solver entry added in `workcell_moveit_config`.

**Phase 3 (done):** `eval.py` — `volume` (offline arm-only vs arm+table reach
gain) and `ik` (GNG seed vs no-seed vs KDL random-restart: success rate, solve
time, manipulability).

Remaining for the paper: a Zacharias-style voxel-capability-map baseline, the
table-aware node-separation ablation as a flag, and plotting scripts.

## Notes / gotchas

- **Cloud is config-independent.** It's the whole reachable workspace; the
  slider pose only moves the robot. At `t1_linear=0` the arm alone reaches a
  ~0.7 m sphere (X up to ~+0.4); the cloud extends to X≈+3.6 only because other
  samples slid the table out. That is correct.
- **GNG leaves a small boundary gap.** Nodes settle at Voronoi-cell centroids,
  so the node hull sits ~one half-cell inside the true reachable surface
  (~0.1–0.25 m). For an exact reachability *boundary* figure, use an
  alpha-shape/voxel over the raw FK samples — worth contrasting in the paper.
- **Joint limits** in `config/arm1_table1.yaml` match the URDF (verified). Table
  rail is sampled 0–3.0 m.

## Test
```bash
cd ros2_ws/src/reachability_gng && python3 -m pytest test/ -q
```
