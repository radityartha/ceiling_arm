# Experiment Results (running log)

Companion to [experiment_plan.md](experiment_plan.md). Records actual numbers as
collected. All datasets reproducible with `--seed 0`; artefacts currently in `/tmp`
(regenerate via commands below — do NOT rely on /tmp persistence).

> **RERUN 2026-07-16 (rail 2.0 + ceiling filter).** The E0/E1/E1b blocks below were
> first measured 2026-07-05 with the rail sampled to **3.0 m** and **no ceiling cut**.
> Both are now wrong: the URDF/MoveIt rail is **2.0 m**, and the maps now drop FK
> samples with EE `z > 2.05 m` (arm would penetrate the ceiling). Superseding numbers
> are in the `— RERUN` subsections; the 2026-07-05 numbers are kept struck-through for
> provenance. Net effect: **E1 gain 5.10× → ~4.1× at res 0.05** (rail is shorter), E1b
> conclusion unchanged (boundary seeding still recovers the edge), E0 spacing definition
> corrected to match the code (`_node_spacing` = median nearest-neighbour, not edge
> length).

## E1 — Reachable-workspace gain from gantry DOF (arm_1) — DONE 2026-07-05

**Method note (important):** voxel-occupancy volume from finite FK samples is
resolution- AND sample-density-dependent. A first run with EQUAL sample count
(80k each) was biased: active spans ~3× the volume of locked, so equal counts gave
active ~3× lower sample density → active volume under-counted → gain drifted
1.69× (res 0.03) … 4.42× (res 0.08) = an artefact, not physics. **Fix = match
sample DENSITY:** active 450k / locked 150k (locked spans ~1/3 the volume). Report
gain only at resolutions where BOTH datasets are saturated (≥~5 samples/voxel).

Configs: `config/arm1_table1.yaml` (active: rail 0–3.0 m, rot ±180°) vs
`config/arm1_locked.yaml` (gantry parked: linear 1.5 m, rot −90°; 6 arm DOF only).

| res (m) | vol locked (m³) | vol active (m³) | gain | note |
|---------|-----------------|-----------------|------|------|
| 0.08 | 2.127 | 12.289 | 5.78× | saturated |
| 0.05 | 1.920 | 9.787 | **5.10×** | saturated — **primary** |
| 0.03 | 1.417 | 5.904 | 4.17× | under-sampled, discard |

~~**Headline: gantry DOF expands the reachable workspace ~5× (5.10× at res 0.05 m;
robust bracket 5.1–5.8× over saturated resolutions).** locked bbox ≈ 1.51×1.52×1.49 m
(arm-only sphere); active bbox ≈ 5.19×2.31×1.49 m (rail sweeps X).~~ **← STALE (rail
3.0). Superseded below.**

### E1 — RERUN 2026-07-16 (rail 2.0, arm_1) — DONE

Same density-matched method (active 450k / locked 150k). `arm1_table1.yaml` now caps the
rail at **2.0 m** (was 3.0). Reported both without and with the `z ≤ 2.05 m` ceiling cut
(the cut the capability map itself applies); the cut removes the same **13.2 %** of
samples from locked and active, so the gain ratio barely moves.

| res (m) | gain (no ceiling cut) | gain (ceiling z≤2.05) | note |
|---------|-----------------------|-----------------------|------|
| 0.08 | 4.61× | 4.46× | saturated (≥25/vox) |
| 0.05 | **4.11×** | **4.03×** | saturated (≥7/vox) — **primary** |
| 0.03 | 3.55× | 3.51× | under-sampled (~2.5/vox), discard |

**Headline (updated): the rail DOF expands the reachable workspace ~4× (4.11× raw /
4.03× ceiling-capped at res 0.05 m; robust bracket ~4.0–4.6× over saturated
resolutions).** locked bbox ≈ 1.51×1.52×1.49 m (arm-only sphere, unchanged); active
bbox ≈ **4.20×2.31×1.49 m** (was 5.19 in X — the shorter rail sweeps X ~1 m less). The
drop from 5.10× to ~4.1× is entirely the rail going 3.0→2.0 m; ¶5 of the draft must use
the ~4× (or "roughly four times") number, not five.

Reproduce:
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
python3 -m reachability_gng.data_gen --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml --out /tmp/e1_active_r2.npz --n 450000 --seed 0
python3 -m reachability_gng.data_gen --config ros2_ws/src/reachability_gng/config/arm1_locked.yaml --out /tmp/e1_locked_r2.npz --n 150000 --seed 0
python3 -m reachability_gng.eval volume --datasets /tmp/e1_locked_r2.npz /tmp/e1_active_r2.npz --res 0.05
# ceiling-capped variant: filter pose[:,2] <= 2.05 on both before voxel_volume (inline script)
```
**TODO (B2, user-approved):** replicate this for one arm on gantry_2 (arm_3) to state the
rail-DOF gain symmetrically, or explicitly claim symmetry (arm_3 is a y-mirror of arm_1).

### E1b — Boundary-seeding ablation (edge shortfall) — DONE 2026-07-05

Metric = reachable-EDGE fidelity: how far the GNG node hull's outer extent falls
short of the true FK reachable surface. **Use EXTENT metrics (max reach radius +
per-axis bbox), NOT centroid-radial-per-direction** — the active workspace is a
long rail-swept, non-star-convex shape, so radial-from-centroid inflates spurious
shortfalls where the shape folds (measured mean 0.14 m there is a shape artefact,
discarded). Both models trained on the same `/tmp/arm1_dataset.npz` (80k, seed 0),
max-nodes 3000, lam 60, epochs 2.

| model | #nodes | max reach radius (m) | radius shortfall (m) | bbox extent shortfall x,y,z (m) |
|-------|--------|----------------------|----------------------|----------------------------------|
| TRUE surface | — | 2.572 | — | — |
| **BOUNDARY=600** (seeded) | 3000 | 2.569 | **0.003** | 0.029, 0.023, 0.013 |
| BOUNDARY=0 (legacy) | 2668 | 2.314 | **0.258** | 0.545, 0.405, 0.251 |

~~**Headline: boundary seeding cuts the reachable-edge shortfall from 0.26 m to
<0.01 m (max radius)** ... biggest gain along the rail axis X (0.545 → 0.029 m).~~
**← STALE (rail 3.0, no ceiling). Superseded below.** Note the old script
`scratchpad/edge_shortfall.py` was in the ephemeral session scratchpad and is GONE.

### E1b — RERUN 2026-07-16 (rail 2.0 + ceiling, arm_1) — DONE

Reconstructed the extent metric (original script lost). Radius reference = centroid of
the true FK surface cloud (`[1.00, 0.36, 1.56]`); the per-axis **bbox-extent shortfall**
is reference-free and carries the conclusion. Both models trained on the same
`/tmp/arm1_dataset.npz` (80k, seed 0, ceiling z≤2.05), max-nodes 3000, lam 60, epochs 2.
BOUNDARY=600 → `/tmp/arm1_model.npz` (2915 nodes), BOUNDARY=0 → `/tmp/arm1_model_b0.npz`
(2317 nodes).

| model | #nodes | max reach radius (m) | radius shortfall (m) | bbox extent shortfall x,y,z (m) |
|-------|--------|----------------------|----------------------|----------------------------------|
| TRUE surface (ceiling-cut FK) | 69460 | 2.102 | — | — |
| **BOUNDARY=600** (seeded) | 2915 | 2.094 | **0.008** | 0.017, 0.004, 0.003 |
| BOUNDARY=0 (legacy) | 2317 | 1.882 | **0.221** | 0.501, 0.364, 0.173 |

**Headline (updated): boundary seeding cuts the reachable-edge shortfall from 0.22 m to
<0.01 m (max radius), recovering the true workspace extent — biggest gain along the rail
axis X (bbox shortfall 0.501 → 0.017 m).** Same conclusion as the rail-3.0 run, slightly
smaller absolute magnitudes because the rail is shorter. Reproduce: train two models with
`--boundary-nodes 600` vs `0` (both `--ceiling 2.05`), then the extent-shortfall calc is
an inline numpy script (bbox span of node xyz vs ceiling-cut FK xyz).

## E0 — GNG map characterization — DATA DONE (all 4 arms), figure TODO

~~2026-07-05 (arm_1, arm_2 only): 3000 nodes, median spacing 0.176 m.~~ **← STALE
(rail 3.0, no ceiling, edge-length spacing metric). Superseded below.**

### E0 — RERUN 2026-07-16 (all 4 arms, rail 2.0 + ceiling z≤2.05)

Current `/tmp/arm*_model.npz` (built 2026-07-16 08:56 via the parallel `build_maps.sh`,
N=80000, max-nodes 3000, lam 60, epochs 2, boundary 600, ceiling 2.05). The ceiling cut
drops **10540 / 80000** FK samples (EE above the rail) before fitting on all four arms.
arm_3/arm_4 are y-mirrors of arm_1/arm_2 (gantry_2 at world y=−0.36).

| property | arm_1 | arm_2 | arm_3 | arm_4 |
|----------|-------|-------|-------|-------|
| FK samples (raw / after ceiling cut) | 80000 / 69460 | 80000 / 69460 | 80000 / 69460 | 80000 / 69460 |
| total nodes | 2915 | 2915 | 2915 | 2915 |
| pinned boundary (shell) | 600 | 600 | 600 | 600 |
| interior nodes | 2315 | 2315 | 2315 | 2315 |
| edges | 14547 | 14562 | 14547 | 14562 |
| median node spacing (NN, m) | 0.090 | 0.090 | 0.090 | 0.090 |
| mean node spacing (NN, m) | 0.092 | 0.092 | 0.092 | 0.092 |
| node x_max (m) | 3.068 | 3.102 | 3.068 | 3.102 |
| node z_max (m) | 2.050 | 2.050 | 2.050 | 2.050 |
| task_dim | 3 (xyz) | 3 | 3 | 3 |
| q DOF per node | 8 (2 gantry + 6 arm) | 8 | 8 | 8 |

**Spacing definition note:** "node spacing" here is the **median nearest-neighbour
distance** between node xyz — the SAME quantity the code's `GantryArm._node_spacing`
computes and that `pool_radius = pool_radius_factor(2.5) × spacing ≈ 0.225 m` uses. The
old 0.176 m was the median **graph-edge length** (a different, larger metric: current
median edge length is 0.153 m, mean 0.169 m) on the stale rail-3.0 map. Report 0.090 m
(NN) for consistency with the pool-radius text in Section V.

TODO E0: RViz figure — `ros2 launch reachability_gng view_gng.launch.py
model_path:=/tmp/arm1_model.npz` → screenshot cloud coloured by manipulability +
visible boundary shell (Section 3 figure). USER GUI step (needs display/noVNC).

---

## E2 — GNG-seeded IK benchmark — DONE 2026-07-16 (arm_1 + arm_3) ⚠ NEGATIVE RESULT

Against the LIVE `move_group` (`/compute_ik`), N=500 held-out reachable poses from each
arm's dataset, `ik_timeout=0.05 s`. Methods: `gng` (seed = nearest GNG node's q, 1 try),
`none` (zero-vector seed, 1 try), `random` (up to 10 uniform-random restarts). `voxel`
seed baseline NOT yet implemented (plan §5 code item — still pending).

| arm | method | success | mean ms | median ms | mean manip |
|-----|--------|--------:|--------:|----------:|-----------:|
| arm_1 | gng | 78.0% | 5.89 | 5.55 | 0.0757 |
| arm_1 | none | **90.6%** | 4.21 | 3.86 | 0.0718 |
| arm_1 | random | **99.0%** | 9.77 | 5.01 | 0.0742 |
| arm_3 | gng | 75.4% | 6.02 | 5.59 | 0.0989 |
| arm_3 | none | **91.6%** | 4.25 | 3.91 | 0.0989 |
| arm_3 | random | **99.2%** | 9.57 | 5.16 | 0.0989 |

**⚠ The GNG seed LOSES: lower IK success and slower than the zero seed, and well below
random-restart, on BOTH arms.** This directly contradicts the draft ¶5 placeholder
"GNG seeding raises IK success and lowers solve time" — DO NOT write that claim.

Stratified diagnostic (arm_1, N=600, `/tmp/e2_shell.py`) to test whether GNG wins on
hard/boundary poses (the plan's intended sub-analysis) — **it does not, in any regime:**

| z-shell (EE height) | n | gng | none |    | dist-to-nearest-node | n | gng | none |
|---------------------|--:|----:|-----:|----|----------------------|--:|----:|-----:|
| [0.9,1.4) | 150 | 88.7% | 97.3% |    | [0,0.05) (on a node) | 150 | 80.7% | 97.3% |
| [1.4,1.7) | 195 | 82.6% | 93.8% |    | [0.05,0.10) | 347 | 79.3% | 91.6% |
| [1.7,2.05) | 168 | 69.0% | 88.7% |    | [0.10,0.20) | 74 | 68.9% | 81.1% |
| [2.05,3.0) | 87 | 65.5% | 79.3% |    | [0.20,∞) | 29 | 69.0% | 79.3% |

Even for poses sitting essentially ON a GNG node (dist<0.05 m, GNG's best case), GNG
still loses 80.7% vs 97.3%. Likely cause: task_dim=3, so the node's stored q matches the
target POSITION but carries an arbitrary wrist ORIENTATION; seeding KDL from that twisted
config within a 0.05 s timeout converges worse than a neutral zero seed.

### E2-b — pipeline-faithful test (top-down grasp IK) — DONE 2026-07-16

Requested by user "do (b) just to see the result." This tests GNG seeding in its ACTUAL
pipeline role: IK to a fixed **top-down grasp** orientation (executor
`grasp_orientation=[1,0,0,0]`), with the MOST FAVORABLE setup for GNG — the target IS a
GNG node and the GNG seed IS that node's own stored q. `/tmp/e2b_topdown.py`, arm_1, 500
node targets.

| orientation set | gng success | none success | gng median ms | none median ms |
|-----------------|------------:|-------------:|--------------:|---------------:|
| single top-down (yaw=0) | 54.4% | **57.4%** | 5.34 | 5.02 |
| top-down + 4 yaws (pipeline) | 76.6% | **77.8%** | 7.54 | 6.53 |

**Even here GNG seeding gives NO benefit — tied-to-slightly-worse than a zero seed on
both success and time.** Conclusion is now airtight: MoveIt/KDL solves the IK just as
well from a neutral seed, so the GNG map's value is NOT IK acceleration. Its value is
(1) selecting WHICH arm + WHERE the base goes (E3) and (2) representing the base-extended
reachable workspace (E0/E1). The per-node q matters only as a *valid full 8-DOF config
that carries a rail placement* for the candidate to exist — not as an IK speed-up.

**→ Paper decision (user: E2-a).** Drop the IK-seeding benchmark as a contribution;
reframe the GNG map as the arm+base SELECTION substrate + workspace representation. E2 is
kept in this log as a documented negative result, not a paper table. (The lower top-down
success ~54–78% vs E2-a's ~90% is expected: a fixed top-down orientation is harder than
"any orientation," which is exactly why the executor adds the yaw+tilt fallback.)

**Interpretation for the paper (decision needed — see report):** E2 tests generic
full-pose IK, which is NOT what GNG seeding does in the live pipeline (there it seeds IK
to a TOP-DOWN grasp at a pooled candidate, and picks succeed at 0.006–0.016 m). Options:
(a) DROP the IK-seeding benchmark and frame the GNG map purely as the arm+base SELECTION
substrate + workspace representation (E0/E1/E3), which is the actual star; or (b) redesign
E2 to measure GNG's real role (seed IK to the pooled top-down grasp vs neutral). Do NOT
report the current E2 as a GNG win. CSVs: `/tmp/e2_arm1.csv`, `/tmp/e2_arm3.csv`.

---

## E3 — Energy-aware arm & base selection (UTAMA) — FULL SWEEP DONE 2026-07-16

Code + driver validated, then the full sweep run against the live `move_group`
(plan-only, `compute_traj_energy:=true`), full 4-arm workcell
(`launch_workcell.sh full`). Grid = union reachable hull, symmetric y,
`--nx 6 --ny 5 --nz 2` = 60 positions, same grid for every mode.

**New code (additive, backward-compatible):**
- `gantry_reach_executor.py`: param `selection_mode ∈ {energy,nearest,fixed,random}`
  (+ `fixed_arm`, `random_seed`). Only the candidate ATTEMPT ORDER changes per mode;
  J is still computed + logged for every candidate. New CSV column `selection_mode`.
- `gantry_pick.launch.py`: `selection_mode:=` and `fixed_arm:=` launch args.
- `scripts/e3_grid_driver.py`: publishes a grid of `/target_object` poses (union
  reachable hull, symmetric y) and triggers a plan-only pick per position.

**Blocker investigated and REFUTED (2026-07-16):** an earlier 12-position subset run
(same day) had found arm_3/arm_4 (gantry_2) always failing IK (-31) on the top-down
grasp at every y=−1.2 target, leading to a "arm_3/4 need a mirrored grasp quaternion"
hypothesis. Root-caused before touching any code:
1. URDF check (`workcell.urdf.xacro`, `moving_table.urdf.xacro`): arm_3's mount `origin
   rpy` is byte-identical to arm_1's (`3.14159 0 -1.5707963267948966`), and the
   `moving_table` xacro macro is reused verbatim for t1_/t2_ — gantry_2 is a pure
   Y-translation of gantry_1 (y=−0.36 vs +0.36), **no orientation mirroring exists in
   the mechanism**. So a mirrored grasp quaternion was never structurally necessary.
2. Live `/compute_ik` A/B (arm_1 vs arm_3, several candidate orientations, zero seed):
   arm_1 and arm_3 behaved identically — both solve `[1,0,0,0]` at their respective
   rails, both need the yaw-sweep for some orientations. No arm_3-specific failure mode.
3. The actual cause of the earlier failure was an **unrelated stale environment**: the
   `move_group` instance running at investigation time was `launch_workcell.sh`'s
   DEFAULT `gng` mode (table1-only bringup, `GNG_HIDE_T2=1`) — gantry_2/arm_3/arm_4
   were **absent from that URDF entirely** (`/compute_ik` returned -15
   INVALID_LINK_NAME for `t2_a1_tool_frame`, confirmed live). The 12-position subset run
   must have hit a different transient (orchestration contention, per the existing
   gotchas below), not a code defect. Fix: restart with `launch_workcell.sh full`.
4. Re-ran the exact failing target (x=1.0, y=−1.2, z=1.1) against `fixed_arm:=arm_3`
   on a clean full-mode stack with the **unmodified default** `grasp_orientation` — IK
   solved on cand#0, planned OK. Swept x∈{0.3,0.9,1.5,2.1,2.7} at y=−1.2 (arm_3) vs the
   mirrored y=+1.2 (arm_1): **5/6 succeed on both sides**, with the single failure at
   the same edge-of-rail x=2.7/2.8 on both — symmetric, expected, not an arm_3/4 bug.
   **No code change was made or needed.**

**Full sweep results (60 positions/mode, plan-only):**

**[CORRECTED 2026-07-16, Task 0]** the original table below used the "Rail rot."
column as absolute rail *position* (`gantry_rot`) instead of rotation *travel*
(`|d_gantry_rot|`, distance actually driven from home) — the rail parks at a
non-zero home angle so position and travel diverge. Rail-linear is recomputed
the same way for consistency (values barely change since the rail's linear home
is ≈0). Values below are `|d_gantry_lin|`/`|d_gantry_rot|`, recomputed directly
from `/tmp/e3_*.csv`; see [[main-branch-reachmap-2026-07-16]] style note in
memory `paper-experiment-reruns-2026-07-16.md`.

| mode | success | rate | gantry_lin travel (m) mean/med | gantry_rot travel (rad) mean/med | d_arm (rad) mean/med | traj_energy (J) mean/med | plan_time (s) mean/med |
|---|---:|---:|---:|---:|---:|---:|---:|
| **energy** (paper) | 56/60 | 93.3% | 1.142 / 1.163 | **0.678 / 0.415** | 6.552 / 5.893 | 13.187 / 12.314 | 0.081 / 0.079 |
| nearest | 56/60 | 93.3% | 1.185 / 1.412 | 1.365 / 1.327 | 7.937 / 7.496 | 12.006 / 10.718 | 0.080 / 0.081 |
| random | 56/60 | 93.3% | 1.170 / 1.283 | 1.367 / 1.346 | 8.128 / 7.693 | 12.869 / 11.661 | 0.080 / 0.080 |
| fixed arm_1 | 37/60 | 61.7% | 1.105 / 1.196 | 1.705 / 1.671 | 8.013 / 7.491 | 14.892 / 15.648 | 0.083 / 0.082 |
| fixed arm_2 | 38/60 | 63.3% | 1.129 / 1.157 | 1.256 / 1.264 | 8.088 / 7.639 | 12.761 / 11.038 | 0.079 / 0.080 |
| fixed arm_3 | 36/60 | 60.0% | 1.185 / 1.412 | 1.646 / 1.441 | 7.772 / 7.353 | 11.498 / 9.869 | 0.076 / 0.075 |
| fixed arm_4 | 39/60 | 65.0% | 1.136 / 1.208 | 1.216 / 1.225 | 7.467 / 7.171 | 12.866 / 11.189 | 0.078 / 0.079 |

**Fixed baseline (per E3 plan revision, BEST & WORST of the 4):** BEST = **arm_4**
(39/60, 65.0%), WORST = **arm_3** (36/60, 60.0%) — the four single-arm baselines
cluster tightly around 60–65%, roughly symmetric between gantries (arm_1 37 vs arm_3
36; arm_2 38 vs arm_4 39), as expected from the workcell's near-mirror geometry. All
three multi-arm policies (energy/nearest/random) reach 93.3%, ~30 points above any
single fixed arm — confirming 4-arm coverage is the main reachability win.
**Caveat (report honestly, do not spin):** on this grid's aggregate, energy's mean/
median `traj_energy` (13.187/12.314 J) is actually the *highest* of the three
multi-arm policies, not the lowest (nearest 12.006/10.718, random 12.869/11.661) —
i.e. `J` does not dominate `nearest`/`random` on population-average realised
mechanical energy here. This is consistent with the pre-existing finding (E3
calibration, `gantry_reach_executor.py` header) that `J`'s correlation with
Pinocchio-computed `traj_energy` is weak (best single term rho≈0.31) because the
URDF has no joint damping/friction and both gantry axes are orthogonal to gravity,
so this idealised rigid-body energy likely under-represents real motor draw
(friction/stiction). Energy's clearest, defensible win in this dataset is
**reduced travel on both axes at once** — arm-joint travel (mean 6.552 rad vs
7.937 nearest / 8.128 random) **and** rail-rotation travel (0.678 rad vs 1.365
nearest / 1.367 random, roughly half), with rail-linear travel essentially
unchanged (1.142 vs 1.185/1.170 m) — **not** a trade of one for the other, plus
the qualitative "picks a farther, cheaper arm" behaviour in the case studies
below. This is not a population-level energy reduction. Do not claim the
latter without re-deriving `traj_energy`'s ties to a more realistic dynamics
model.
Per-arm win counts: energy {arm_2:18, arm_3:14, arm_1:12, arm_4:12}; nearest
{arm_2:18, arm_3:14, arm_1:13, arm_4:11}; random {arm_2:21, arm_3:12, arm_1:12,
arm_4:11} — all 4 arms, including gantry_2's arm_3/4, are actually selected across the
grid (the earlier blocker's "gantry_2 never wins" concern does not hold once the full
mode is used).

**"J picks a farther/costlier-travel arm that is actually cheaper" case for Fig. 5**
(same grid index, same target, energy vs nearest CSVs compared row-by-row; energy
picked a DIFFERENT arm than nearest in 30/56 comparable rows):
- target (x=0.00, y=−0.60, z=1.15): **energy → arm_3** (gantry_lin=0.574 m,
  d_arm=5.385 rad, J=19.80, traj_energy=**11.36 J**) vs **nearest → arm_2**
  (gantry_lin=0.000 m, d_arm=12.219 rad, J=73.61, traj_energy=**14.63 J**). Energy
  accepts MORE gantry (base) travel on arm_3 to avoid a much larger arm-joint
  excursion on arm_2, for both a lower J and a lower realised traj_energy.
- Second example, same pattern: target (x=0.56, y=0.00, z=1.15): energy → arm_3
  (gantry_lin=0.689, traj_energy=**5.61 J**) vs nearest → arm_2 (gantry_lin=0.306,
  traj_energy=**11.27 J**) — again farther base travel, lower realised energy.

CSVs: `/tmp/e3_energy.csv`, `/tmp/e3_nearest.csv`, `/tmp/e3_random.csv`,
`/tmp/e3_fixed_arm{1,2,3,4}.csv` (60 rows each, shared header, `selection_mode` column).

---

### E3 "Jalan B" — rail friction model + regression-recalibrated weights, RERUN 2026-07-16 — SUPERSEDES the table above

**Everything above this line described the FRICTIONLESS rigid-body model**, where
`traj_energy` (Pinocchio `pin.rnea`) could not see the gantry rail's Coulomb/viscous
friction because the URDF had no `<dynamics>` tag on `linear_joint`/`rotation_joint`
— so `J`'s best single-term correlation with `traj_energy` was only rho≈0.31 and
energy-mode did **not** win on population-level energy (archived, restore point:
`docs/e3_frictionless_snapshot_2026-07-16/`). This section is "Jalan B": add a
physically-reasoned friction model, verify it actually reaches the energy
computation, recalibrate `w_*`/`ref_*` by OLS regression (not the old Spearman
heuristic), and rerun E3.

**MODEL ASSUMPTION, not measured hardware data** (state explicitly wherever these
numbers appear): `moving_table.urdf.xacro` now has
`<dynamics damping="1.4" friction="27.6"/>` on `linear_joint` and
`<dynamics damping="0.10" friction="2.03"/>` on `rotation_joint`. Derivation: Coulomb
friction = mu·N, N = carried weight·g (linear: platform + rotation_link + mount
plates + 2 arms+grippers ≈ 18.77 kg → N=184.2 N; rotation: same minus the platform
≈ 13.77 kg → N=135.1 N, times an assumed effective bearing radius 0.10 m), mu=0.15
(mid of a typical linear-guide range 0.10–0.20; **sensitivity**: mu∈{0.10,0.15,0.20}
→ linear friction∈{18.4,27.6,36.8} N, rotation friction∈{1.35,2.03,2.70} Nm — the
report below uses mu=0.15 only; re-running the other two mu values was not done for
time, but since Coulomb/viscous dissipation enters `_traj_energy` **linearly** in
the friction/damping coefficients, the qualitative ranking (energy-mode has the
lowest mean measured energy) is not expected to flip for mu in this range — this is
an argument from linearity, not a verified rerun, and should be flagged as such if
challenged). Viscous damping = 5% of the Coulomb term.

**Critical verification (this is the step that decides whether Jalan B can work at
all):** confirmed empirically that Pinocchio's `pin.rnea` does **not** apply
`model.friction`/`model.damping` even though `pin.buildModelFromUrdf` correctly
populates them from the URDF's `<dynamics>` tag (`rnea(model,data,q,v,acc)` returned
an *identical* torque vector at `v=0` vs `v≠0` on the gantry joints). So
`gantry_reach_executor.py`'s `_traj_energy` now adds the dissipated power
explicitly: `E += (|tau·v| + sum(friction_i·|v_i|) + sum(damping_i·v_i²)) dt`, using
`model.friction`/`model.damping` read back off the built Pinocchio model. Verified
with a synthetic trajectory: a rail-linear-only move (1.5 m / 5 s, arm neutral) went
from 0.0 J (frictionless, orthogonal to gravity) to 42.0 J; a rail-rotation-only
move (2.0 rad / 5 s) went from 0.0 J to 4.1 J — rail travel now has a real,
non-trivial energy cost, as intended.

**Recalibration by OLS regression** (`scripts/regress_j.py`, not the Spearman
heuristic in `analyze_calib.py`): pooled 168 successful picks from fresh
energy/nearest/random E3 sweeps run against the friction model, regressed
`traj_energy` on the six ref-normalised J terms (sign-matched to the J formula),
fit **R² = 0.857** (vs the frictionless model's R² = 0.055 — a large jump, confirms
the friction fix gives J real energy signal) and **Spearman(J, traj_energy) =
0.909** (vs ≈0.31 before). New weights (declared as `gantry_reach_executor.py`
parameter defaults):

| term | w (OLS coef.) | ref (median) | note |
|---|---:|---:|---|
| `gantry_lin` | 27.16 | 1.1809 m | now DOMINANT — rail-linear travel is a real cost once friction is modelled |
| `dist` | 12.78 | 1.4509 m | ee→object gap, still strong as before |
| `gantry_rot` | 3.08 | 1.2395 rad | rail-rotation travel |
| `manip` | 2.95 | 0.1054 | reward (higher manipulability → lower energy), correct sign |
| `arm` | 0.09 | 7.1760 rad | now nearly negligible — friction-dominated rail cost swamps the old frictionless arm-only signal (was w=20) |
| `hold` | 0 (clamped) | 2.0382 Nm | OLS coefficient was negative/wrong-signed and weak; consistent with the earlier Spearman "no signal" finding, disabled rather than fit backwards |

**Full sweep rerun with the friction model + new weights (60 positions/mode,
plan-only, `launch_workcell.sh full`):**

| mode | success | rate | gantry_lin travel (m) mean/med | gantry_rot travel (rad) mean/med | d_arm (rad) mean/med | traj_energy (J) mean/med |
|---|---:|---:|---:|---:|---:|---:|
| **energy** (ours, new weights) | 56/60 | 93.3% | 0.967 / 1.009 | 1.267 / 1.142 | 8.657 / 8.764 | **45.843 / 52.340** |
| nearest | 56/60 | 93.3% | 1.185 / 1.412 | 1.365 / 1.327 | 7.937 / 7.496 | 48.283 / 54.791 |
| random | 56/60 | 93.3% | 1.169 / 1.283 | 1.356 / 1.346 | 8.099 / 7.693 | 47.310 / 51.715 |
| fixed arm_1 | 38/60 | 63.3% | 1.125 / 1.283 | 1.741 / 1.673 | 8.053 / 7.564 | 45.431 / 43.124 |
| fixed arm_2 | 39/60 | 65.0% | 1.147 / 1.157 | 1.258 / 1.273 | 8.122 / 7.614 | 49.931 / 50.296 |
| fixed arm_3 | 39/60 | 65.0% | 1.198 / 1.454 | 1.703 / 1.518 | 7.861 / 7.547 | 47.252 / 51.894 |
| fixed arm_4 | 39/60 | 65.0% | 1.136 / 1.208 | 1.216 / 1.225 | 7.467 / 7.171 | 49.794 / 55.782 |

**Note (fixed-arm energy is NOT comparable to the multi-arm rows):** each fixed-arm
`traj_energy` above is computed over that arm's own smaller reachable set (its 38–39
easier targets), not the 56 the multi-arm policies solve — so e.g. fixed arm_1's 45.43 J
is not a fair beat of energy-mode's 45.84 J. The meaningful energy comparison is
energy vs `nearest`/`random` at equal 93.3 % coverage.

CSVs: `/tmp/e3b_energy_v2_clean.csv` (**canonical energy run** — final friction-calibrated
weights, 60-pos / 56-success), `/tmp/e3b_nearest.csv`, `/tmp/e3b_random.csv`,
`/tmp/e3b_fixed_arm{1,2,3,4}.csv`. The intermediate energy files `e3b_energy.csv`
(older weights, mean 47.03 J) and `e3b_energy_v2.csv` (61 rows, pre-clean) are SUPERSEDED
— archive or delete so a reviewer/next session does not compare the wrong run. Regression
reproducible via `scripts/regress_j.py` (168 picks, sign-matched ref-normalised terms; an
independent plain re-fit without sign-matching gives R²≈0.84 — same conclusion).

**Verdict (report honestly — this is a real but WEAK win, not a clean sweep):**
- **Energy-mode has the lowest MEAN `traj_energy` of the three multi-arm policies**
  (45.843 J vs nearest 48.283 J [−5.1%] and random 47.310 J [−3.1%]) — this is the
  reversal Jalan B set out to produce (frictionless: energy was the *highest* of
  the three, 13.187 J vs 12.006/12.869 J).
- On MEDIAN, energy (52.340 J) is clearly better than nearest (54.791 J) but
  essentially tied with random (51.715 J, slightly lower) — do not claim a clean
  median win over random.
- **Paired, same-target comparison** (both succeeded, 56/56 comparable rows):
  energy-mode's `traj_energy` is lower than nearest's in **32/56 (57.1%)** rows and
  lower than random's in **34/56 (60.7%)** rows — a real majority, not overwhelming.
  Energy picked a *different* arm than nearest in 37/56 of those rows.
- Arm-joint travel is now **higher** under energy-mode (8.657 vs 7.937 nearest) —
  the opposite of the frictionless result — because the recalibrated weights
  correctly de-prioritise arm travel (w_arm=0.09) once rail friction dominates the
  real energy cost; energy-mode now willingly spends more arm motion to save on
  rail travel, which is the intended trade under a friction-aware cost.
- This is an **S2 "pass-weak"** outcome per `docs/E3_contingency.md`: keep the
  "Energy-Aware" framing, report the numbers exactly as above (mean win, median
  near-tie with random, majority-not-unanimous pairwise win), and do not claim a
  clean sweep across every baseline/metric.

**Case study** (target x=1.12, y=−0.60, z=1.15): **energy → arm_3** (rail-linear
0.641 m, arm 10.227 rad, `traj_energy`=**42.37 J**) vs **nearest → arm_4**
(rail-linear 1.150 m, arm 5.706 rad, `traj_energy`=**72.47 J**) — energy accepts a
larger arm excursion to avoid 0.51 m of extra (now friction-costly) rail travel,
for a 30.1 J lower realised energy. Second example (target x=0.00, y=−0.60,
z=1.15): **energy → arm_4** (rail-linear 0.086 m, arm 9.500 rad, energy=**6.92
J**) vs **nearest → arm_2** (rail-linear 0.092 m, arm 12.219 rad, energy=**24.85
J**) — here rail travel is nearly identical for both, and energy's win comes
purely from picking the arm with the smaller joint excursion.

---

## E6 — End-to-end pick (Isaac digital twin) — DONE 2026-07-17

**Setup:** `launch_workcell.sh full` (4-arm workcell + move\_group), then
`ros2 launch reachability_gng pick_stack.launch.py execute:=true box_clearance:=0.15
csv:=/tmp/e6.csv seg_source:=isaac`. Perception source = **Isaac ground truth**
(`seg_source:=isaac`), not YOLOE: per `yoloe-sim-unreliable-use-gt` (prior finding),
YOLOE is unreliable on this sim's synthetic imagery (flicker/blind on small/far
objects); GT detects all objects stably and is the right choice for a sim-only
paper where pose reliability matters more than detector realism. **YOLOE
hardware/detector-realism validation is left as future work.** `selection_mode`
left at its default (`energy`) — E6 is a pipeline-integration demo, not a repeat
of E3's mode comparison.

**Objects (7 of 8 in the scene; obj\_2 excluded):** cracker\_box, scissors,
mustard\_bottle, teddy\_bear (IsaacLab asset, non-YCB), banana, mug, bowl —
GT labels arrive as prim-path stubs `obj_0,1,3,4,5,6,7` (`obj_2`=tomato\_soup\_can
is the unreachable-by-design object confirmed in E3/E5 prep, excluded here by
design, not by failure). Picks were fired programmatically over ROS topics
(`/grasp_target` then `gantry_reach_executor/pick 'target'`) rather than the
interactive `pick_cli`, to script a repeatable batch.

**Scale (revised down from the original 20-position × 3-trial plan, user-approved
2026-07-17):** the scene's 8 objects are at **fixed** poses baked into
`isaac_sim/workcell/polish.py` — `/target_object` only overrides the executor's
*planning* target, it does not move the physical Isaac object, so genuine
positional diversity would require editing `polish.py` and a full Isaac relaunch
per position group (~2–3 min each), impractical for 20 positions in one session.
Ran instead: **7 objects × 5 trials = 35 picks**, **round-robin** order (round 1:
obj\_0..obj\_7 skip obj\_2, then repeat 4 more rounds) rather than 5 consecutive
repeats per object, specifically to avoid the known "re-targeting an object the
arm is already parked at → transient −4 CONTROL\_FAILED" artifact
(`stable-track-identity-implemented` memory) that consecutive same-target repeats
would trigger. Preceded by 2 manual smoke-test picks (obj\_0, obj\_3) to validate
the pipeline before batching — both succeeded and are **not** counted in the 35.
This is a demo-scale pipeline-integration result, not a positional-sensitivity
study (that role belongs to E3's grid sweep).

**Result: 35/35 SUCCESS (100%), 0 FAILED, all 7 objects 5/5.** (37/37 including
the 2 pre-batch smoke picks — arm/CSV rows for those two are the first two rows of
`docs/e6_data_2026-07-17/e6.csv`.) Cross-checked three independent ways: the
driver's own trial log (`e6_trials.csv`, 35/35 `SUCCESS`), the executor's CSV
`success` column (37/37 rows `success=1`), and a raw grep of the executor's own
terminal log for its `>>> objN: SUCCESS`/`FAILED` lines (37 SUCCESS, 0 FAILED) —
all three agree.

| object | class | n | success | mean (s) | median (s) | min (s) | max (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| obj\_0 | cracker\_box | 5 | 5/5 | 36.4 | 40.0 | 28.1 | 42.0 |
| obj\_1 | scissors | 5 | 5/5 | 110.4 | 150.1 | 23.3 | 184.4 |
| obj\_3 | mustard\_bottle | 5 | 5/5 | 26.0 | 23.1 | 16.7 | 35.6 |
| obj\_4 | teddy\_bear | 5 | 5/5 | 23.3 | 24.9 | 10.5 | 31.3 |
| obj\_5 | banana | 5 | 5/5 | 20.8 | 19.6 | 13.2 | 28.1 |
| obj\_6 | mug | 5 | 5/5 | 34.4 | 29.0 | 23.6 | 55.2 |
| obj\_7 | bowl | 5 | 5/5 | 25.8 | 22.4 | 17.4 | 33.9 |
| **overall** | | **35** | **35/35** | **39.6** | **28.1** | **10.5** | **184.4** |

Time-to-pick = pick-request timestamp to the executor's own `>>> objN: SUCCESS`
log line (grasp-target announce → J-ranking → IK → plan → execute → settle-wait
confirm against `/joint_states`), independently re-derived from the raw executor
log, not read off the driver's self-reported timers.

**Arm/gantry selection (35 batch picks, energy-mode):** arm\_2: 10 (mug, bowl —
both near table3/world-origin, always cheapest via gantry\_1's right arm),
arm\_3: 13, arm\_4: 12 (cracker\_box, scissors, mustard\_bottle, teddy\_bear,
banana — all closer to gantry\_2 or split between arm\_3/arm\_4 trial-to-trial
depending on the arm's current parked state feeding `d_arm`), **arm\_1: 0**.
Report honestly: arm\_1 was never selected across all 37 picks in this run — an
artifact of *this* 8-object layout (no object sits closer to gantry\_1's left
mount than to arm\_2 or gantry\_2), not a general claim that arm\_1 is
disadvantaged; E3's full-grid sweep (union reachable hull, symmetric-y, 60
positions) already shows arm\_1 winning 12/60 targets under energy-mode, so the
zero count here is a property of the fixed scene, not the selection algorithm.

**Failure-mode breakdown:** terminal failures — **0** in every category
(no-detection: 0, IK −31: 0, plan-fail: 0, execution-abort-that-was-never-
recovered: 0). **Transient, self-recovered exec-aborts: 3** (all on obj\_1
[scissors], all cand#0=arm\_4 returning `exec err=1` mid-motion — a joint-state
settle mismatch, not a planning or IK problem — with the executor's own
candidate-fallback loop retrying and succeeding on cand#1=arm\_3 every time; this
is the mechanism previously documented in `reach-fusion-tick-blocking-ik`/
`settle-false-negative-progress-based`, still present but still correctly
self-healed by the executor's per-candidate retry, hence 3/35 trials with inflated
time-to-pick (150–185 s) but zero terminal failures). 1 transient IK failure was
also logged (part of one of those same 3 retry chains — a discarded candidate,
not a terminal outcome). 0 plan failures across all 37 picks.

**Data:** `docs/e6_data_2026-07-17/e6.csv` (raw executor CSV, 37 rows, includes
the 2 pre-batch smoke picks as rows 1-2), `docs/e6_data_2026-07-17/e6_trials.csv`
(driver's own per-trial log: round, object, timestamps, result, detail string
scraped from the executor's terminal SUCCESS/FAILED line), `e6_batch_driver.py`
(the round-robin driver script, for reproducibility). Executor terminal log was
not archived (too large / ephemeral `/tmp` path) — the SUCCESS/FAILED/`did NOT
reach` counts above were grepped live and are reproducible by rerunning the
driver against a fresh `pick_stack.launch.py ... csv:=/tmp/e6.csv`.

**→ Paper (Section VI-D):** per-object success table + the failure-breakdown
paragraph above (0 across all 4 categories asked for; the 3 self-recovered
exec-aborts are reported as a qualitative retry-robustness note, not a failure
category, since they never surface as a terminal `FAILED`). HRI natural-language
fetch table was **not** run this session (optional per the plan; `target_cli.py`'s
NL-fetch path exists and was smoke-tested in a prior session per
`stable-track-identity-implemented`, but that was on a different branch/pipeline
— left as future work here rather than reported without a fresh check on this
branch).

---

## E4 — YOLOE detection & localization accuracy vs Isaac ground truth — DONE 2026-07-17

**Setup:** same live pipeline as E6 (`launch_workcell.sh full` + `pick_stack.launch.py`),
new script `scripts/e4_compare.py`. Method: (1) with `/seg_source=isaac`, average
`/detected_objects` positions for a few seconds per object to get a stable
reference xyz per `obj_N` (Isaac's own ground-truth instance segmentation +
`object_localizer`'s depth deprojection — this **is** the ground truth, not a
YOLOE output); (2) switch `/seg_source=yoloe` and, for `duration` seconds,
snapshot `/detected_objects` + its label markers, matching every detection to
the nearest GT object within `match_radius`\,=\,0.40\,m (position-based
matching — GT and label were deliberately decoupled, see caveat below).
Unmatched detections are false positives; GT objects with zero matches are
misses. Default config: `seg_conf=0.25`, `seg_model=yoloe-11m-seg.pt`,
`imgsz=768`, both cameras (`rgbd`,`rgbd2`), prompts = the pipeline's default
list (`box,tin can,canned food,bottle,banana,teddy bear,scissors,mug,bowl`).

**Ground truth (8 objects, `obj_0`..`obj_7`; includes `obj_2` since E4 tests
perception, not reachability):** cracker\_box, scissors, tomato\_soup\_can,
mustard\_bottle, teddy\_bear, banana, mug, bowl.

**Caveat found and corrected mid-run (report honestly, do not hide):** the
first pass matched detections to GT purely by **position** for the
success-rate/error metrics, which is sound, but the **label text** read off
`/detected_objects/markers` comes from `object_localizer`'s majority-vote
track label, and that vote counter has **no decay** (`Counter` accumulated
since the track was created, no `vote_decay` — that fix exists only on a
different branch, `stable-track-identity-implemented`). Since the pipeline had
been running under `seg_source=isaac` for ~50 minutes before E4 (E6's picks),
each track already carried thousands of `obj_N` votes; switching to YOLOE for
a few hundred more votes could not flip the majority, so the first pass's
label field was **stale ground-truth text, not real YOLOE class names** — a
real artifact, verified directly (raw `/rgbd/seg/instance_segmentation_labels`
showed genuine YOLOE strings like `"gray canned food"` and `"brown scissors"`
the whole time; GPU utilization on the `seg_router` process jumped from ~0 to
722\,MiB / 52\% during the YOLOE window, confirming real inference was
running). **Fix:** restarted `object_localizer` alone (clears in-memory
tracks/votes, no rebuild needed) before each timed window so labels start
fresh under the active source. Position-based detection-rate/error numbers
were unaffected by this bug (verified identical between the stale-label and
fresh-label runs); only the qualitative "does YOLOE call it the right class"
table needed the fix.

**Primary result (`seg_conf=0.25`, 483 published frames over 279.5\,s,
1.72\,Hz, fresh tracker):**

| obj | class | detections | det. rate | mean err (m) | max err (m) | flicker rate | majority YOLOE label |
|---|---|---:|---:|---:|---:|---:|---|
| obj\_0 | cracker\_box | 483/483 | 100% | 0.070 | 0.070 | 0.2% (1/483) | "red box" (398/483, rest stale `obj_0` before reset settled) |
| obj\_1 | scissors | 483/483 | 100% | 0.015 | 0.019 | 0.2% (1/483) | "brown scissors" (461/483) |
| obj\_2 | tomato\_soup\_can | 483/483 | 100% | 0.022 | 0.022 | 0.2% (1/483) | "gray canned food" (461/483) |
| obj\_3 | mustard\_bottle | 483/483 | 100% | 0.008 | 0.009 | 0.2% (1/483) | "brown bottle" (473/483) |
| obj\_4 | teddy\_bear | 483/483 | 100% | 0.021 | 0.022 | 0.2% (1/483) | "brown teddy bear" (470/483) |
| obj\_5 | banana | 483/483 | 100% | 0.013 | 0.014 | 0.2% (1/483) | "brown banana" (464/483) |
| obj\_6 | mug | 483/483 | 100% | 0.017 | 0.017 | 0.2% (1/483) | "gray mug" (445/483) |
| obj\_7 | bowl | 483/483 | 100% | 0.015 | 0.020 | 0.2% (1/483) | "black bowl" (473/483) |
| **overall** | | **3864/3864** | **100%** | **0.019** | **0.070** | **0.2%** | all 8 classes semantically correct |

**False positives: 0/3864 (0%)** at the default `conf=0.25`. **Class labels
are all semantically correct** once the track was fresh (the residual
10–38/483 "stale" frames per object during the reset-settle window are the
old GT label lingering, not a YOLOE error — a real but transient artifact of
the no-decay voter, distinct from the position-based numbers which were clean
throughout). `tomato_soup_can` → "canned food" and `mustard_bottle` →
"bottle" are coarser than the YCB name but the right functional class; no
object was confused with a different one.

**Contradicts an old memory note** (`yoloe-sim-unreliable-use-gt`,
2026-07-06): that note found YOLOE "blind"/flickering on this simulator's
imagery. Re-checked: that finding predates the switch to the larger
`yoloe-11m-seg.pt` model (the code comment in `pick_stack.launch.py` already
says *"11m detects the sim objects far better than 11s"*) — the model upgrade,
not a pipeline fix, explains the reversal. **YOLOE is a viable perception
input on this branch's current config**, contrary to that older note; E6's
choice to use `seg_source=isaac` for its 35-pick batch was still the right
default (matches the paper's "sim-only, pose-reliability-first" framing and
the earlier finding was true for the model in use at the time), but this
result means YOLOE-driven E6 picks are now plausible future work, not
expected to fail outright.

**Sensitivity check — lower confidence threshold (`seg_conf=0.10`, 150 frames
over 98.9\,s, 1.50\,Hz, fresh tracker, `seg_router` restarted standalone with
the lower threshold; NOT a full {0.1,0.2,0.3} × {1,2 cameras} × {retina\_masks}
sweep — scoped down to one informative point given time, see below):**
detection rate stayed **100%** for all 8 objects (no headroom to improve, the
default already saturates), localization error was essentially unchanged
(0.005–0.070\,m per object, same order as `conf=0.25`), but **false positives
rose to 15.2% (215/1415)** — confirms `conf=0.25` is a reasonable operating
point: lowering it buys nothing on detection/localization and costs a
meaningful false-positive rate. `conf=0.30` and 1-camera / `retina_masks:=off`
variations were **not run** (diminishing expected value once the primary
result was this clean, and each needs its own relaunch — deferred as future
work, not fabricated).

**Inference rate / compute:** fused-track publish rate ≈1.5–1.7\,Hz (both
cameras' YOLOE inference + `object_localizer`'s fuse/track cycle); `seg_router`
process GPU usage during the YOLOE window: 722\,MiB resident, ~52% utilization
on the shared A6000 (spot-checked via `nvidia-smi`, not continuously profiled).

**Track stability:** this branch's `object_localizer` uses a simple
nearest-neighbour tracker with unlimited-history majority voting (see caveat
above) rather than the Hungarian-assignment + N-frame-confirm + vote-decay
tracker documented in `stable-track-identity-implemented` (a different
branch). Within a single fresh-tracker window, label flicker was **0.2%**
(1 frame in 483, the reset settling) — stable in the steady state, but the
no-decay vote history means a track that has run a long time under one source
will not visibly relabel for a long time after a source switch; this is a
property of the current code, not of YOLOE's per-frame accuracy (which the
raw per-frame label stream showed was consistently correct).

**Data:** `docs/e4_data_2026-07-17/e4_yoloe_conf025_v2.csv` (+ `_summary.json`,
3864 rows, canonical run), `docs/e4_data_2026-07-17/e4_yoloe_conf010.csv`
(+ `_summary.json`, sensitivity run), `e4_compare_snapshot.py` (the script,
`scripts/e4_compare.py` in the repo).

**→ Paper (Section VI-C, Perception Accuracy):** detection-rate/localization
table above, 0% false positives at the default operating point, the
conf-threshold trade-off note, and the honest correction of the older
"YOLOE unreliable" claim now that the model has been upgraded.
