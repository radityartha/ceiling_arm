# Experiment Results (running log)

Companion to [experiment_plan.md](experiment_plan.md). Records actual numbers as
collected. All datasets reproducible with `--seed 0`; artefacts currently in `/tmp`
(regenerate via commands below — do NOT rely on /tmp persistence).

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

**Headline: gantry DOF expands the reachable workspace ~5× (5.10× at res 0.05 m;
robust bracket 5.1–5.8× over saturated resolutions).** locked bbox ≈ 1.51×1.52×1.49 m
(arm-only sphere); active bbox ≈ 5.19×2.31×1.49 m (rail sweeps X).

Reproduce:
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
python3 -m reachability_gng.data_gen --config ros2_ws/src/reachability_gng/config/arm1_table1.yaml --out /tmp/e1_active_hi.npz --n 450000 --seed 0
python3 -m reachability_gng.data_gen --config ros2_ws/src/reachability_gng/config/arm1_locked.yaml --out /tmp/e1_locked_hi.npz --n 150000 --seed 0
python3 -m reachability_gng.eval volume --datasets /tmp/e1_locked_hi.npz /tmp/e1_active_hi.npz --res 0.05
```

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

**Headline: boundary seeding cuts the reachable-edge shortfall from 0.26 m to
<0.01 m (max radius), recovering the true workspace extent — biggest gain along
the rail axis X (0.545 → 0.029 m).** Reconciles with the earlier ad-hoc 0.26→0.00
claim. Shortfall script: `scratchpad/edge_shortfall.py` (extent calc inline).

## E0 — GNG map characterization (arm_1, arm_2) — DATA DONE 2026-07-05 (figure TODO)

Fresh `build_maps.sh` (N=80000, seed per data_gen default, max-nodes 3000, lam 60,
epochs 2, boundary 600). Build time both arms ≈ 5m45s.

| property | arm_1 | arm_2 |
|----------|-------|-------|
| FK samples | 80000 | 80000 |
| surface pts detected | 8578 | 8459 |
| total nodes | 3000 | 3000 |
| pinned boundary (shell) | 600 | 600 |
| interior nodes | 2400 | 2400 |
| edges | 15896 | 15965 |
| median node spacing | 0.176 m | 0.176 m |
| mean node spacing | 0.196 m | 0.197 m |
| task_dim | 3 (xyz) | 3 (xyz) |
| q DOF per node | 8 (2 gantry + 6 arm) | 8 |
| reachable volume (res 0.05) | 9.79 m³ (from E1) | ~ (rerun if needed) |

TODO E0: RViz figure — `ros2 launch reachability_gng view_gng.launch.py
model_path:=/tmp/arm1_model.npz` → screenshot cloud coloured by manipulability +
visible boundary shell (Section 3 figure). USER GUI step (needs display/noVNC).
