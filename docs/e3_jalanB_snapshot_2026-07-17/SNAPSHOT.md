# E3 Jalan B Snapshot — 2026-07-17 (restore point / comparison baseline before Jalan C)

Preserved so that **Jalan C** (add a motor copper-loss term so `w_hold` becomes
meaningful, then recalibrate + rerun E3) has a fixed, known-good baseline to compare
against and revert to if it fails. This is the state committed on
`paper/e3-energy-aware` at commit `e3e1cd7` (+ later prose-only commits), BEFORE any
Jalan C code change. Jalan C work happens on a separate branch, `paper/e3-jalanC`.

Raw CSVs archived in `./csv/` (7 files, 60 rows each, shared header,
`selection_mode` column). These are the **rail-friction, mechanical-work-only**
energy model (`E = sum |tau·qdot| dt` + explicit rail Coulomb/viscous dissipation),
BEFORE any motor copper-loss (`I²R`) term.

## Jalan B weights/refs (live `declare_parameter` defaults in `gantry_reach_executor.py`)

| term | weight (w) | ref (median) |
|---|---:|---:|
| `gantry_lin` | 27.16 | 1.1809 m |
| `dist` (ee_dist) | 12.78 | 1.4509 m |
| `gantry_rot` | 3.08 | 1.2395 rad |
| `manip` | 2.95 | 0.1054 |
| `arm` | 0.09 | 7.1760 rad |
| `hold` | **0.0 (disabled)** | 2.0382 Nm |

`w_hold=0` is the specific thing Jalan C is trying to fix: under mechanical-work
energy (`tau·qdot`), a purely static holding posture (`qdot=0`) does **zero**
mechanical work, so `hold` (static gravity torque) structurally cannot correlate
with `traj_energy` — the OLS fit came out negative/wrong-signed and was clamped to
0. This is a blind spot analogous to the pre-Jalan-B frictionless rail.

## Jalan B fit quality (168-pick OLS regression, `scripts/regress_j.py`)

- **R² = 0.857**
- **Spearman(J, traj_energy) = 0.909**

## Jalan B E3 full-sweep results (60 positions/mode, plan-only, friction model)

| mode | success | traj_energy mean/median (J) |
|---|---:|---:|
| **energy (Jalan B)** | 56/60 (93.3%) | **45.84 / 52.34** |
| nearest | 56/60 (93.3%) | 48.28 / 54.79 |
| random | 56/60 (93.3%) | 47.31 / 51.72 |
| fixed arm_1 (worst) | 38/60 (63.3%) | 45.43 / 43.12 |
| fixed arm_2 | 39/60 (65.0%) | 49.93 / 50.30 |
| fixed arm_3 | 39/60 (65.0%) | 47.25 / 51.89 |
| fixed arm_4 (best, tied) | 39/60 (65.0%) | 49.79 / 55.78 |

Paired same-target win rate (56 comparable rows): energy-mode's `traj_energy` lower
than nearest's in 32/56 (57.1%) and lower than random's in 34/56 (60.7%). Verdict:
**S2 "pass-weak"** per `docs/E3_contingency.md` — energy-mode wins on mean, is
close on median, real-but-not-overwhelming pairwise majority.

## Verdict recorded in the paper (as of this snapshot)

`docs/experiment_results.md` E3 "Jalan B" section, `energy_aware_selection.tex`
(abstract, `tab:weights`, `tab:e3`, Section V-A, VI-C) all reflect these exact
numbers. If Jalan C fails (`w_hold` stays ~0, or energy-mode stops winning, or the
copper-loss term dominates unrealistically), **revert to this state**: `git switch
paper/e3-energy-aware` (Jalan B is fully intact there), delete/archive
`paper/e3-jalanC`, and keep the framing exactly as committed at this snapshot
(`w_hold=0` disabled, with an honest note that a motor-loss model is future work).
Do not force `w_hold` to be non-zero if the data does not support it.

## Reproduce

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch reachability_gng gantry_pick.launch.py execute:=false \
    compute_traj_energy:=true selection_mode:=energy csv:=/tmp/e3b_energy_v2.csv
python3 scripts/e3_grid_driver.py --nx 6 --ny 5 --nz 2 --wait 7
# repeat per mode (nearest/random/fixed_arm1..4); regression: scripts/regress_j.py
```
