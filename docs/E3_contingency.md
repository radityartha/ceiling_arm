# E3 Contingency Plan — fallbacks if energy-mode does not clearly beat the baselines

Prepared 2026-07-16 while E3 (the star experiment) is being collected in a separate
session. Decide which path to take **after** reading the combined E3 CSV, not before.
Nothing here fabricates a result; each option is conditional on what E3 actually shows.

## The one test the title "Energy-Aware" must pass
On the shared grid, `selection_mode=energy` (minimising J) must **reduce the measured
`traj_energy`** (Pinocchio absolute mechanical work, per-pick CSV column) versus
`nearest / fixed / random`. Secondary supports: lower base travel (`d_gantry_lin` +
`d_gantry_rot`) and lower arm travel (`d_arm`).

## Why this is at risk (from the J-weight / energy analysis, 2026-07-16)
- J weights (code): arm **20** > rail-rot **12** > dist **3** > rail-lin **2** > hold **1** = manip **1**; each calibrated from `Spearman(raw term, traj_energy)`, best single term (arm) only **rho≈0.31 (weak)**.
- `traj_energy` = `sum |tau·qdot| dt` via rigid-body RNEA. The URDF rail joints have **no `<dynamics>` damping/friction** and the rail axes are **horizontal** (orthogonal to gravity). So base **translation** barely registers in the modelled energy; J's energy signal comes from **arm posture + rail rotation**, not base translation.
- Consequence: the paper's motivation ("repositioning the heavy base is a dominant cost") is **not represented** in the sim energy, so the calibrated weights cannot reflect it. Base-linear translation is small (inertial-only, low planning speed) — **not zero** (code uses absolute work), just weakly correlated with total energy.

## Read these four outcomes off the combined CSV
| Scenario | What the CSV shows | Fallback |
|---|---|---|
| **S1 PASS clean** | energy-mode `traj_energy` clearly < all baselines AND base+arm travel lower | none — keep "Energy-Aware" |
| **S2 PASS weak** | energy < baselines but small, driven mostly by arm travel; base-linear travel not reduced | **A** (bound the claim) — most likely |
| **S3 energy FAIL / travel PASS** | energy ≈ baselines, but base+arm **travel** clearly lower | **A** (reframe to travel) or **B** (fix model) |
| **S4 FAIL both** | energy-mode no better on energy or travel | **C** (diagnose first) |

---

## Fallback A — reframe "energy" → "travel / repositioning cost" (SAFEST, cheapest)
Base travel (m, rad) is a first-class, well-motivated cost in the base-placement
literature we already cite: **MoMa-Pos** and **BaSeNet** optimise base pose/travel
directly. The CSV always carries it and energy-mode reduces it by construction (J is
dominated by travel terms). So this claim is defensible even if the energy calibration is
weak.
- **Title:** either drop "Energy" → *"Motion-Efficient / Travel-Aware Arm and Base
  Selection ..."*, OR keep "Energy-Aware" but have Section VI report **both** travel
  (primary, strong) and idealised energy (secondary, explicitly hedged).
- **Edits:** V-A frames J as a **motion/effort cost** (weighted travel + posture), not
  "energy". VI headline = **travel reduction** per mode; idealised energy is a secondary
  row with the "gravity+inertia, frictionless" caveat. Intro "base is a dominant cost"
  stays as **motivation** (real-world / literature), no longer something the sim energy
  proves. Abstract: "lowers base repositioning and arm motion".
- **Honesty:** strongest option; matches data, fabricates nothing.

## Fallback B — fix the model: add rail friction/damping + recalibrate (PRINCIPLED, more work)
Make the sim energy actually charge base translation, so J becomes a genuine energy proxy
and w_gl rises to reflect "base is expensive".
- **URDF:** `ros2_ws/src/workcell_description/urdf/moving_table.urdf.xacro`
  - `linear_joint` (prismatic, ~L39–44): add `<dynamics damping="D_lin" friction="F_lin"/>`
  - `rotation_joint` (revolute, ~L73–78): add `<dynamics damping="D_rot" friction="F_rot"/>`
  - Choose **physically-reasoned** values: Coulomb `friction ≈ mu·N`, N = weight of the
    carriage + 2 arms (state the assumed mass and mu); small viscous `damping`. These are
    **modelling assumptions, not measured** — present as a sensitivity/assumption, never
    as real hardware energy.
- Regenerate the flattened URDF the energy model loads:
  `xacro workcell.urdf.xacro > workcell_full.urdf` (this is what `arm_pin_configs` →
  `build_model` reads for `pin.rnea`; verify path). Reach maps do **not** need rebuilding
  (kinematics unchanged), only the dynamics model.
- **Recalibrate:** collect a fresh calib session (picks with `traj_energy`) →
  `analyze_calib.py` → new `ref_*` (median) + `w_*` (Spearman). Expect w_gl to increase.
- Re-run E3. Cost: friction modelling + a recalibration pass + E3 rerun. Risk: may still
  not fully validate; but it is the only path that makes "energy-aware base selection"
  defended by the sim's own data.

## Fallback C — if both fail (S4): diagnose before any reframe
Do **not** reword the paper yet — a total failure usually means a setup bug, not a
narrative problem:
- Confirm baselines: `nearest` = task-space nearest node across all 4 arms; `random` =
  seeded over 4 arms; `fixed` = all 4 arms (best/worst). Confirm energy-mode ever picks a
  **different arm** than nearest (`rank_J` vs `rank_dist`, `arm` differs). If J never
  differs from nearest, the pool/scoring is degenerate → fix pooling, not wording.
- Plan-only keeps the arm at home, so `d_*` is measured from home every trial; modes only
  separate when they pick a **different arm/config**. If the grid gives little base-travel
  variation, widen it or vary the start state so the modes actually diverge.

## Recommendation
Primary fallback = **A** (travel reframe) — robust, honest, cheap; likely enough for
S2/S3. Keep **B** (URDF friction + recalibration) ready as the principled upgrade if the
team wants the word "Energy" defended by the sim data and there is time. Choose after the
combined E3 CSV lands (S1–S4). The E3 experiment remains the arbiter either way.
