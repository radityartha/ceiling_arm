# Figure Plan — paper

Companion to [paper_draft.md](paper_draft.md). Where images/diagrams go, what each
shows, priority, and whether the asset already exists. IEEE 6-page papers usually run
5–7 figures; keep to the essentials plus the two result plots.

Legend: ⭐ essential · ➕ nice-to-have · 📸 asset already exists · 🎨 needs to be made

---

## Fig. 1 — Two-panel teaser: system + end-to-end pipeline  ⭐  (Section III, page-1 teaser)
A combined page-1 figure that answers both "what is the robot?" and "how does the method
work?" in one glance.
- **Left panel — physical system + frames.** Schematic of the ceiling rail carrying the
  two arms, with world → rail → arm-base → tool frames labelled, the rail's prismatic (d)
  and revolute (θ) axes drawn, the 8-DOF chain called out, and the two overhead RGB-D
  cameras with their field of view over the workspace.
- **Right panel — end-to-end pipeline.** Three-stage data flow: Perception (two RGB-D
  cameras → YOLOE detect+segment → object pose) ┃ Selection (per-arm capability map →
  pool candidates → score by energy cost J → round-robin IK) ┃ Planning/Execution (MoveIt
  plan against the octomap → optional execute). Draw the Selection stage with its internal
  steps visible, since that is the contribution.
- Asset: 🎨 both panels to draw; 📸 an Isaac render can back the left panel.
- Caption draft: see caption list below.

## Fig. 2 — Isaac Sim scene  ➕  (Section III-B)
The simulated workspace with the tabletop objects, showing the reachable spread and the
one object near the edge. Can be merged into Fig. 1's left panel to save space.
- Asset: 📸 screenshot from the running scene (polish.py layout).

## Fig. 3 — Perception front end  ➕  (Section III-C)
YOLOE detection/segmentation overlaid on the scene (masks + labels) and the localized
object point in the world frame. Sells the enabler in one glance; drop first if space
is tight.
- Asset: 📸 capture from the live detector (/detected_objects overlay).

## Fig. 4 — Base-aware GNG capability map  ⭐  (Section IV, = E0 figure)
The GNG node cloud in RViz, coloured by manipulability (blue edge → red dexterous),
with the pinned boundary shell visible and the rail-extended extent obvious. The title
names this map, so it must be shown. A second small panel coloured by reachability
density (hits) is a strong optional add.
- Asset: 📸 exists — `view_gng.launch.py model_path:=/tmp/arm1_model.npz`
  (see experiment_plan.md E0). Just capture cleanly.

## Fig. 5 — Energy-selection illustration  ⭐  (Section V)
A concrete top-view: object plus both arms, showing the base travel each would need, and
J choosing the arm/config with lower energy — ideally the "picks the farther arm because
it is cheaper" case. Turns the cost function from math into intuition.
- Asset: 🎨 diagram, values from a real logged pick (E3 CSV) so it is not invented.

## Fig. 6 — Results  ⭐  (Section VI)
Two plots, likely a 2-panel figure:
(a) Reachable-workspace gain, rail locked vs active (E1) — bar chart, numbers exist
    (~5.1× @0.05 m). 📸 data ready.
(b) Base travel / mechanical energy per selection mode (E3) — boxplot,
    energy vs nearest/fixed/random. 🎨 pending E3 data.
- Asset: (a) ready now; (b) after E3 run.

## Fig. 7 — Real robot photo  ➕  (Section VII, sim-to-real)
One photo of the physical platform to back the "hardware validation is future work"
statement (per the sim-only justification in experiment_plan.md).
- Asset: 📸 take one photo of the real platform.

---

## Caption drafts (keterangan tiap figure)
IEEE style: caption below the figure, one or two sentences, self-contained. Numbers
appear only where real data exists.

- **Fig. 1.** Overview of the ceiling-rail dual-arm robot and the proposed pipeline.
  Left: a rail mounted to the ceiling carries two Kinova Gen3 Lite arms and adds a
  prismatic axis (d) and a revolute axis (θ) that both arms share, so each arm forms an
  eight-degree-of-freedom chain q = [d, θ, q₁…q₆]; two overhead RGB-D cameras observe the
  workspace. Right: the end-to-end pipeline, from open-vocabulary perception, through
  energy-aware arm-and-base selection over the base-aware capability map, to collision-free
  planning and execution.
- **Fig. 2.** The simulated workspace in NVIDIA Isaac Sim, viewed from one of the two
  overhead RGB-D cameras, showing the tabletop objects used in the experiments; at least
  one object is placed near the edge of the reachable workspace.
- **Fig. 3.** Open-vocabulary perception. YOLOE detects and segments the requested
  object from a text prompt (masks and labels shown); the mask is back-projected with
  depth to localize the object in the world frame.
- **Fig. 4.** The base-aware GNG capability map for one arm, rendered in RViz. Nodes are
  coloured by manipulability (blue near the workspace edge, red where the arm is
  dexterous); the pinned boundary shell reaches the true rail-extended reach surface.
- **Fig. 5.** Energy-aware selection in a representative pick (values from a logged
  trial). The cost J prefers the arm and base placement with lower total energy, which
  here is not the arm whose end-effector is nearest the object.
- **Fig. 6.** Results. (a) Reachable-workspace volume with the rail locked versus active,
  showing the gain from treating the base as a DOF. (b) Base travel and mechanical
  energy per selection mode (energy-aware vs nearest, fixed, random).
- **Fig. 7.** The physical ceiling-rail platform. Hardware validation is left to future
  work; all results in this paper are obtained in simulation.

## Priority if space forces cuts
Keep: Fig. 1, 4, 6. Strongly keep: Fig. 5. Cut first: Fig. 3, then Fig. 2 (merge
into Fig. 1's left panel). Fig. 7 is one small inset, cheap to keep.

## Note on honesty
Any number shown in a figure (Fig. 5 example values, Fig. 6 plots) must come from real
logs (E1/E3 CSVs), never illustrative/made-up numbers. Mark a figure "placeholder" until
its data exists.
