# Paper Draft (fixed sections)

Working draft of *"Energy-Aware Arm and Base Selection for a Dual-Gantry Quad-Arm
Ceiling Robot Using a GNG Capability Map."* (Title shortened 2026-07-16: dropped the
redundant second "-aware" — the map's base-awareness is stated in the body, not the
title.) Only paragraphs the user has
signed off on live here. Citations use tags from [paper_references.md](paper_references.md);
`[PLACEHOLDER]` = experiment number pending; `[NEED CITE]` = source still to be found
(never fabricated).

SCOPE (updated 2026-07-16): the paper now covers the FULL system — TWO ceiling
gantries, each carrying TWO arms, four arms total. This supersedes the earlier
"one gantry, two arms" scope, following the system's code default
(`arm_names=[arm_1..arm_4]` across `gantry_1`/`gantry_2`) becoming consistent across
every relevant file (executor, pick_cli, reachability_check/cloud, SRDF,
kinematics.yaml, build_maps.sh). Do NOT touch
docs/IEEE-conference-Energy_Aware/energy_aware_selection.tex — that file belongs to
a different branch's draft and is off-limits.

Terminology (locked, updated for 4-arm scope): single chain = "arm" (a 6-DOF
manipulator); whole system = "ceiling-mounted quad-arm service robot"; base = "a
ceiling rail" — there are TWO rails (one per gantry), each with one linear and one
rotational DOF; two arms share each rail, so arms on the SAME rail share a base while
arms on DIFFERENT rails move independently.

---

## I. Introduction

**¶1 — Social background.**
Service robots are moving into the home, where they are increasingly asked to fetch
objects, tidy up, and hand things to the people living there [IFR2025], [Lisondra2025].
This setting is far less forgiving than a factory floor. A home is cluttered and
short on space, and the robot has to share that space with its occupants at all
times. A conventional floor-standing mobile manipulator sits awkwardly in this
picture. It occupies the same walkable area that people use, and its motion near the
ground raises real safety and collision concerns. One way around this is to lift the
robot off the floor entirely. If the arms hang from the ceiling and travel along an
overhead rail, their working volume stays above everyday human activity, the floor is
left clear, and objects are approached from above rather than from across the room
[Jayathilaka2025]. We study a robot of exactly this kind. Four arms, each a 6-DOF
manipulator, are carried in pairs by two ceiling rails, each of which can both slide
and rotate over the workspace.

**¶2 — Specific problem.**
Because each rail moves its pair of arms as well, every arm paired with its rail
becomes an 8-DOF redundant chain. A given object can usually be reached by more than
one arm, possibly on different rails, and from many different rail positions. The
robot therefore faces two coupled questions at once. Which arm should do the task, and
where should the base be placed before it reaches. Neither question is free of cost. Repositioning the heavy base is one of the
dominant expenses in mobile manipulation, both in time and in energy [MoMaPos2024],
[BaSeNet2024], [EnergySLR2024], so a simple policy that always reaches with the
closest arm, or that leaves the base wherever it happens to be, can waste a great deal
of base travel. Choosing the arm and the base placement that keep this energy low is
the problem we set out to solve.

**¶3 — Limitations of existing approaches.**
Reachability and capability maps are a common way to describe where an arm can reach
and how well it can move there, and they are widely used to decide where a robot
should stand [Zacharias2007], [Makhal2018]. Two assumptions limit them for our
setting. First, most maps are built for a fixed base, so they never capture how moving
the base itself opens up new reachable space. Second, they tend to record only whether
a cell is reachable, without attaching a cost of getting there or a configuration that
inverse kinematics can start from [Zacharias2007]. More recent methods add awareness
of obstacles and nearby objects [MoMaPos2024], [InvReach2022], yet they still decide
where a single arm should stand rather than which of several arms to use under an
energy budget. Redundancy-resolution work optimizes a single arm's posture for
measures like manipulability [Yoshikawa1985], but again does not choose between arms
or reason about the expensive base degree of freedom. What is missing is a
representation that answers, directly, which arm and which base placement cost the
least energy on a redundant ceiling-rail robot.

**¶4 — Approach and contributions.**
Our answer is an energy cost function J that operates on a base-aware capability map
built with Growing Neural Gas (GNG) [Fritzke1995]. Every node in the map keeps a
representative 8-DOF configuration, along with its manipulability [Yoshikawa1985] and
the effort needed to hold that posture against gravity. A node is thus two things at
once, a cell of reachable space and a ready starting point for inverse kinematics. The
paper makes three contributions.
First, we introduce an energy-aware selection cost J that chooses the arm and the full
8-DOF configuration together, rail travel included, instead of settling for the
nearest reachable pose.
Second, we build a base-aware GNG capability map that treats each ceiling rail as a
genuine degree of freedom, and that enriches plain reachability with a representative
8-DOF configuration, a manipulability value, and a holding cost at every node. Because
that configuration already includes a rail placement, each node is a self-contained
candidate for the selection stage rather than a bare occupancy bit. A boundary-seeding
step keeps the map accurate right up to the edge of the reachable workspace.
Third, we evaluate the whole approach in Isaac Sim, characterizing the map, measuring how
much the rail as a degree of freedom expands the reachable workspace, and comparing
energy-aware selection against nearest, fixed, and random baselines. Open-vocabulary
perception with YOLOE [Wang2025] provides the perception front end, supplying the target
objects that the selection pipeline then acts on.

**¶5 — Results preview and outline.**
The simulation results support each claim. Letting a rail act as a degree of freedom
enlarges the reachable workspace by roughly four times compared with an arm-only
baseline, and the boundary-seeding step recovers the true reach at the workspace edge to
within a centimetre. Energy-aware selection cuts base travel and mechanical energy
relative to the nearest, fixed, and random baselines by `[PLACEHOLDER]`. All of these
numbers come from Isaac Sim, and validation on the physical robot is left for future
work. The rest of the paper is organized as follows.
Section II reviews related work, Section III describes the system, and Section IV
presents the base-aware GNG capability map. Section V defines the energy cost J and the
selection procedure, Section VI reports the experiments, and Section VII concludes.

---

## II. Related Work

**Reachability and capability maps.**
A long line of work represents where a manipulator can reach by discretizing its
workspace and recording, for each cell, whether the end-effector can be placed there.
Zacharias et al. turned this idea into a capability map that also captures how the arm
can be oriented at each location, giving a richer picture than a plain binary
occupancy grid [Zacharias2007]. Reuleaux extended the same principle to base placement
by inverting the reachability map, so the robot can ask where it should stand to serve
a target [Makhal2018]. More recent representations focus on making these maps cheaper
to build and query. RM4D, for example, reduces the discretization from six dimensions
to four and serves both reachability and inverse-reachability queries from one
structure [RM4D2024]. These maps are built for a single arm on a fixed base, and a cell
usually stores reachability alone. Our map differs on both counts. It treats the
ceiling rail as part of the configuration, so it records how base motion extends
reach, and every node carries a ready inverse-kinematics seed together with a
manipulability and a holding cost rather than a single occupancy bit.

**Base placement for mobile manipulators.**
Deciding where a mobile base should stop before manipulating is a well-studied problem.
Sandakalum et al. learn an inverse-reachability network that places the platform for a
given task while accounting for obstacles [InvReach2022]. MoMa-Pos optimizes base
placement by reasoning about the kinematics of the objects to be manipulated
[MoMaPos2024], and BaSeNet plans a whole sequence of base poses so that a robot can
pick up several objects in turn [BaSeNet2024]. All of these methods answer where a
single arm should be positioned. None of them chooses among several arms, and they do
not rank candidate placements by the energy that reaching from them would cost. Our
work adds both of these missing pieces for a redundant ceiling-rail robot.

**Redundancy resolution and inverse kinematics.**
When a chain has more joints than the task requires, its extra freedom can be used to
improve a secondary objective. Yoshikawa's manipulability measure is a classic choice,
steering a redundant arm away from singular postures [Yoshikawa1985]. Solving the
inverse kinematics itself has also seen renewed interest through learning, with recent
solvers such as IKDiffuser generating configurations from a diffusion model
[IKDiffuser2025]. These approaches resolve the posture of one arm for one criterion at
a time. They do not decide which arm, among several, should act, and they carry no notion of
the base travel that a given posture would require. We keep the inverse kinematics
solver itself standard and classical, and use the capability map for a different purpose,
to enumerate candidate 8-DOF configurations, each with its own arm and rail placement,
that the energy cost then ranks to settle the arm choice and the base motion.

**Multi-arm coordination.**
Systems with several arms raise the question of how to divide the task between them.
Recent planners such as DAG-Plan and RoboPARA build dependency graphs over a task and
assign subtasks to each arm so that several arms can act in parallel [DAGPlan2024],
[RoboPARA2025]. Their concern is the temporal and logical structure of a multi-step
task. Ours is different and complementary. For a single reaching action we ask which
arm reaches the target for the least energy, given that arms sharing a rail share one
moving base while arms on different rails move independently, and we answer it with
the same map and cost that also place the base.

---

## III. System

**III-A. Robot platform and kinematic model.**
The robot consists of two rails mounted to the ceiling, arranged side by side, each
carrying two arms. Each arm is a Kinova Gen3 Lite, a six-joint manipulator with a
reach of about 0.76 m and a two-finger gripper at its wrist [Kinova], for four arms in
total. Each rail adds two more degrees of freedom that its pair of arms share, a
prismatic axis that slides the carriage horizontally and a revolute axis that rotates
it, so relative to the ground each arm forms an eight-degree-of-freedom chain. We
write a configuration of one arm as q = [d, θ, q₁, …, q₆], where d and θ are the
translation and rotation of that arm's own rail, and q₁…q₆ the arm joints. The
kinematic tree is anchored at a fixed world frame. Each rail base is attached to the
world, each arm base is attached to its rail, and a tool frame at the gripper defines
the end-effector pose that the task is specified in. Two arms share each carriage, so
moving a rail moves its pair of arms together and couples the choice of arm to the
placement of that base, while arms on different rails move independently of one
another. Figure 1 shows the platform and its frames alongside an overview of the
end-to-end pipeline, from perception through energy-aware selection to planning and
execution.

**III-B. Simulation environment.**
All experiments in this paper are carried out in NVIDIA Isaac Sim, a GPU-accelerated
simulator with a physics engine suitable for contact-rich manipulation
[IsaacSim2026], [Mittal2023]. The simulated robot is built from the same URDF
description that defines the physical platform, so the kinematics and joint limits used
in simulation match the real hardware. The scene places a set of everyday tabletop
objects within the workspace, some within easy reach of several arms and at least one
deliberately positioned near the edge of the reachable workspace (Fig. 2). The scene is
observed by two RGB-D cameras placed above the workspace, one at each end of the shared
corridor the two rails run along, which together are the system's only exteroceptive
sensor for both gantries. Their images serve two roles at once. The color stream feeds the
object detector, and the depth stream builds the collision octomap used for planning, so
a single sensing rig supports both perception and collision avoidance. We treat the simulator as the evaluation platform for
this work and validate on the physical robot in future work.

**III-C. Perception front end.**
Target objects are supplied to the pipeline by an open-vocabulary perception module
based on YOLOE, a real-time model that detects and segments objects from free-form text
prompts [Wang2025]. Running on the color image from each of the two overhead cameras, it
returns a segmentation mask per object, which we back-project with the aligned depth and
combine across the two viewpoints to estimate the object's position in the world frame. This lets the operator ask for an object by name and have the system
localize it without a fixed object database (Fig. 3). Perception acts as the enabling
front end here, providing the target that the capability map and the energy cost then
act on.

**III-D. Planning and execution stack.**
Given a target and a candidate configuration, the motion layer is built on MoveIt 2
[Coleman2014] with planners from the Open Motion Planning Library [Sucan2012]. Inverse
kinematics requests are answered by MoveIt against the current robot state, and
collisions are checked against an octomap that is populated from the two cameras' depth
images, so a plan avoids both the scene and the robot's own body. The object chosen as
the target is excluded from the cloud that feeds the octomap so that it can be grasped,
while the remaining objects and the environment stay as obstacles. The capability map of
Section IV is built offline with the Pinocchio rigid-body-dynamics library
[Carpentier2019], which we use for forward kinematics when sampling configurations and
for the gravity torque that gives each map node its holding cost. At run time a candidate
drawn from the map seeds the inverse-kinematics query, MoveIt plans a collision-free
motion to it, and the motion is optionally executed in simulation. The selection
procedure that decides which candidate to try, and in which order, is the subject of
Section V.

---

## IV. Base-Aware GNG Capability Map

The core of our method is a capability map of a single arm. For every location the
end-effector can reach, the map stores a configuration that reaches it and a measure of
how good that configuration is. Unlike a reachability map, which encodes only whether a
location is attainable, this representation associates each reachable location with a
concrete configuration and a quality measure, and it is base-aware in that the stored
configuration incorporates its rail's degrees of freedom. We build one such capability
map per arm, so the four arms across the two rails are described by four maps that we
query independently in Section V.

**IV-A. Sampling reachable configurations.**
We sample the eight-degree-of-freedom configuration q = [d, θ, q₁, …, q₆] uniformly
within the joint limits and compute the resulting end-effector pose by forward kinematics
with the Pinocchio library [Carpentier2019]. For each sample we also record the
manipulability w = √det(JJᵀ) [Yoshikawa1985] and the gravity torque required to hold the
configuration. This produces a dataset of (pose, q, manipulability, holding cost) tuples
that covers the combined arm-and-rail workspace. Because the rail translation d and
rotation θ are sampled alongside the arm joints, the dataset already reflects the reach
that base motion provides, not just the reach of a fixed arm.

**IV-B. Growing the map.**
We fit a Growing Neural Gas network [Fritzke1995] to this dataset. Each node stores a
vector [x | q] that concatenates the end-effector position x, which we call the task
part, with the configuration q that produces it. Training is arranged so that the
best-matching-unit search uses the task part alone, which makes the nodes tile the
reachable workspace, while adaptation updates the whole vector, so each node's q
converges to a configuration that is representative of its workspace cell. Edges are
formed between co-activated nodes by competitive Hebbian learning, giving the map a
topology over the workspace. Every node is therefore two things at once, a location the
end-effector can reach and a representative 8-DOF configuration that reaches it, rail
placement included. This is the sense in which the map is base-aware. Because q carries
the rail translation and rotation, two ways of reaching the same point from different
base placements settle into different nodes, so a node's configuration stays a valid
single posture rather than an average of incompatible base positions. That configuration
later seeds the inverse-kinematics query for its candidate, but its role in this paper is
to make each node a self-contained arm-and-base candidate for the energy cost to score. To keep the map faithful at the reachable
boundary, where nodes grown only from interior samples would otherwise settle inward, we
pin a shell of nodes on the measured reach surface before growing the interior (Fig. 4),
and Section VI reports the resulting edge fidelity.

**IV-C. Capability layers.**
Beyond plain reachability, each node carries two quality values that the energy cost of
Section V draws on. The manipulability w [Yoshikawa1985] records how dexterous the arm is
at that node, low near the workspace edge and near singular postures and high where the
arm can move freely in any direction. This quality layer is what separates our map from a
binary reachability grid that only marks a cell reachable or not [Zacharias2007]. The
holding cost, the norm of the gravity torque at the node's configuration, records how
much effort it takes to hold that posture against gravity. Both values are computed once,
offline, with the same Pinocchio model used for sampling [Carpentier2019] and stored on
the node, so they are available at query time at no extra cost.

---

## V. Energy-Aware Arm and Base Selection

Given the object pose from perception and the four capability maps, the selection stage
decides which arm to use and which eight-degree-of-freedom configuration to reach with.
It makes both decisions together, by an energy cost evaluated over candidates drawn from
the maps.

**V-A. The energy cost.**
We score a candidate configuration by a weighted sum of the effort of using it,

J = w_gl·(Δ_lin/ρ_gl) + w_gr·(Δ_rot/ρ_gr) + w_arm·(Δ_arm/ρ_arm)
    + w_dist·(e/ρ_dist) + w_hold·(h/ρ_hold) − w_manip·(m/ρ_manip),

where Δ_lin, Δ_rot, and Δ_arm are the rail-linear, rail-rotation, and summed arm-joint
travel from the current state to the candidate, e is the distance from the arm's current
tool frame to the object, h is the gravity torque needed to hold the candidate posture,
and m is its manipulability [Yoshikawa1985]. The rail's linear and rotational axes carry
separate weights because their units and their cost differ, metres of heavy-carriage
travel against radians of rotation. Manipulability enters with a minus sign, so a more
dexterous configuration is cheaper. Each term is divided by a fixed reference ρ before
its weight is applied, which makes the terms dimensionless and of comparable magnitude so
that a weight reads directly as the priority of that term. We set each reference to the
median value the term takes over a calibration set of picks, so that no term dominates
the sum merely because of the units it is measured in, and we tune the weights on the
same set rather than by hand. Table 1 lists the calibrated weights and references.

**Table 1.** Calibrated weights and normalization references for the energy cost J.
Each reference ρ is the median raw value of its term over the calibration set of picks.

| Term | Symbol | Weight | Reference ρ |
|------|--------|-------:|------------:|
| Rail linear travel | Δ_lin | 2 | 0.95 |
| Rail rotation travel | Δ_rot | 12 | 0.70 |
| Arm joint travel | Δ_arm | 20 | 6.00 |
| Tool-to-object distance | e | 3 | 1.36 |
| Gravity holding torque | h | 1 | 2.90 |
| Manipulability (subtracted) | m | 1 | 0.145 |

The terms are proxies for the energy of carrying out the pick. Rail and arm travel are
the mechanical work of moving each axis, with the heavy rail the dominant cost
[MoMaPos2024], [BaSeNet2024]; the tool-to-object distance stands for the arm motion still
needed to reach the object; and the holding term charges for fighting gravity. The weights
in Table 1 were set from how strongly each raw term correlates with the mechanical energy
of the executed trajectory, computed by rigid-body inverse dynamics over the full
gantry-and-arm chain, over a calibration set of picks; arm and rail-rotation travel track
it most consistently. We do not claim that J equals this mechanical energy exactly, only
that it is built to move in the same direction, and Section VI reports whether minimizing
J lowers the measured trajectory energy relative to the baselines [EnergySLR2024].

**V-B. Pooling candidates.**
For a target object, we gather from each arm's capability map the nodes whose task
position lies within a radius of the object, which serves as the reachability filter, and
score each of them with J. The radius scales with the node spacing of the map, so the
number of pooled candidates stays stable no matter how densely the map was grown. Because
all four arms have separate maps, the pool holds candidates from every arm at once, each
carrying its own base placement and full configuration, so scoring the pool compares all
four arms — including arms on different rails — and their base placements on the same
scale.

**V-C. Round-robin selection and execution.**
We sort the pooled candidates by J and try them in order, but interleave the order across
the arms so that no single arm can consume every attempt before the others are tried; the
arm that holds the overall lowest-J candidate is given a small head start. For each
candidate in turn we solve inverse kinematics, seeded by the candidate configuration, to a
pre-grasp pose that stands off a fixed clearance above the top of the object with a
top-down orientation, and we ask MoveIt for a collision-free plan. The search is
plan-only, so the arm never moves during selection, and the first candidate that yields a
valid plan is selected (Fig. 5). When execution is enabled, only the selected candidate is
planned and executed, and if execution is aborted by a change in the scene the search
falls through to the next candidate. The arm that wins and the rail placement its
configuration implies are exactly the arm-and-base decision that this paper sets out to
make.

---

## Open items carried forward
- ¶4 contribution style: kept as "First/Second/Third" prose; user may still switch to
  a bullet list.
- E2 IK-SEEDING DROPPED (2026-07-16, user "E2-a"): the GNG seed gives NO IK benefit —
  it loses to a neutral zero seed on success and time in every regime (E2), and ties/
  loses even for the exact top-down grasp on its own node (E2-b). So ¶4 no longer claims
  "GNG-seeded IK" and ¶5 no longer previews an IK-seeding win. The GNG map is framed as
  the arm+base SELECTION substrate + workspace representation only. Do NOT reintroduce an
  IK-speed/success claim for GNG. (E2 kept in experiment_results.md as a negative result,
  not a paper table.)
- ¶5 workspace number updated 5×→"roughly four times" (E1 rerun at rail 2.0: 4.11× raw /
  4.03× ceiling-capped @res 0.05). The remaining ¶5 `[PLACEHOLDER]` (selection vs
  baselines) fills from E3.
- Title's "Energy-Aware" leans on the `traj_energy` validation in E3; E3 must show
  actual mechanical-energy reduction vs baselines.
- J weights/refs are code-synced but have drifted before — CURRENT (2026-07-16,
  gantry_reach_executor.py): weights w_gl=2, w_gr=12, w_arm=20, w_dist=3, w_hold=1,
  w_manip=1; refs ρ = median raw term over a 63-pick calib session (ρ_gl=0.95,
  ρ_gr=0.70, ρ_arm=6.0, ρ_dist=1.36, ρ_hold=2.90, ρ_manip=0.145). `hold` IS in J now
  (w_hold=1). RE-VERIFY these against the code before submission; V-A prose keeps them
  symbolic so numbers live in one place. NOTE: reachability_gng/README.md is stale
  again (weights re-tuned since last sync) — re-sync before submission.
- HONESTY CAVEAT (found in code comments, 2026-07-16): the weights were derived from
  Spearman correlation (ρ) between each raw term and measured `traj_energy`; even the
  best single term (arm travel) only reaches ρ≈0.31 (weak), because the URDF has no
  joint damping/friction and the rail axes are orthogonal to gravity, so this rigid-body
  J cannot capture real motor friction/stiction. V-A now hedges this ("built to move in
  the same direction," not "equals... exactly") — do not strengthen this claim in
  Section VI beyond what the E3 data actually shows.
- SCOPE CHANGE (2026-07-16, user-approved): paper now covers TWO gantries / FOUR arms
  (was one gantry / two arms). Title changed to "...Dual-Gantry Quad-Arm Ceiling
  Robot...". Revised: title, ¶1, ¶2, ¶4 (rail→"each rail"), Related Work "Multi-arm
  coordination" (was "Dual-arm coordination"), III-A (two rails, four arms, shared-vs-
  independent base nuance), III-B (two cameras cover both rails' shared corridor),
  IV opening (four maps, was two), V opening/V-B/V-C (four arms in the pool and
  round-robin, was two). E1 data so far (5.10x gain) is still valid — it was measured
  per-arm (arm_1) and generalizes to "a rail," not tied to arm count. E0 table (Section
  VI, not yet written) will need arm_3/arm_4 rows added when those maps are built.
  reachability_gng/README.md, memory (paper-plan.md), and any other paper-scope note
  should be re-synced to this 4-arm scope. Do NOT propagate this scope change into
  docs/IEEE-conference-Energy_Aware/energy_aware_selection.tex — that file is
  off-limits per explicit user instruction.
