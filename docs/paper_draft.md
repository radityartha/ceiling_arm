# Paper Draft (fixed sections)

Working draft of *"Energy-Aware Arm and Base Selection for a Ceiling-Rail Dual-Arm
Robot Using a Base-Aware GNG Capability Map."* Only paragraphs the user has signed
off on live here. Citations use tags from [paper_references.md](paper_references.md);
`[PLACEHOLDER]` = experiment number pending; `[NEED CITE]` = source still to be found
(never fabricated).

Terminology (locked): single chain = "arm" (a 6-DOF manipulator); whole system =
"ceiling-mounted dual-arm service robot"; base = "ceiling rail" (one linear + one
rotational DOF).

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
[Jayathilaka2025]. We study a robot of exactly this kind. Two arms, each a 6-DOF
manipulator, are carried by a single ceiling rail that can both slide and rotate over
the workspace.

**¶2 — Specific problem.**
Because the rail moves the two arms as well, every arm paired with the rail becomes an
8-DOF redundant chain. A given object can usually be reached by either arm, and from
many different rail positions. The robot therefore faces two coupled questions at
once. Which arm should do the task, and where should the base be placed before it
reaches. Neither question is free of cost. Repositioning the heavy base is one of the
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
Second, we build a base-aware GNG capability map that treats the ceiling rail as a
genuine degree of freedom, and that enriches plain reachability with a per-node IK
seed, a manipulability value, and a holding cost. A boundary-seeding step keeps the
map accurate right up to the edge of the reachable workspace.
Third, we evaluate the whole approach in Isaac Sim, characterizing the map,
benchmarking GNG-seeded IK, and comparing energy-aware selection against nearest,
fixed, and random baselines. Open-vocabulary perception with YOLOE [Wang2025] provides
the perception front end, supplying the target objects that the selection pipeline
then acts on.

**¶5 — Results preview and outline.**
The simulation results support each claim. Letting the rail act as a degree of freedom
enlarges the reachable workspace by about five times compared with an arm-only
baseline. GNG seeding raises IK success and lowers solve time against
`[PLACEHOLDER: none/KDL/voxel]`, and energy-aware selection cuts base travel and
mechanical energy relative to the nearest, fixed, and random baselines by
`[PLACEHOLDER]`. All of these numbers come from Isaac Sim, and validation on the
physical robot is left for future work. The rest of the paper is organized as follows.
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
a time. They do not decide which of two arms should act, and they carry no notion of
the base travel that a given posture would require. We keep the inverse kinematics
solver itself lightweight and classical, and instead use the capability map to supply
a good starting configuration, while the arm choice and the base motion are settled by
the energy cost.

**Dual-arm coordination.**
Systems with two arms raise the question of how to divide the task between them. Recent
planners such as DAG-Plan and RoboPARA build dependency graphs over a task and assign
subtasks to each arm so that the two can act in parallel [DAGPlan2024], [RoboPARA2025].
Their concern is the temporal and logical structure of a multi-step task. Ours is
different and complementary. For a single reaching action we ask which arm reaches the
target for the least energy, given that both arms share one moving base, and we answer
it with the same map and cost that also place the base.

---

## III. System

**III-A. Robot platform and kinematic model.**
The robot consists of a single rail mounted to the ceiling that carries two arms side
by side. Each arm is a Kinova Gen3 Lite, a six-joint manipulator with a reach of about
0.76 m and a two-finger gripper at its wrist [Kinova]. The rail adds two more degrees of
freedom that both arms share, a prismatic axis that slides the carriage horizontally
and a revolute axis that rotates it, so relative to the ground each arm forms an
eight-degree-of-freedom chain. We write a configuration of one arm as
q = [d, θ, q₁, …, q₆], where d is the rail translation, θ the rail rotation, and
q₁…q₆ the arm joints. The kinematic tree is anchored at a fixed world frame. The rail
base is attached to the world, each arm base is attached to the rail, and a tool frame
at the gripper defines the end-effector pose that the task is specified in. Because the
two arms are mounted on the same carriage, moving the rail moves both of them together,
which is what couples the choice of arm to the placement of the base. Figure 1 shows
the platform and its frames.

**III-B. Simulation environment.**
All experiments in this paper are carried out in NVIDIA Isaac Sim, a GPU-accelerated
simulator with a physics engine suitable for contact-rich manipulation
[IsaacSim2026], [Mittal2023]. The simulated robot is built from the same URDF
description that defines the physical platform, so the kinematics and joint limits used
in simulation match the real hardware. The scene places a set of everyday tabletop
objects within the workspace, some within easy reach of both arms and at least one
deliberately positioned near the edge of the reachable workspace (Fig. 2). The scene is
observed by two RGB-D cameras placed above the workspace, which are the system's only
exteroceptive sensor. Their images serve two roles at once. The color stream feeds the
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
configuration incorporates the rail's degrees of freedom. We build one such capability map per arm, so the two
arms sharing the rail are described by two maps that we query independently in Section V.

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
end-effector can reach and a ready inverse-kinematics seed for reaching it. This is the
sense in which the map is base-aware. Because q carries the rail translation and
rotation, two ways of reaching the same point from different base placements settle into
different nodes, so a node's seed stays a valid single configuration rather than an
average of incompatible base positions.

**IV-C. Boundary seeding.**
A network grown only from interior samples tends to pull its outermost nodes inward,
because a node settles near the centroid of the samples it wins, so the node hull falls
short of the true reach surface. That surface is exactly where an object is most likely
to be judged just out of reach, so a shortfall there is costly. To prevent it, we pin a
shell of boundary nodes on the measured reach surface before growing the interior. The
pinned shell holds the outer extent of the map while the interior nodes fill the volume,
so the hull follows the true surface (Fig. 4). Section VI quantifies this shortfall and
the fidelity gained by boundary seeding.

**IV-D. Capability layers.**
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

## Open items carried forward
- ¶4 contribution style: kept as "First/Second/Third" prose; user may still switch to
  a bullet list.
- ¶5 numeric `[PLACEHOLDER]`s fill from E2/E3 once data is collected.
- Title's "Energy-Aware" leans on the `traj_energy` validation in E3 because `w_dist`
  (ee->object distance) is currently the leading J term (w_dist=25); E3 must show
  actual mechanical-energy reduction vs baselines.
