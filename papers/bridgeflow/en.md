> **TL;DR** BridgeFlow is a flow-matching trajectory generator that starts from a noisy straight line between start and goal and always plans in a frame with the start at the origin and the goal on the x-axis. That frame makes it exactly SE(2)-equivariant: rotate or shift the task and the answer follows. It reaches 100% success in 2 steps (7 ms for 40 candidates) on a dense 2D benchmark and keeps 96% on randomly rotated tasks, where diffusion and flow baselines fall to 40–52%.

## Why this paper

Parking is the textbook SE(2) problem: a slot on the north side of a garage and the same slot rotated 90° on the east side are the same problem. A learned planner trained in absolute map coordinates does not know that; it memorizes where things were. Our classical parking planners already work in the slot frame by habit. BridgeFlow puts numbers on why the same habit matters for *learned* planners and samplers, like those in our IROS 2023 and JKRS 2026 work.

## The problem

- **Diffusion planners** (e.g., MPD) produce good, multi-modal trajectories, but denoising takes many steps, and they do not respect rotations or shifts of the task.
- **Equivariant networks** build symmetry into the architecture, but they are heavy and slow at inference.
- **Flow matching** (e.g., FlowMP) is faster: it learns a velocity field that carries noise to data along nearly straight lines, so a few integration steps suffice. But it starts from plain Gaussian noise, which is far from any trajectory, so the field has to curve a lot. It also ignores obstacles during training and leans on cost gradients at test time.
- **Mini-batch optimal transport (OT)**, a common trick to straighten flows, pairs each noise sample with the closest expert trajectory in the batch. In planning, that can pair noise drawn for one task with an expert path from a *different* task, so the network is asked to learn contradictory targets.

## Key idea

Make the learning problem easier before the network sees it:

1. **Start close to the answer**: a Brownian-bridge prior, i.e., a straight line from start to goal with noise that is zero at both ends and largest in the middle.
2. **Pair sensibly**: OT matching only among samples of the same task.
3. **Remove the global pose**: translate and rotate every task into a canonical frame, run a normal U-Net there, and rotate the output back.

## How it works

**Brownian-bridge prior.** For waypoint i of N, with γ = (i−1)/(N−1):

x₀,ᵢ = (1 − γ)·start + γ·goal + σ·√(γ(1 − γ))·ε, with ε ~ N(0, I)

The flow only has to learn the *deformation* around obstacles. A short theorem shows lower expected transport cost than a Gaussian prior (given small noise and obstacles symmetric around the line on average). Because the noise is zero at the endpoints, the network learns zero velocity there, so start and goal stay fixed without "inpainting" them at every step.

![Figure 1. Initial samples (orange) and expert trajectories (blue) for one 2D task. Left: a standard Gaussian prior scatters samples everywhere. Right: the Brownian-bridge prior pins both endpoints and keeps samples near the straight line, so much less deformation is needed. (From the paper.)](fig1.jpg)

**Context-aware mini-batch OT.** Split the training batch by task (same start, goal and map) and solve the noise-to-expert assignment with the Hungarian algorithm only within each group (20 expert trajectories per start–goal pair in 2D).

**Classifier-free guidance on the map.** The occupancy map is sometimes replaced by a learned "null" token in training; at inference v = v_∅ + ω·(v_map − v_∅), ω ≥ 1. No cost gradients at test time.

**Task-centric canonicalization.**

- Heading of the task: θ = atan2(g_y − s_y, g_x − s_x).
- Map everything (waypoints, start, goal, occupancy map) through T_can(p) = R(−θ)·(p − s). The start lands at the origin and the goal at (|g − s|, 0).
- A standard U-Net predicts the velocity v̂ in this frame.
- Rotate it back: v = R(θ)·v̂.

Any rotated and shifted copy of a task lands in the *same* canonical frame, so the network sees identical inputs and the output transforms exactly with the task. The paper proves this, and that the bridge prior is equivariant too.

![Figure 2. Canonicalization. A task (start, goal, occupancy map) is shifted and rotated so the start is at the origin and the goal on the x-axis; a standard U-Net predicts the velocity field there; the velocity is rotated back to the world frame. (From the paper.)](fig2.jpg)

**Data.** RRT-Connect plus B-spline smoothing. 2D point mass: 200 dense maps (150 train, 50 unseen), 500 start–goal pairs × 20 trajectories, doubled by swapping start and goal. Franka arm: four regions A–D at the corners of a square table, training only on paths from A.

## Results

40 candidate trajectories per task. **SR** = at least one candidate is collision-free; **VR** (valid rate) = fraction of candidates that are collision-free.

**PointMass2D Dense, training maps.**

| Method | Steps | SR | VR | Time |
|---|---|---|---|---|
| MPD (diffusion) | 15 | 98% | 42.5% | 0.1128 s |
| FlowMP | 5 | 100% | 61.2% | 0.0138 s |
| FlowMP + standard OT | 5 | 100% | 58.4% | 0.0139 s |
| FlowMP + bridge prior | 5 | 100% | 66.1% | 0.0139 s |
| **BridgeFlow** | 2 | 100% | 70.3% | **0.0071 s** |
| **BridgeFlow** | 5 | 100% | **81.0%** | 0.0133 s |

- Standard OT alone *hurts* (61.2% → 58.4%), because of cross-task pairing. The bridge prior plus task-aware OT together reach 81.0%.
- The headline "15× faster" compares BridgeFlow at 2 steps with MPD at 15 steps. FlowMP at 2 steps is just as fast (0.0072 s); BridgeFlow's real gain is a much higher valid rate for the same number of steps.

**Unseen maps** (guidance added to every baseline for fairness): BridgeFlow 5 steps reaches SR 90%, VR 39.4%; MPD 15 steps 83%, 19.1%; FlowMP 5 steps 80%, 29.9%.

**Randomly rotated and shifted training tasks**: BridgeFlow 5 steps keeps SR 96%, VR 74.9%. MPD (15 steps) falls to SR 52%, VR 5.7%, and FlowMP (5 steps) to SR 40%, VR 4.7%. The baselines had memorized absolute coordinates.

![Figure 3. Three flow steps on a dense 2D map, in the world frame (top) and in the canonical frame (bottom). Samples start as a noisy band around the start–goal line and contract into a few distinct routes; yellow paths are collision-free, black ones collide. (From the paper.)](fig3.jpg)

**7-DoF Franka (simulation).** Training only on tasks from region A:

| Method | VR, seen (A → B/C/D) | VR, unseen regions | Time |
|---|---|---|---|
| MPD, 15 steps | 43.0% | 19.0% | 0.152 s |
| FlowMP, 5 steps | 65.0% | 21.0% | 0.038 s |
| **BridgeFlow, 5 steps** | **88.0%** | **80.0%** | 0.045 s |

BridgeFlow also gives the shortest paths. Canonicalization costs only about 7 ms over FlowMP.

## Limitations

- **Positions only.** The 2D agent is a point mass with no heading or kinematics, and the canonical frame comes from the start→goal direction.
- **Many invalid samples on new maps.** 60% of candidates collide on unseen maps (VR 39.4%); a collision check and selection step are required.
- **Less diverse.** Diversity at 5 steps (19.5) is well below MPD's at 15 steps (32.9).
- **The arm experiment is thin.** The paper does not explain how SE(2) acts on 7-DoF joint trajectories (presumably a base-joint rotation around a symmetric table): four regions, no real robot.
- Rotating a gridded map by arbitrary angles needs interpolation, so equivariance is exact for a continuous map and only nearly so for a grid.

## DYROS lens

**Canonicalization: yes, this would robustify a learned parking sampler.** For a car the natural canonical frame is the **slot frame** (origin at the slot, x-axis along the slot), or the start pose, not the start→goal direction. The car heading θ simply shifts by the frame rotation, and the same proof goes through. Any learned sampler we train for parking should take its inputs in the slot frame; this paper gives clean evidence for why (96% vs 40–52% success under random SE(2) transforms).

**Bridge prior: yes, with a car-shaped bridge.** A straight line is a poor prior for a parking maneuver with a gear change. The car analog is a bridge around a Reeds-Shepp path, or around a TargetTree continuous-curvature path, from start to slot: x₀(γ) = RS(γ) + σ·√(γ(1 − γ))·ε. The flow would then only learn how obstacles bend a nominal maneuver, which is far less than learning the maneuver itself.

**Speed: fast enough to run every cycle.** About 7 ms for 40 candidates is cheap enough to regenerate the sampling distribution at every replanning step. But samples are neither guaranteed collision-free nor kinematically feasible, so in a narrow slot they should feed TargetTree-RRT\* as biased samples, with LA3T\*'s confidence-based ratio deciding how many to trust.

**Bidirectional note (JKRS 2026).** The authors double their data by swapping start and goal. For a car this is valid only if the gear is flipped too (a path driven backwards is legal in reverse): the same time-reversal symmetry bidirectional parking planners use.

**What we could try in a week.** Take parking problems we already solve with our planner, express everything in the slot frame, and train a FlowMP-style 1D U-Net with an occupancy crop around the slot. Test on slots at other positions and orientations in the same garage, with and without canonicalization. Then plug the samples into our planner as the learned sampling distribution.

**Paper opportunity.** "Slot-frame flow matching with a Reeds-Shepp bridge prior as a sampler for narrow parking" combines all three ideas in a car-specific way, and the valid rate of each batch doubles as the confidence signal LA3T\* needs.

## Key terms

- **Flow matching**: a generative model that learns a velocity field carrying noise samples to data samples; a few ODE steps generate new data.
- **Brownian bridge**: random noise pinned to zero at both ends; here, a noisy straight line from start to goal.
- **SE(2)-equivariance**: if the task is rotated and shifted in the plane, the output is rotated and shifted in exactly the same way.
- **Canonicalization**: transforming every input into a standard frame before the network, then transforming the output back.
- **Mini-batch optimal transport**: pairing noise and data samples within a batch so that the total distance is minimal, which straightens the learned flow.
- **Valid rate (VR)**: the fraction of generated candidates that are collision-free.
