> **TL;DR** SanD-Planner is a local planner that looks at four depth images and a goal, and lets a diffusion model generate eight B-spline control points instead of a list of waypoints. A hand-written ESDF critic then picks the safest of 16 candidates. Trained on only 500 expert runs (0.25% of what the NavDP baseline used), it reaches 90.1% success in a cluttered benchmark and 72.0% in photorealistic indoor scenes, and runs zero-shot on a real quadruped.

## Why this paper

Learned planners usually win by data: NavDP, the baseline here, used 200,000 trajectories and 32 A100 GPUs for a day. A lab like ours cannot generate or train on that scale for every new platform. This paper asks the opposite question: how much can you get from a few hundred demonstrations if you choose the output representation well? The answer is about what a learned planner should output, and about keeping the learned part honest with an explicit geometric check.

## The problem

**Local navigation** (also called local planning) means: given what the robot sees right now and a goal a few meters away, produce a short collision-free path, execute the first part, and replan. Here the robot has no map, only a depth camera.

**Imitation learning (IL)** learns this mapping from expert demonstrations. Two problems hold it back:

- **Data hunger.** Performance tracks demonstration count. Hundreds of kilometers of simulated driving is normal for state-of-the-art methods.
- **Bad output representations.** Most learned planners output a list of waypoints. A fixed number of waypoints at fixed spacing gives a short horizon (8 points × 0.2 m = 1.4 m), which makes the robot short-sighted. Interpolating cubic splines fix smoothness but are globally coupled: an error in a far point (where depth is noisy) bends the whole curve, including the part you are about to drive.

## Key idea

**Learn in B-spline control-point space, and leave safety to an explicit geometric critic.**

A **clamped cubic B-spline** is a smooth curve shaped by a few control points, like a flexible ruler pulled toward pegs. It has three properties that make learning easier:

1. **Compact.** Eight control points describe a path about 6 m long. The horizon grows without growing the output size.
2. **Smooth by construction.** A cubic B-spline is C² continuous (smooth position, velocity and acceleration), so the network does not need to learn smoothness from data.
3. **Errors stay local and bounded.** Each part of the curve depends only on 4 nearby control points (local support), so a noisy far point cannot shake the near segment. And the curve always lies inside the convex hull of its control points, so if each predicted control point is off by at most δ, the whole path is off by at most δ.

![Figure 1. Pipeline. Four depth frames become tokens through a ResNet-18 encoder; goal and previous heading become one token each; a 2-layer Transformer fuses them into a context that conditions the diffusion model. It denoises B-spline control points, and a critic picks the top trajectory; its initial heading is fed back to the next step. (From the paper.)](fig1.jpg)

## How it works

**Inputs.** The last four depth images, the relative goal (x, y, z) in the robot frame, and a **velocity token**: the heading direction of the plan chosen in the previous step.

**Encoding.** Each depth frame passes through ResNet-18. Features come from an intermediate layer to keep fine geometry such as stair edges. Goal and velocity tokens come from small MLPs. A two-layer Transformer fuses everything into the context C_t.

**Generation.** A conditional diffusion model (DDPM, a 1D U-Net over the 8 control points) samples K candidate control-point sets. The first control point is pinned at the robot (origin). In real-world runs, K = 16, and each cycle warm-starts from the previous solution, denoising only from step 6 of 10.

**Selection.** For each candidate, sample M points along the curve and score:

- **Safety**: an ESDF (Euclidean signed distance field, the distance to the nearest obstacle at every voxel) is built from the current depth image. The cost penalizes points closer than a safety margin, with a discount γ^j so that near points matter more than far ones (they replan before reaching the far part anyway).
- **Length**: total path length, against detours.
- **Goal**: distance from the curve's end to the goal.

The candidate with the lowest weighted sum is executed, and its initial heading becomes the next velocity token.

**Why the velocity token matters.** In front of a symmetric obstacle like a pillar, going left and going right are both valid. Without memory, the diffusion model samples both modes every cycle, the critic picks a different side each time, and the robot wobbles into the pillar. Conditioning on the previous heading keeps the candidates on one side.

![Figure 2. (a) An A* path and 8 waypoints at 0.2 m spacing cover only the first meter or so. (c) An 8-point interpolating cubic spline and (d) an 8-point B-spline, with the last four points randomly perturbed: the cubic spline swings wildly, the B-spline keeps its shape. (b) Mean displacement along the path: the B-spline stays near zero for the first 1.5 m. (From the paper.)](fig2.jpg)

**Training data.** 500 episodes from the PCT-planner expert in 10 simulated worlds (Matterport3D and Gazebo, including stairs and low obstacles). Each episode is cut into many random sub-trajectories and refit with 8 control points, which multiplies the labels. Depth is randomized for sim-to-real (RealSense-like noise, pixel dropouts, frame freezing to mimic latency). Training takes about 5 hours on one RTX 4080.

## Results

**Benchmark (InternNav, point-goal navigation; baseline numbers taken from the NavDP paper).** Success rate (SR) and SPL (success weighted by path length):

| Benchmark | Method | SR | SPL |
|---|---|---|---|
| ClutteredEnv (2020 episodes) | iPlanner | 84.8 | 83.6 |
| | ViPlanner | 72.4 | 72.3 |
| | NavDP | 89.8 | **87.7** |
| | **SanD-Planner** | **90.1** | 84.0 |
| InternScenes (4040 episodes) | iPlanner | 48.8 | 46.7 |
| | ViPlanner | 54.3 | 52.5 |
| | NavDP | 65.7 | 60.7 |
| | **SanD-Planner** | **72.0** | **63.7** |

Training cost: 0.5k trajectories and about 5 GPU-hours, against NavDP's 200k trajectories and 32 A100s for 24 h.

**Representation ablation** (same pipeline, 8 output points each, two scenes per benchmark):

- ClutteredEnv SR: waypoints 75.5%, cubic spline 75.0%, B-spline **93.0%**.
- InternScenes SR: waypoints 75.5%, cubic spline 72.0%, B-spline **83.5%**.

**Data scaling.** With 50 trajectories (10% of the data) SR is already 55.5%; with 250 it is 65.0%. SPL keeps climbing up to the full set (49.6% → 78.9%), so small data teaches "don't crash" and more data teaches "don't detour".

**Velocity token.** Average SR on two ClutteredEnv scenes rises from 86.0% to 93.0% with the token; failures without it cluster around pillars.

![Figure 3. Without the velocity token (top), candidates (blue) flip between bypassing the pillar on the left and right, the chosen heading oscillates, and the robot collides. With it (bottom), candidates stay on one side and the robot passes. (From the paper.)](fig3.jpg)

**Real world.** Unitree Go2 with a RealSense D435, planner on a tethered laptop with an RTX 4090 at about 10 Hz, tracked by MPC. No fine-tuning. Demonstrations include narrow mazes, cluttered offices, outdoor scenes, pedestrians and stair climbing. These are qualitative; no real-world success rates are reported.

## Limitations

- **Depth only.** Small, thin, or specular objects (glass, shiny surfaces) are hard; the authors say so.
- **No memory of the scene.** The ESDF is built from the current depth image, so anything outside the field of view is invisible to the critic.
- **The critic is a ranking, not a guarantee.** If all 16 candidates are bad, the best of them is still executed; the paper does not say what happens then.
- **Train/test mismatch in the velocity token.** During training it comes from the expert's own control points (privileged information), at test time from the robot's previous choice.
- **Baselines are copied numbers** from the NavDP paper, and NavDP still has the best SPL on ClutteredEnv.

## DYROS lens

**Where it fits.** The structure is generate-and-select: a learned model proposes many candidates, a hand-written geometric cost picks one. That is a close cousin of our IROS 2023 framework, where a network shapes the sampling distribution and a sampling-based planner does the checking. The new lesson is about representation: predicting B-spline control points instead of waypoints gave +8 to +18 points of success in their ablation, with the same network.

**What would break on our car.**

- **Nonholonomic constraints.** A cubic B-spline is smooth but has no curvature bound. A car needs curvature ≤ 1/R_min and, for continuous-curvature parking like TargetTree-RRT\*, continuous curvature. The convex hull property could help here: limits on the control polygon give conservative bounds on the curve.
- **Gear changes.** A parking maneuver with cusps is several curves joined at direction switches. A single clamped B-spline cannot represent a cusp; you would need one spline per segment plus a gear flag.
- **Sensing.** Our input would be AVM bird's-eye images or a LiDAR occupancy grid instead of forward depth, which is actually easier: an ESDF from an occupancy grid is standard.

**What we could try in a week.** Generate 500 parking episodes with our planner in simulation, fit each forward segment with 8 B-spline control points, and train a small diffusion model conditioned on a BEV occupancy image and slot pose. Compare waypoints vs B-spline control points as the output (their Table III protocol), with an ESDF critic on the occupancy grid.

**Paper opportunity.** A "nonholonomic SanD": diffusion over per-segment B-spline control points with a gear token, plus a critic that adds a curvature cost from the control polygon. Or use the diffusion samples not as final paths but as a sampling distribution for RRT\*, which is the line of our JKRS 2026 learned-sampling work. The velocity-token trick is also worth copying as is: condition the generator on the previous choice so that consecutive plans do not flip between modes around a pillar.

## Key terms

- **Clamped cubic B-spline**: a smooth (C²) curve defined by control points, that starts and ends exactly at the first and last control points.
- **Local support**: each piece of a B-spline depends only on a few nearby control points, so local errors stay local.
- **Convex hull property**: a B-spline curve always lies inside the convex hull of its control points, so control-point errors bound the curve error.
- **ESDF (Euclidean signed distance field)**: a grid that stores the distance to the nearest obstacle at every cell; used to score clearance.
- **SPL (success weighted by path length)**: success rate discounted by how much longer the path is than the shortest one.
