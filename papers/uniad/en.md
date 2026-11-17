> **TL;DR** UniAD puts the whole driving stack (tracking, online mapping, motion forecasting, occupancy prediction and planning) into one camera-only network and trains all of it for the sake of the final plan. Modules talk through learned query vectors instead of boxes and lists, and on nuScenes the plan's L2 error and collision rate drop by about half compared to the previous end-to-end method.
>
> **Why it is a classic:** It won the CVPR 2023 Best Paper award and became the reference architecture for "modular but end-to-end" driving: most later end-to-end planners compare against it, and its open-loop numbers later became the center of a debate on how such planners should be evaluated.

## Why this paper

Our planners take a map, obstacles and a goal as given. UniAD asks from the other side: if the planner is what finally moves the car, which perception and prediction tasks does it need, and how should they pass information? Even if we never train a UniAD, its answer, "judge every upstream module by what it does for the plan", is a useful lens on our SLAM and parking stack.

## The problem

![Figure 1. Ways to build a driving stack: (a) separate models per task, (b) one backbone with separate task heads, (c.1) direct planning from pixels, (c.2) explicit but partial pipelines, and (c.3) UniAD, where tracking and mapping feed motion, motion feeds occupancy, and everything feeds the planner. (From the paper.)](fig1.jpg)

There were three common designs, and each has a weak spot:

- **Standalone modules** (a), as in most industry stacks. Easy to split across teams, but each module optimizes its own metric, information is lost at each interface, and errors pile up.
- **Multi-task learning** (b): one shared backbone, many heads. Cheap, but tasks can hurt each other ("negative transfer"), and the heads do not cooperate.
- **Direct end-to-end planning** (c.1): pixels in, trajectory out. Simple, but there is nothing to inspect when it fails and no safety handle.

Earlier explicit end-to-end systems (c.2) kept some intermediate tasks but skipped others. The question is which preceding tasks the planner needs, and how to wire them.

## Key idea

Design backwards from planning, and connect every module with **queries**: learned feature vectors that each module refines and hands to the next.

A bounding box says "car at (x, y), heading θ". A query carries that *plus* whatever else the network found useful about the car, so if the detector gets the heading slightly wrong, the motion module can still recover. The ego car gets its own query and takes part in the same interaction reasoning as every other agent.

## How it works

![Figure 2. The UniAD pipeline. Multi-camera images become a bird's-eye-view (BEV) feature B. TrackFormer and MapFormer produce agent and map queries; MotionFormer predicts every agent's future; OccFormer predicts future occupancy; the planner uses the ego query and the occupancy to output waypoints. (From the paper.)](fig2.jpg)

1. **Backbone.** Six surround cameras are turned into one BEV feature map with BEVFormer's encoder.
2. **TrackFormer.** Detection queries find new agents; track queries follow agents from earlier frames. Detection and tracking happen together, with no hand-made association step. One extra query represents the ego car.
3. **MapFormer.** Map queries segment lanes, dividers, crossings and drivable area.
4. **MotionFormer.** Each agent's motion query attends to other agents, to map elements, and to the BEV feature around its predicted goal point. It outputs the top-k future trajectories for all agents in one pass. During training, ground-truth trajectories are first smoothed for jerk and curvature, so the network is not asked to learn physically impossible targets from a noisy start.
5. **OccFormer.** Predicts a future occupancy grid while keeping each agent's identity, by letting every BEV cell attend to the agent that occupies it.
6. **Planner.** The ego query plus a navigation command (turn left, turn right, go straight) becomes a "plan query" that reads the BEV feature and outputs 3 s of waypoints. **At inference only**, the waypoints are then optimized with Newton's method: stay close to the network's plan, but get pushed away from cells the occupancy module predicts as occupied.

Training has two stages: 6 epochs for tracking and mapping, then 20 epochs end-to-end.

## Results

All results are on the nuScenes validation set. Planning is evaluated **open-loop**: the predicted trajectory is compared with the logged human trajectory, and the car is never actually driven by the model.

**Planning** (average over 1–3 s):

| Method | L2 (m) | Collision rate |
|---|---|---|
| ST-P3 (camera) | 2.11 | 0.71% |
| FF (LiDAR) | 1.43 | 0.43% |
| EO (LiDAR) | 1.60 | 0.33% |
| **UniAD** (camera) | **1.03** | **0.31%** |

That is 51.2% lower L2 and 56.3% lower collision rate than ST-P3, and it also beats the LiDAR-based methods on average.

**Which modules matter** (Table 2 of the paper):

- Planning alone: L2 1.13 m, collision 0.77%.
- Adding tracking, mapping and motion: 1.01 m, 0.72%.
- Adding occupancy too (full UniAD): 1.00 m, **0.43%**. Occupancy is what cuts collisions.
- A naive multi-task model with the same heads: 1.15 m, 0.94%.
- Tracking and mapping make motion forecasting 9.7% better in minADE.

**Per task:** motion forecasting minADE 0.71 m vs 1.15 m for PnPNet; tracking AMOTA 0.359 vs 0.294 for MUTR3D. The occupancy-based optimization step alone lowers the 3 s collision rate from 1.39% to 1.05%.

## Limitations

- **Open-loop evaluation.** L2 to the human log and collision rate on recorded data say little about closed-loop driving. Later work showed that on nuScenes, models that mainly use the ego car's own speed and history reach similar open-loop scores. The same shortcut is why Diffusion Planner drops ego velocity and acceleration from its input and reports closed-loop results.
- **Compute.** The authors themselves note it needs a lot of computation to train; lightweight deployment is left open.
- **Safety is bolted on.** Collision avoidance comes from an inference-time optimization against predicted occupancy, not from the learned policy.
- **Failures** in the paper: long-tail objects like large trailers, and over-cautious behavior on a dark, narrow street.

## DYROS lens

**What to take from it in 2026.** Two things. First, the template: sensors → BEV → agent and map tokens → joint prediction → planning query, which most later end-to-end driving models still follow. Second, the cautionary tale: open-loop L2 is not driving. Any learned planner we evaluate should be tested in closed loop.

**Is it realistic for us?** Training a UniAD-style model on our own car is not: it needs nuScenes-scale labels (boxes, track IDs, maps, occupancy for every frame) and many GPUs. The setting also misses what we care about: no parking maneuvers or gear changes, a 3 s horizon, commands like "turn left" rather than "reverse into slot 15", and pinhole cameras instead of AVM fisheye views.

**What we can adopt.**

- **Planning-oriented evaluation.** Judge our AVM + weighted GICP SLAM and slot detection by what they do to the parking planner's final error, not only by ATE or detection IoU.
- **Learned proposal, classical refinement.** UniAD's planner outputs a trajectory and then optimizes it against occupancy. This is the same "learning proposes, optimization or sampling disposes" split as our IROS 2023 planner.
- **Ego as an agent.** Passing the ego query through motion prediction is joint prediction-planning, useful for parking aisles with oncoming cars.

**One-week experiment.** In simulation, inject SLAM-like pose drift and slot-detection noise into our parking pipeline and measure the final parking error and the replanning rate. Find which perception error the planner is actually sensitive to.

**Paper opportunity.** "Planning-oriented SLAM for parking": tune or weight the SLAM back-end by its effect on parking success rather than trajectory error alone, which links both halves of our work.

## Key terms

- **BEV (bird's-eye view) feature**: a top-down feature map built from the surround cameras.
- **Query**: a learned feature vector that represents one agent, map element or the ego car and is refined across modules.
- **Occupancy prediction**: forecasting which grid cells will be occupied in the next few seconds.
- **Open-loop vs closed-loop evaluation**: comparing a plan to a recorded log vs letting the model actually drive in simulation.
- **minADE / AMOTA**: best-of-k average displacement error for forecasting / a standard multi-object tracking accuracy score.
