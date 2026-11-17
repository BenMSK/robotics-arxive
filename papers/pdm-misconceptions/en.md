> **TL;DR** On the nuPlan benchmark, the authors show that predicting where a human driver will go (open-loop) and actually driving well (closed-loop) pull in opposite directions. A 20-year-old car-following rule, IDM, beats every learned planner they test in closed loop, and a lightly extended version (PDM-Closed) scores 92–93 out of 100. A tiny network that sees only the lane centerline gives the best open-loop score, and the combination (PDM-Hybrid) won the 2023 nuPlan challenge.

> **Why it is a classic:** It made the driving-planning community re-check its assumptions: open-loop imitation scores do not measure driving, and rule-based baselines belong in every table. PDM-Closed became the standard nuPlan baseline, and the later NAVSIM benchmark named its main metric the PDM Score.

## Why this paper

Our group builds planners that run on real cars, and we also use learning (IROS 2023, the Sci. Rep. 2022 imitation learning work). A common story is "rule-based planners do not generalize, so learn the planner from data." This paper tests that story on the first large real-world planning benchmark and finds it mostly wrong. Before we claim a learned component helps, this is the bar to clear.

## The problem

**nuPlan** is a data-driven simulator built from 1,300 hours of real driving, with 15-second scenarios, HD maps and perfect perception. It has three scores from 0 to 100:

- **OLS (open-loop score)**: the log is replayed, and the planner's 8-second forecast is graded against what the human did. This is **ego-forecasting**.
- **CLS-NR (closed-loop, non-reactive)**: the planner drives; other agents replay their logs.
- **CLS-R (closed-loop, reactive)**: the planner drives; other vehicles are controlled by IDM.

Closed-loop scores reward progress, time-to-collision, speed-limit compliance and comfort, multiplied by penalties: an at-fault collision or leaving the drivable area usually zeroes the scenario.

Most learned planners are trained as ego-forecasters: imitate the human's future trajectory. The question is whether a good forecast means good driving.

![Figure 1. The same nuPlan scenario. Top: IDM follows a different lane than the human (dashed black line), so its open-loop score is low, but it stays on the road. Bottom: the learned PDM-Open matches the human lane at t = 0, then compounds its own small errors and leaves the road by t = 3 s. (From the paper.)](fig1.jpg)

## Key idea

Treat the two tasks separately, and use the simplest tool for each.

- **Driving (closed loop)**: a rule-based planner with a little model-predictive flavor. Generate a few candidate trajectories, simulate them, score them, pick the best.
- **Forecasting (open loop)**: a small MLP that sees only the route centerline and the ego state.
- **Both**: let the learned model correct only the far part of the trajectory, which the controller never executes.

The base is the **Intelligent Driver Model (IDM)**: follow the lane centerline and set the acceleration by

dv/dt = a · (1 − (v / v₀)^δ − (s\* / s)²),

where v₀ is the target speed, s the gap to the lead vehicle and s\* the desired gap.

## How it works

![Figure 2. PDM-Closed (top) selects a centerline by graph search, forecasts other agents, builds IDM proposals, simulates and scores them, and picks one trajectory. PDM-Hybrid (bottom) feeds that trajectory, the centerline and the ego history into a small MLP (PDM-Offset) that corrects only the long-horizon waypoints. (From the paper.)](fig2.jpg)

**PDM-Closed** (driving):

1. **Centerline**: graph search on the lane graph along the route.
2. **Forecast**: other agents move at constant velocity, 8 s at 10 Hz.
3. **Proposals**: IDM with five target speeds (20–100% of the speed limit) × three lateral offsets (−1, 0, +1 m) = 15 proposals, each 4 s long.
4. **Simulation**: each proposal is tracked with an LQR controller and a kinematic bicycle model, as nuPlan itself does. So the planner scores what the car will *actually* do, not the ideal path.
5. **Scoring**: a function that mirrors the nuPlan metrics.
6. **Selection**: take the best proposal and extend it to 8 s. If even the best one collides within 2 s, brake.

**PDM-Open** (forecasting): the centerline (120 m at 1 m spacing) and 2 s of ego history go into an MLP with two 512-wide hidden layers. It outputs 8 s of waypoints, trained with an L1 loss on about 177k scenarios.

**PDM-Hybrid**: final waypoints = PDM-Closed waypoints + a learned offset, applied only after the correction horizon C = 2 s. The nuPlan controller uses only the first 2 s, so driving does not change, but the 8-second forecast becomes human-like.

## Results

The authors define **Val14**, 1,118 validation scenarios across the 14 leaderboard scenario types. Their submission scored almost identically on Val14 and on the leaderboard.

| Planner | Input | CLS-R | CLS-NR | OLS | Time (ms) |
|---|---|---|---|---|---|
| Urban Driver | polygons | 50 | 53 | 82 | 64 |
| GC-PGP | lane graph | 55 | 59 | 83 | 100 |
| PlanCNN | raster | 72 | 73 | 64 | 43 |
| IDM | centerline | 77 | 76 | 38 | 27 |
| PDM-Open | centerline | 54 | 50 | **86** | 7 |
| PDM-Closed | centerline | **92** | **93** | 42 | 91 |
| PDM-Hybrid | centerline | **92** | **93** | 84 | 96 |
| Log replay (human) | ground truth | 80 | 94 | 100 | - |

What stands out:

- **IDM alone beats the best learned planner** in closed loop (77 vs 72 CLS-R).
- **The trade-off is real.** Lowering IDM's maximum acceleration from 1.0 to 0.1 m/s² raises OLS from 38 to 48 but drops CLS-R from 77 to 54. For PDM-Open, the centerline lifts OLS from 72 to 86, while ego history barely helps OLS and lowers closed-loop scores.
- **PDM-Closed beats replaying the human log** in reactive closed loop (92 vs 80), partly because nuPlan's controller drifts from the logged trajectory and PDM-Closed scores the controller's actual output.
- **Ablations**: correcting all waypoints (C = 0) gives OLS 87 but CLS-R falls to 58. Removing the agent forecast costs 6 CLS-R points; removing lateral or longitudinal proposal variety costs 3–4.
- **Challenge**: a preliminary version (PDM-Closed + GC-PGP) ranked first of 25 teams with an overall score of 90.

## Limitations

- **No lane changes.** Half-finished lane changes are heavily penalized in nuPlan, so PDM never tries.
- **Privileged inputs.** HD map and perfect perception; no localization or detection error.
- **One simulator.** Non-reactive agents ignore the ego car (too aggressive); reactive agents are IDM and hug the centerline (too passive).
- **PDM-Hybrid games the metric.** It raises OLS without changing the driving, which is the authors' point, but it is a benchmark artifact, not a better driver.
- **No real-car test** in this paper.

## DYROS lens

**What a 2026 reader should take from it.** Three habits. Judge any learned planner in closed loop, never by displacement error alone. Always include a strong rule-based baseline. Score candidates *after* simulating them through the real controller and vehicle model.

**Where it fits our work.** PDM-Closed is really a sampling-based planner with a tiny, structured sample set: 15 trajectories from a rule family, simulated and scored. A well-chosen small set can beat a learned model. For our learned samplers (IROS 2023, JKRS 2026), the lesson is to report closed-loop success and planning time, not how well the network reproduces demonstrations.

**What would break on our car.** Parking lots and campus roads are semi-structured, and often have no lane centerline, PDM's backbone. Our closest equivalent is a parking-road graph like the one in our ITSC 2019 Rural Postman work. IDM has no reversing, gear changes or multi-point turns, so PDM-Closed is not a parking planner. And nuPlan's perfect perception hides exactly the localization problems our AVM/SLAM work deals with.

**What we could try in a week.** Build a PDM-Closed-style baseline for aisle driving in CARLA: follow the parking-road route, generate IDM proposals at several speeds and lateral offsets, simulate them through our tracking controller, and score. Compare it in closed loop with our sampling-based planner and our imitation-learning policy on success, collisions and comfort.

**Paper opportunity.** Replace PDM's hand-made proposals with samples from a learned trajectory distribution like ours, keep the simulate-then-score step, and test in narrow, unstructured spaces where centerline proposals do not exist. Whether open-loop accuracy of the learned distribution predicts closed-loop parking success is an open and easy-to-measure question.

## Key terms

- **Open-loop vs closed-loop evaluation**: grading a forecast against a replayed log vs letting the planner actually drive in simulation.
- **Ego-forecasting**: predicting the ego vehicle's own future trajectory, usually by imitating the human driver.
- **IDM (Intelligent Driver Model)**: a classic car-following rule that sets acceleration from speed and the gap to the lead vehicle.
- **PDM-Closed / PDM-Open / PDM-Hybrid**: the rule-based planner, the centerline-only forecasting MLP, and their combination.
