> **TL;DR** A valet-parking car cannot see the whole lot, and spots keep changing as other cars come and go. This paper forecasts, with a Bayes filter, which spots will be free 5 s from now, treating "vacant now" and "occupied now" differently and reading the motion of nearby cars. A planner then parks, waits briefly for a spot about to open, or explores for information. In 50 randomized mall-lot runs it parks about 26% faster with shorter, smoother paths than earlier AVP methods.

## Why this paper

Every driver knows two tricks in a busy lot: if a car is reversing out, wait a few seconds and take its spot; if a car is creeping toward the free spot ahead of you, forget that spot. Planners that only look at what is free *right now* miss both.

This paper turns those tricks into a probability model and plugs it into a Hybrid A\* parking planner. It comes from the same Honda Research Institute group as the "Selecting Spots" paper in this list, which uses it as its main baseline.

## The problem

**Type-1 autonomous valet parking (AVP)**: the car parks itself in a lot with no infrastructure help (no lot cameras, no V2X). The ego car knows the static map (lot boundary, aisles, spot positions) but must observe occupancy itself, with limited onboard sensors.

What makes it hard:

- **Partial view.** Spots far away or out of view are unknown, and sensing gets less reliable with distance.
- **The lot changes.** Cars arrive and leave. A spot's status now says little about its status in 5 s.
- **Prior methods simplify.** Greedy methods use only current availability; a vanilla Bayes filter (InfPath) ignores other cars' motion and assumes uniform sensor confidence; exploration methods forget that the goal is to park.

## Key idea

Forecast **future** occupancy per spot with a Bayes filter whose prediction step depends on:

- **whether the spot is vacant or occupied now** (a vacant spot can be *taken*, an occupied spot can be *freed*), and
- **where nearby cars and pedestrians are going** (close to the spot *and* heading toward it).

Then let a planner choose between **park now**, **wait a little and then park**, or **explore**, all scored by one cost.

## How it works

![Figure 1. The simulated mall lot. The ego (green) sees spots inside the blue rectangle fully, those between blue and orange partially, and nothing outside orange. Red rectangles are predicted paths of other cars, the red circle a pedestrian, black boxes parked cars. (From the paper.)](fig1.jpg)

**Observation model.** The field of view (FoV) is a rectangle shifted ahead of the car. With their settings, a region about 25 m long and 11 m wide is fully trusted; out to 1.5× that size, the probability of reading a spot correctly decays smoothly from 1 to 0.5 (a coin flip); beyond that, nothing is observed.

![Figure 2. The pipeline. Perception gives spot observations and predicted motion of other agents. The spot occupancy estimator predicts and updates future occupancy probabilities. The strategy planner filters spots, computes and scores paths, or explores. (From the paper.)](fig2.jpg)

**Spot occupancy estimator** (prediction over T = 5 s in 0.1 s steps):

1. **Effect of other agents.** Each predicted position of another agent is a Gaussian whose covariance grows over time. The chance that agent j occupies spot s is a product of two terms: *proximity* (Mahalanobis distance to the spot) and *alignment* (is its predicted velocity pointing toward the spot?). A car passing by fast scores low; a car slowing and turning in scores high. Over all agents: q = 1 − Π(1 − p).
2. **Asymmetric prediction.**
   - *Vacant now*: the worry is that someone takes it. The occupancy belief rises with q.
   - *Occupied now*: the hope is that it frees up. If the parked car's predicted motion takes it out of the spot, the belief drops.
   - *Unobserved*: only slow background arrival and departure rates from a parking study (about 0.0006 and 0.0004 per second).
3. **Update.** Fully observed spots trust the current observation. Partially observed ones are fused by Bayes' rule with the distance-dependent confidence. Unobserved ones keep the prior.

**Strategy planner.**

1. **Filter spots.** Keep spots in view whose predicted occupancy in 5 s is low: ≤ 0.3 if vacant now, ≤ 0.7 if occupied now (a spot being vacated is accepted more easily).
2. **Plan and score.** Hybrid A\* to each candidate, in parallel, with static obstacles in the search and moving agents only in the cost. The cost adds travel time, waiting time, an exponential barrier on obstacle distance, and smoothness (acceleration, direction changes, reversing, gear changes). Hard clearances: 50 cm to cars, 90 cm to pedestrians, 20 cm to static obstacles.
3. **Wait-and-go.** Each path may start after a waiting time t_w ≤ 5 s. The path is checked against the other agents' predictions shifted by that delay, so "wait 3 s for the car to back out, then go in" is just another candidate.
4. **Explore** if no spot works. Three goals: straight ahead, turn left, turn right at the next aisle. Each is scored by the same cost minus the **information gain**, i.e. how much the trip would reduce the entropy of all spot beliefs.

![Figure 3. A car (red) leaves a spot near a vacant spot V. With Greedy or Identical prediction (top middle) the ego drives on and parks elsewhere; Identical prediction keeps the leaving car's spot near-occupied (orange curve). The asymmetric estimator (green curve) drops below 0.1 within about 2 s, and the ego takes the freed spot (top right). (From the paper.)](fig3.jpg)

## Results

**Setup.** A simulated shopping-mall lot with 8 rows × 12 columns of spots, more crowded near the entrance. Car model from a 2024 Honda Accord (4.97 × 1.86 m, wheelbase 2.83 m, up to 3.5 m/s). Other cars enter or leave spots on Hybrid A\* paths; pedestrians walk at up to 1.5 m/s. 50 randomized runs per method. Python on a desktop Xeon CPU.

**Against existing AVP methods** (means over 50 runs):

| Method | Path length | Parking time | Runtime per step | Smoothness cost |
|---|---|---|---|---|
| InfPath – Traversal | 42.4 m | 26.7 s | 0.059 s | 13.6 |
| InfPath – MCBFT | 35.7 m | 23.4 s | 0.281 s | 14.5 |
| Rule-OBCA (nearest spot + MPC) | 32.9 m | 25.2 s | 0.416 s | 20.5 |
| **Ours** | **24.0 m** | **17.2 s** | **0.045 s** | **3.5** |

- At least 26% shorter parking time and 27% shorter path than any baseline, with much smoother paths (average curvature 0.024 vs 0.061–0.077 1/m).
- Fast: 45 ms per planning step in Python, including Hybrid A\* to all candidate spots.
- Clearance stays reasonable: 8.6 m average distance to the nearest moving agent, 0.27 m minimum to static obstacles (Rule-OBCA keeps more: 0.34 m).

**Ablations.**

- *Estimator.* Greedy (no look-ahead) 23.7 s, Identical prediction (same rule for vacant and occupied spots) 22.6 s, Position only (no velocity alignment) 21.5 s, full estimator 17.2 s. The paper says the full estimator keeps "at least 2.3×" more distance from moving agents; that is true against Greedy and Identical (3.3–3.7 m vs 8.6 m) but not against Position only (8.3 m).
- *Strategy.* No waiting: 28.5 s and a smoothness cost of 32.7 (it explores and then backs up). Pure lawn-mower exploration: 32.5 s. Both lose by more than 39% in time and distance.

## Limitations

- **Predictions of other agents are assumed given.** No predictor is built; the estimator consumes perfect-looking forecasts from the simulator.
- **No occlusion.** The FoV is a rectangle that sees through parked cars. The follow-up "Selecting Spots" paper points this out and replaces it with ray casting.
- **Simple world.** Parallel rows only (three exploration goals: straight, left, right). Multi-level garages are future work. Other agents follow fixed paths and do not react to the ego.
- **Coarse Hybrid A\*** (5 steering × 5 speed values) and a fixed 5 s maximum wait, whatever the traffic.
- Simulation only, 50 runs per method, no real sensor data.

## DYROS lens

**Where it fits our stack.** The planner here is "Hybrid A\* to every candidate spot, then pick by cost". Two pieces transfer directly to a TargetTree-RRT\* pipeline:

- **Wait-and-go as a time shift.** Instead of replanning geometry when a car blocks the aisle, keep the continuous-curvature path and add a start delay t_w, checked against shifted predictions. This is a cheap way to give a geometric parking planner some dynamic behavior.
- **Asymmetric occupancy reasoning** as the slot-scoring layer above the planner: a freed spot near the ego may beat a free spot further away.

**What would break on our car.**

- **The observation model.** Their confidence depends only on distance inside a clean rectangle. Our slot observations come from AVM images, with fisheye distortion, short range, and pillars and parked cars in the way.
- **Spot identity and localization.** Spots are given in a known map, and the ego pose is assumed exact. In a GNSS-denied indoor garage with repetitive lines and pillars, matching an observed slot to a map slot depends on SLAM accuracy. Our weighted GICP AVM SLAM work addresses that error source.
- **Layouts.** Three exploration goals assume an open-air grid of rows. Underground garages with ramps and dead ends need a graph-based exploration (our ITSC 2019 Rural Postman work on visiting parking roads is the natural replacement for "straight / left / right").

**What we could try in a week.** Fit their observation model to real data. Record a few AVM drives in a parking lot, compare slot detections against hand labels, and plot detection accuracy versus distance and angle from the car. If it is far from their sigmoid, that alone shows the simulation results are optimistic and gives a calibrated model to plug back in.

**Paper opportunity.** Both HRI papers stop at simulation with ideal perception. A real-car AVP study in an actual garage, combining (1) an AVM-calibrated, occlusion-aware occupancy filter, (2) intention prediction for nearby cars, and (3) a fast continuous-curvature planner with wait-and-go, would fill that gap. DYROS has the car, the lots and the planner; the missing piece is mostly the occupancy layer described here.

## Key terms

- **Type-1 AVP**: valet parking done by the car alone, with no help from lot infrastructure.
- **Bayes filter**: a recursive estimator that predicts a probability forward, then corrects it with each new observation.
- **Field of view (FoV)**: the region the onboard sensors can observe.
- **Mahalanobis distance**: distance scaled by uncertainty; a spot inside the agent's uncertainty ellipse is "close".
- **Information gain**: the drop in uncertainty (entropy) of the spot beliefs expected from driving a path.
- **Wait-and-go**: a candidate plan that pauses for up to a few seconds before moving, to let another agent clear.
