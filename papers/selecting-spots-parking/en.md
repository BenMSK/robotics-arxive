> **TL;DR** When another car is hunting for a spot in the same aisle, guess which spot it wants from how it has been moving, and do not pick that one. This Honda Research Institute pipeline does exactly that with a learned intention model and a belief map of the lot. In 500 simulated lots it parks in 99.8% of runs against non-reactive drivers (about 90% for earlier pipelines) and never takes a spot another car was heading for.

## Why this paper

Our parking planners (TargetTree-RRT\*, the narrow-space planner) start once the slot is chosen. In a real valet-parking run nobody hands you the slot. The car enters, looks around and has to pick one while other drivers are doing the same. Pick the spot someone is about to back into and you block them, race them, or end up waiting in the aisle.

This paper is about that layer above the motion planner: **which slot to drive to**, given what the other cars seem to want. It also measures "social" failures (stolen spots, interrupted drivers), not only collisions.

## The problem

**Autonomous valet parking (AVP)**: drop the passengers, explore the lot, find a spot, negotiate for it with other vehicles, and park, with no human. The ego car knows the map (entrance, spot poses, roads) but not which spots are free or what other drivers intend.

Three things make spot selection hard:

- **Intentions are many and vague.** Any free spot is a possible goal, and drivers in a lot ignore lanes. A car creeping forward may be heading for the spot on its right, or pulling ahead to back into the one behind it.
- **Short-term motion is a weak hint.** The previous pipeline from the same group (Nawaz et al., the occupancy-aware AVP paper in this list) infers intention from *predicted future trajectories*, and its tests assumed those futures were known. Real parking predictors, trained on the one public dataset (Dragon Lake Parking, DLP: 3.5 hours, one lot), are themselves conditioned on an intention, so the logic turns circular.
- **The learned intention model needs data the ego does not have.** ParkPredict+ (trained on DLP) wants a 400×400 semantic bird's-eye view (BEV) image around the *other* car: roads, spots, parked cars, and the last 4 s of every moving car within 40 m. The ego only sees part of that.

## Key idea

Predict **explicitly** where each visible car wants to park, from its **past** motion, and fill in the parts of its surroundings you cannot see with a **belief map**. Then avoid the spots that someone else probably wants.

Think of a mall garage: a car slows next to a gap and its reverse lights come on. You do not need a precise forecast of its path to know that spot is taken. You read the intention from what it has already done.

## How it works

![Figure 1. Each timestep: the current observation and a belief map of which spots are taken are merged into a reconstructed BEV around each moving car. The intention model and trajectory prediction run on that, and the ego then parks, explores, or idles. (From the paper.)](fig1.jpg)

At every timestep:

1. **Belief map.** Each spot i has a belief bᵢ ∈ [0, 1]: the probability that another car is parked there or will park there soon. All spots start at 0.5. A spot seen vacant is set to 0, seen occupied to 1; spots out of view keep their old value.
2. **BEV reconstruction.** For each moving car the ego can see, draw the BEV it would need: known roads and spots, every observed car with its history, and, for every spot the ego cannot see, a default-size car (4.97 × 1.86 m) if bᵢ ≥ 0.5.
3. **Intention prediction.** Feed that image plus a few simple features (distance and heading to each candidate spot, speed, time in the lot) to the ParkPredict+ CNN. It outputs a probability η for each candidate spot and for "keep exploring".
4. **Belief update.** A vacant spot's belief becomes the chance that at least one moving car parks there: b = 1 − Π(1 − η).
5. **Trajectory prediction.** For every intention with η ≥ 0.3, draw a cubic Bézier curve from the car's pose to the spot, with the end tangent along the spot heading, and move along it at the car's current speed. Pedestrians get a constant-velocity guess.
6. **Decide.**
   - **Park**: candidates are observed vacant spots with b ≤ 0.3. Hybrid A\* plans to each one while staying out of spots believed occupied; plans that hit any predicted trajectory are dropped; the cheapest survivor wins.
   - **Explore**: otherwise, Hybrid A\* toward points where the aisles leave sensor range.
   - **Idle**: if nothing is safe, wait.

![Figure 2. The three approaches on one scene where the other car (yellow) pulls forward to back into a spot. Implicit: the ego only ranks spots for itself. Explicit, Future: intention from a straight-line forecast, which points to the wrong spot. Explicit, Past (this paper): intention from motion history, which matches the ground truth. (From the paper.)](fig2.jpg)

**Simulator.** The ego senses with 360 rays inside an 11.5 m radius, so parked cars block the view. Other agents follow scripted plans but brake when they foresee a collision with the ego within a "passiveness" of 2–6 steps. A non-reactive version never brakes, which is the harder test.

## Results

**Setup.** A small lot with 40 spots. One or two other cars spawn near the bottom spots and park with one of 8 human maneuvers seen in DLP. 500 random setups, each run with reactive and non-reactive agents, no pedestrians. Only the spot-selection rule differs between methods.

**Baselines.**

- **Explicit, Future** (Nawaz et al.): intention from constant-velocity trajectory forecasts.
- **Implicit** (ParkPredict+ on the ego only): ranks spots for the ego; ignores other drivers' goals.
- This pipeline with other trajectory predictors: constant velocity, Hybrid A\*, the ParkPredict+ transformer.

| Agents | Method | Success | Spots stolen | minADE / minFDE |
|---|---|---|---|---|
| Non-reactive | **Ours (Bézier)** | **99.8%** | 0.0% | 2.02 / 2.40 m |
| Non-reactive | Explicit, Future | 90.8% | 0.0% | 3.13 / 5.64 m |
| Non-reactive | Implicit | 90.4% | 0.2% | 2.77 / 5.00 m |
| Reactive | **Ours (Bézier)** | **100.0%** | **0.0%** | 2.02 / 2.39 m |
| Reactive | Explicit, Future | 98.8% | 4.6% | 2.90 / 5.28 m |
| Reactive | Implicit | 98.6% | 3.4% | 2.52 / 4.59 m |

What stands out:

- **Non-reactive drivers**: explicit, history-based intention adds about 9 points of success, and the final prediction error (minFDE) drops by more than half.
- **Reactive drivers**: everyone almost always succeeds because the others brake, so the difference is social. This pipeline never took a spot another car was heading for, and interrupted other drivers less (0.07 interrupted timesteps per run vs 0.41 and 0.21).
- **Simple beats fancy.** The Bézier predictor gives the best success of the four predictors and matches the learned transformer on accuracy (minADE 2.02 vs 1.82 m, minFDE 2.40 vs 2.46 m) while choosing spots about 10× faster (0.005 s vs 0.054 s).
- **Cost.** Spot selection adds only about 4 ms; Hybrid A\* (about 1–1.5 s per plan) is the real bottleneck. Parking time is similar for all methods (19–21 s).

## Limitations

- **Not real time.** Hybrid A\* needs about 1–1.5 s per cycle; 10 Hz is the stated target.
- **Simulation only.** Other drivers follow scripted paths with a simple braking rule. The lot has 40 spots, one or two other cars, and no pedestrians in the main tests. Two spare spots are always free, so success numbers are not a hard stress test.
- **Perfect perception within line of sight.** Vacant or occupied, parked or moving, and 4 s of history for each visible car are all assumed correct. No noise and no range falloff.
- The intention model ignores pedestrians, cars leaving their spots, and other vehicle types.

## DYROS lens

**Where it fits our stack.** This is the layer directly above TargetTree-RRT\*. Their pipeline calls Hybrid A\* once per candidate spot every cycle, and that is their bottleneck. The authors name faster planners, sampling-based ones included, as the obvious swap. A target tree suits many candidate spots well: the continuous-curvature entry paths for each slot can be built ahead of time, one RRT\* tree can grow toward all of them, and the belief bᵢ becomes a cost on each goal. That turns "plan to N spots, then pick" into one planning query.

**What would break on our car.**

- **Spot identity.** The belief map is indexed by spot. In an indoor garage the spot positions come from AVM slot detection plus SLAM. With repetitive pillars and parking lines, a small drift attaches a belief to the wrong slot. This is exactly where our AVM + weighted GICP SLAM work matters.
- **History of other cars.** The intention model needs 4 s of history for each moving car. Occlusion by parked cars and pillars cuts that history short, and an AVM alone only sees a few meters around the car.

**What we could try in a week.** The non-learned parts are cheap: belief map, Bézier predictor, and the park / explore / idle logic. Put them in our simulator, use TargetTree-RRT\* (or Biased Target-tree\*) as the Park planner, and measure planning time per cycle against their 1–1.5 s Hybrid A\*. Second check: record a few drives through a busy lot, label the slot each other car took, and see whether constant velocity or Bézier-per-intention would have guessed it.

**Paper opportunity.** "Belief-aware multi-goal target-tree planning": one sampling-based tree, one target tree per candidate slot, each slot's cost weighted by the probability that someone else takes it, at 10 Hz. Their open problem is planning speed, which is where our planners are strongest; a real-car run would go beyond anything in this paper.

## Key terms

- **Autonomous valet parking (AVP)**: the car finds a spot and parks by itself after the passengers get out.
- **Intention**: here, the spot another driver is going to park in (or "keep exploring").
- **Belief map**: a probability per spot that another car is in it or will be soon.
- **Bird's-eye view (BEV)**: a top-down image of the scene around a car.
- **minADE / minFDE**: the average / final position error of the best of several predicted trajectories.
- **Passiveness**: how many steps ahead a simulated driver looks for a collision before it brakes.
