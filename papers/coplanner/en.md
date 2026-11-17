> **TL;DR** CoPlanner uses a diffusion model to generate the future of the ego car and nearby agents in two pieces: a short shared "trunk" (the next 4 s) that is checked first and then frozen, and several long branches (4–8 s) grown from it. Each ego candidate is scored against all the branches, not just the most likely one, so the car commits to a near-term motion that still works whichever way the scene unfolds. On nuPlan it beats the same diffusion backbone by 3.7 points (Val14 reactive, no post-processing) and reaches the best Val14 scores with a refinement step.

## Why this paper

Many learned planners now follow a "generate, then evaluate" recipe: sample many possible futures, score them, and execute the single best one. That last step quietly throws away everything the sampler learned about uncertainty. If the other car does the less likely thing, the planner has no plan B ready and switches plans abruptly.

Classical **contingency planning** has a known answer: keep a shared first segment that is safe in every future, and only branch later, once you know more. This paper brings that idea into a diffusion planner. It also touches a real-time replanning question we care about: which part of the plan do we commit to now, and how do we keep it stable across cycles?

## The problem

The paper names three failure modes of current learned planners:

- **Collapse to one future.** Multimodal generators produce diverse futures, but the evaluator picks the top one and drops the rest. No fallback is left.
- **Decoupled prediction and planning.** If the planner treats predicted trajectories of other agents as fixed inputs, the resulting joint scene can be socially inconsistent: the ego plan assumes a car will yield, the prediction assumes it will not.
- **Early divergence.** Without a shared short-term anchor, candidates split apart from the first timestep. From cycle to cycle the executed plan jumps between modes, which looks like hesitation or jerky control.

## Key idea

Split the 8 s future into a **shared short-term segment** S (0 to t_b = 4 s) and a **long-term branch** B (4 to 8 s), and model them in that order:

p(future | scene) = p(S | scene) · p(B | S, scene)

1. Sample a few joint short-term segments (for the ego car and the 10 nearest agents) and keep only those that pass a safety check.
2. For each kept segment, sample several long-term branches that all start exactly from it. The paper calls this the **pivot-conditioned diffusion mechanism**.
3. Score each ego candidate under every branch and use the average. The winner is a plan whose first 4 s are shared with good continuations in several futures.

An everyday analogy: approaching a car that might cut into your lane, you ease off a little now (the shared trunk), and keep two continuations in mind: keep going if it stays, brake if it moves over. You do not bet everything on one guess.

## How it works

![Figure 1. CoPlanner pipeline. Encoders turn agents, static objects, lanes and the route into tokens. A Diffusion Transformer first decodes a shared short-term segment, then completes N long-term branches from it. A contingency-aware scorer evaluates candidates across the branches and outputs the final trajectory. (From the paper.)](fig1.jpg)

**Inputs.** 2 s of history for the ego car and the 32 nearest agents within 100 m, the vectorized map, traffic lights and the route. Futures are predicted for the ego car plus the 10 nearest agents.

**Two diffusion models.** Both are Diffusion Transformers (DiT) that output the clean trajectory directly (x(0)-prediction), which the authors found more stable under heavy conditioning.

- G_shared generates the joint short-term segment on [0, 4 s].
- G_full generates the whole 0–8 s scene given a clean short-term segment. During training, noise is added only to frames after t_b, and the short-term condition is dropped 50% of the time so the model does not lean on it too much.

![Figure 2. Pivot-conditioned diffusion. At every reverse step the short-term frames are re-noised from the fixed anchor and written back, while the branch frames are denoised by the model; a mask merges the two. All N branches share the same short-term segment. (From the paper.)](fig2.jpg)

**Pivot-conditioned sampling.** This is the same trick image diffusion uses for inpainting. At each denoising step:

1. take the known short-term segment and add exactly the amount of noise that matches the current step;
2. let the model denoise the branch frames;
3. merge the two with a time mask (1 for t ≤ t_b, 0 after).

At the end, every branch starts with the identical 4 s segment. Sampling uses DPM-Solver++.

**Two-stage scoring.** The score is the PDM score, the rule-based nuPlan-style metric (collisions, time-to-collision, drivable area, comfort, progress) from the PDM planner.

- Stage 1: score each short-term segment over [0, 4 s]. Drop segments that score 0. If all of them score 0, trigger an emergency-braking profile for this cycle.
- Stage 2: within a surviving group, each ego branch is a candidate. Evaluate it with an 8 s PDM score against each of the N generated futures and take the mean. Pick the best candidate in each group, then the best group.

In the experiments K = 3 short-term segments and N = 3 branches per segment. The selected plan is projected onto a kinematic bicycle model and tracked by LQR. Training used about 1M nuPlan clips, 300 epochs on 4× RTX 4090 (about 2 days).

## Results

All numbers are nuPlan closed-loop scores (0–100), non-reactive (NR, logged agents) and reactive (R, IDM agents).

**Learning-only planners (no post-processing):**

| Planner | Test14-hard NR / R | Test14 NR / R | Val14 NR / R |
|---|---|---|---|
| PLUTO w/o refine | 70.03 / 59.74 | 89.90 / 78.62 | 88.89 / 78.11 |
| Diffusion Planner w/o refine | 72.39 / 61.21 | 87.33 / 75.51 | 86.87 / 75.30 |
| **CoPlanner w/o refine** | **76.82 / 64.47** | **90.31** / 78.81 | **89.48 / 79.00** |

**With a refinement module (hybrid):**

| Planner | Test14-hard NR / R | Test14 NR / R | Val14 NR / R |
|---|---|---|---|
| PDM-Closed | 65.08 / 75.19 | 90.05 / 91.63 | 92.84 / 92.12 |
| PLUTO | **80.08** / 76.88 | 92.23 / 90.29 | 92.88 / 76.88 |
| Diffusion Planner w/ refine | 78.87 / **82.00** | **94.80** / 91.75 | 94.26 / 92.90 |
| **CoPlanner w/ refine** | 76.05 / 78.59 | 93.47 / **92.00** | **94.45 / 93.13** |

What stands out:

- The clean comparison is against Diffusion Planner, which shares the same DiT backbone and training recipe. Splitting into trunk + branches adds 3.7 points on Val14 R (79.00 vs 75.30) and improves every sub-metric: collisions 96.14 vs 95.57, TTC 92.27 vs 91.50, comfort 88.14 vs 85.50.
- With refinement the margins are small (93.13 vs 92.90 on Val14 R), and on Test14-hard CoPlanner is behind Diffusion Planner. The authors blame their reliance on the PDM score, which is known to be less informative on that split.
- **Longer is not better.** Simply scoring with an 8 s PDM window instead of 4 s lowers the score (91.97 vs 92.95): fewer collisions, but comfort drops from 92.67 to 89.04 because the planner brakes for distant hypothetical conflicts. Scoring the trunk short and averaging the long part over futures fixes this (93.13, comfort 94.28).
- Under the same contingency score, independent sampling gets 92.89, re-anchoring every frame 93.03 (with jitter), pivot conditioning 93.13.

![Figure 3. Qualitative results. Top: under a shared short-term segment (green), several long-term branches (red) appear at merges, lane changes and unprotected turns. Bottom: a few seconds later, unlikely branches are gone and the set has converged. (From the paper.)](fig3.jpg)

## Limitations

- **No runtime numbers.** Each cycle runs K short-term samples, K·N branch samples and K·N·N full-horizon PDM evaluations. For a real-time stack this is the first thing we would want to know.
- **Fixed branching time** t_b = 4 s for every scene. A highway merge and a parking aisle need very different values.
- **Equal weights** over futures. A rare but dangerous branch counts the same as a common one; there is no likelihood or risk weighting.
- **Fallback is emergency braking.** When no short-term segment passes, the planner just stops.
- **Simulation only** (nuPlan, IDM agents). Feasibility comes from projecting onto a bicycle model after the fact, not from the generator.

## DYROS lens

**Relation to our work.** Our IROS 2023 framework learned a distribution of committed trajectories and let a sampling-based planner draw from it. CoPlanner sits on the other side of the same idea. It generates the trajectories directly with diffusion, and a rule-based scorer picks one. Feasibility comes only from post-projection and LQR. In our design the network only says *where* to look, and the planner owns collision checking and kinematics. Still, the two share a core concern: the near-term part of the plan must stay stable while the far part stays open. CoPlanner's "shared trunk, frozen during sampling" is a clean, learned version of that.

**What would break on our car.**

- Parking maneuvers are slow and long. 8 s at parking speed covers a few meters, and a multi-point turn needs reverse gear and cusps that nuPlan data barely contains.
- In a narrow aisle, "project onto a bicycle model and track with LQR" can turn a collision-free sample into a colliding one. We would need exact collision checks on the executed path.

**What we could try in a week.** Take the scoring idea without the diffusion model. In our sampling-based planner, keep the committed segment fixed and, for each candidate continuation, evaluate it against two or three predicted futures of a car in the aisle (stays parked / pulls out / passes). Compare mean-over-futures scoring to single most-likely scoring on replan stability (how often the executed trajectory flips) and on collisions in simulation.

**Paper opportunity.** "Contingency-aware learned sampling": learn the distribution of continuations *conditioned on the committed prefix* (the same masked-conditioning trick), sample from it inside our planner, and score across predicted futures. That keeps the planner's guarantees, keeps real-time, and adds the fallback branches CoPlanner shows are worth having.

## Key terms

- **Contingency planning**: planning a shared near-term motion with several later branches, one per possible future, so a fallback is ready.
- **Generation-then-evaluation**: sample many candidate trajectories, score them with an evaluator, execute the best.
- **Pivot-conditioned diffusion**: inpainting-style sampling that keeps a known short-term segment fixed while denoising only the later frames.
- **PDM score**: the rule-based score from the PDM planner that mirrors nuPlan metrics (collisions, TTC, drivable area, comfort, progress).
- **nuPlan Val14 / Test14 / Test14-hard**: standard closed-loop splits of the nuPlan benchmark; Test14-hard is a curated set of difficult scenarios.
