> **TL;DR** ConsistencyPlanner is a learned driving planner whose decoder is a **consistency model**: a diffusion-style generator that jumps from noise to a trajectory in one network call. In the Waymax simulator it matches a 10-step diffusion version (collision rate 2.77% vs 2.76%) while running in 15 ms instead of 122 ms, and it has the lowest off-road and collision rates among the learned baselines it compares to.

## Why this paper

Diffusion planners are attractive because they can represent several driving options at once (yield or go, left or right). The price is speed: each trajectory needs many denoising passes through the network. Even with a fast solver, that is about 10 passes per sample. On an onboard computer, with a perception stack running at the same time, that budget is real.

This paper asks a narrow, practical question: can we keep the "diffusion-like" distribution and drop the iteration? For a lab that runs planners on real cars, and that has used a learned distribution to feed a sampling-based planner, a generator that is ≈8× faster per sample changes what is affordable inside a planning cycle.

## The problem

- **Rule-based planners** are interpretable but hand-tuned; they cope poorly with traffic that falls outside their rules.
- **Plain behavior cloning** (a network that regresses one trajectory) averages between modes. If half the drivers go left around an obstacle and half go right, the average goes straight into it.
- **Generative planners** (diffusion, autoregressive world models) represent multiple modes, but their iterative sampling is slow. Diffusion Planner, the reference here, is strong on nuPlan but needs many network evaluations per plan.

## Key idea

Train the decoder as a **consistency model**. A diffusion model learns to remove a little noise at a time, so sampling walks down a long staircase from pure noise to a clean trajectory. A consistency model learns a function F(x_t, t) that maps *any* point on that staircase straight to the bottom: every noisy version of the same trajectory should map to the same clean trajectory (the "consistency" condition). At test time you draw noise once and call the network once.

Analogy: instead of walking down a building floor by floor, you learn an elevator that goes to the ground floor from any floor in one trip.

The second, smaller contribution is the decoder design: a Transformer block that fuses the scene, the route and the noisy action, instead of concatenating everything into an MLP.

## How it works

![Figure 1. Framework. Left: the scene inside a field of view is tokenized (roads, route, other vehicles, ego) and encoded by a BERT-style scene encoder. Right: the consistency decoder fuses a route + timestep condition with the noisy action tokens and the scene feature through stacked Transformer blocks (AdaLN-Zero modulation, self-attention, cross-attention) and outputs the final action. (From the paper.)](fig1.jpg)

**Scene encoder** (same as PlanT / EasyChauffeur):

- Everything is expressed in the ego frame, inside a fixed field of view.
- Roads and the route are simplified with the Ramer–Douglas–Peucker algorithm into oriented rectangles [x, y, w, h, ψ, id]: 40 road rectangles (fixed 0.5 m width) and 20 route rectangles (ego width).
- Up to 127 other vehicles plus the ego car, each as a 6-number rectangle with velocity.
- A BERT encoder turns these tokens into one compact scene embedding.

**Consistency decoder:**

- A separate route feature (20 points of x, y, ψ, downsampled from the logged trajectory) goes through MLP-Mixer layers and is added to the timestep embedding to form a "fusion condition".
- The noisy action (future waypoints) is encoded into action tokens.
- 3 Transformer blocks (4 heads, width 256). The fusion condition modulates each block through AdaLN-Zero (scale and shift of the normalization), the action tokens attend to themselves, then cross-attend to the scene embedding (action = query, scene = key/value).
- The output is written as c_skip(t)·a_t + c_out(t)·F(a_t, t | s), the standard consistency-model parameterization that makes the output equal the input at zero noise.

**Training.** The loss is the L1 distance between the model's output on a noised expert action and the expert action, with noise levels from the Karras schedule (40 levels, minimum 0.002, maximum 80, ρ = 7). 20 epochs on 8 V100 GPUs.

**Inference.** One sample of Gaussian noise, one network call, and the result is executed.

## Results

Waymax closed-loop simulation on the Waymo Open Motion Dataset: 487,002 training scenarios, 44,096 validation scenarios, 8 s each at 10 Hz, other agents controlled by IDM. Metrics: off-road rate (OR, lower is better), collision rate (CR, lower is better), progress ratio (PR, higher is better; the authors cap it at 100% in their runs, and values above 100 come from baselines with extra route information).

| Method | Route | OR % ↓ | CR % ↓ | PR ↑ |
|---|---|---|---|---|
| Wayformer (waypoints) | logged + drivable futures | 7.89 | 10.68 | 123.58 |
| EasyChauffeur-PPO | logged | 3.95 | 4.72 | 98.26 |
| PlanT (reproduced) | logged | 2.29 ± 0.11 | 3.08 ± 0.02 | 95.38 ± 1.03 |
| LatentDriver | logged | 2.33 ± 0.13 | 3.17 ± 0.04 | 99.57 ± 0.1 |
| **ConsistencyPlanner** | logged | **2.09** | **2.77** | 93.72 |

Two ablations carry most of the message:

| Variant | OR % | CR % | PR | Inference |
|---|---|---|---|---|
| MLP decoder | 2.39 | 2.82 | 94.99 | – |
| Transformer decoder (ours) | 2.09 | 2.77 | 93.72 | 15 ms |
| Diffusion decoder, DPM-Solver 10 steps | 2.07 | 2.76 | 93.63 | 122 ms |

How to read this honestly:

- **The speed result is the strong one.** The one-step model is within 0.02 points of the 10-step diffusion version on every metric at about 1/8 of the time.
- **The safety gain over baselines is modest.** Against PlanT, which uses the same encoder with an MLP head, CR drops from 3.08% to 2.77% and OR from 2.29% to 2.09%. Baselines report ± std over three seeds; ConsistencyPlanner reports a single number.
- **Progress is lower** (93.72 vs 99.57 for LatentDriver). The authors trace this to conservative behavior at intersections.
- Wayformer-BC with a discrete action space has a lower OR (1.11%), but it uses extra route information ("drivable futures"), so it is not a fair comparison.

![Figure 2. Closed-loop examples, left to right: PlanT, EasyChauffeur-PPO, LatentDriver, ConsistencyPlanner. Top: right turn with an oncoming left-turning car; the first three collide (red ×). Middle: left turn with crossing traffic. Bottom: a tight U-turn where the others leave the road. (From the paper.)](fig2.jpg)

## Limitations

- **"Diverse set of trajectories" is not used.** The abstract stresses multimodal sampling, but the planner as described draws one sample and executes it. There is no candidate set, no scorer, no collision check. Multimodality lives in the distribution; the planner does not exploit it.
- **The route comes from the logged trajectory.** That tells the planner where the expert ended up, which makes the task easier. All the main baselines share this setup, so the comparison is fair, but real deployment does not have it.
- **Small margins, single run** for the proposed method.
- **Simulation only**, with IDM-controlled agents and no real-vehicle test. The 15 ms figure is not tied to a stated GPU.
- **Lower progress**; the proposed fix (guidance toward progress) is future work.
- The text has some slips (for example, OR is called "overlap rate" in one ablation, and the overview mentions "iterative denoising" for a one-step model), so read the tables, not the prose.

## DYROS lens

**Relation to our work.** Our IROS 2023 planner learned a distribution of committed trajectories and let a sampling-based planner sample from it. The bottleneck of that idea is always the cost of drawing samples from the learned model inside one planning cycle. ConsistencyPlanner does not address planning at all in that sense: it executes its sample directly. But its ablation is exactly the number we care about. A diffusion-quality generator that costs one network call instead of ten means we could draw tens of samples per cycle and let the planner check them.

So the honest comparison is: **"generate the trajectory"** (this paper) gives fast, smooth outputs with no safety check; **"learn where to sample"** (our approach) keeps the planner's collision checking and kinematic guarantees but needs the sampler to be cheap. A consistency model is a good candidate for that sampler.

**What would break on our car.**

- Waymax is road driving with forward motion only. No reverse gear, no parking slots, no tight clearances.
- Executing one unchecked sample in a narrow garage is not acceptable. One bad sample is a scraped bumper.
- The route from the logged future does not exist at run time. In a parking task the equivalent would be the target slot pose, which comes from slot detection or SLAM and is noisy.

**What we could try in a week.** Generate a dataset with our own planner in parking scenes (start pose, slot pose, occupancy → planned path). Train two small generators on it: a diffusion model with 10 DPM-Solver steps, and a consistency model with one step. Use each as the biased sampler for RRT\* and measure time to first solution, final path cost, and success rate in narrow scenes. The question is whether a cheaper but slightly blurrier sampler wins once planning time is counted.

**Paper opportunity.** "One-step generative sampling for real-time sampling-based planning in narrow spaces." The planner keeps all guarantees; the consistency model supplies multimodal samples (forward-in, reverse-in, multi-point turn) at a rate that fits our replanning loop. It connects directly to our learned-sampling line (IROS 2023, JKRS 2026) and to the T-ASE 2025 narrow-environment planner.

## Key terms

- **Consistency model**: a generator trained so that any noisy version of a sample maps to the same clean sample, allowing one-step generation.
- **Diffusion model**: a generator that turns noise into data by many small denoising steps.
- **DPM-Solver**: a fast ODE solver for diffusion models that cuts sampling to about 10 steps.
- **Waymax**: a closed-loop driving simulator built on the Waymo Open Motion Dataset.
- **AdaLN-Zero**: conditioning a Transformer by predicting the scale and shift of each layer normalization from the condition, initialized to zero.
- **Off-road / collision / progress rate**: Waymax metrics for leaving the drivable area, hitting another object, and distance covered relative to the logged route.
