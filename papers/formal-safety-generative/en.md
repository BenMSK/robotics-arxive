> **TL;DR** Diffusion and flow-matching planners are too big for formal verification tools. SaGe-MP does not verify the planner at all: it verifies a tiny neural tracking controller (8–16 neurons per layer) that follows the planner's trajectory, and proves that the robot stays collision-free and reaches the goal from every start state in a set, under bounded disturbances. A library of such certified plans keeps the planner's multiple modes.

## Why this paper

Many labs now let a diffusion model output the trajectory directly, one step beyond our IROS 2023 idea of learning where to sample. Anyone who has tested on a real car asks: "how do you know the output is safe and drivable?" A diffusion model can produce a beautiful path that clips a pillar or asks for a curvature the car cannot follow. This paper gives one concrete answer that comes with a hard guarantee, not a statistical one.

## The problem

A **generative motion planner (GMP)** is a network (diffusion model, conditional flow matching, neural ODE, or even a vision-language model) that turns a start, a goal and some context (an image, a text prompt) into a reference trajectory. Two problems come with it:

- **Safety cannot be checked directly.** **Neural network verification (NNV)** tools such as CROWN compute guaranteed bounds on a network's outputs for a whole box of inputs. They work for networks with a few hundred neurons. A GMP has millions, and the bounds become useless.
- **Dynamic feasibility is not guaranteed.** The GMP outputs states, not controls, so the trajectory may be one the robot physically cannot follow.

Alternatives are weaker. Statistical checks (sampling, conformal prediction) give probabilistic guarantees and need huge numbers of rollouts for long horizons. Learned safety filters such as neural control barrier functions (CBFs) are only enforced at sampled states, so they can fail between samples.

## Key idea

**Keep the big planner as an open-loop "idea generator", and put all the guarantees on a small controller that tracks its output.**

Once a reference trajectory τ is sampled, it is just a fixed list of numbers. The closed-loop system "robot dynamics + small tracking controller following τ" is a small computational graph, even if the network that produced τ is huge. NNV can handle that small graph. So the question changes from "is the diffusion model safe?" (intractable) to "is this specific plan safely trackable from every start state in a box, under every allowed disturbance?" (tractable).

An analogy: you cannot audit the brain of the navigator who drew the route, but you can certify that your driver, following that route, will never leave the lane.

## How it works

![Figure 1. The GMP turns an image and a prompt into a reference τ. A small controller π_θ sees the errors to the next N reference points and outputs controls; the dynamics f are unrolled K times into one graph, and one-shot reachability gives a reachable-set over-approximation (RSOA). Certified plans go into a reference + RSOA library. (From the paper.)](fig1.jpg)

**1. A small, reference-conditioned tracking controller.** The controller π_θ is an MLP with leaky-ReLU activations and at most about 100 neurons (2 hidden layers of 8 or 16 in the experiments). Its input is the error to the next N reference points, e_k = [p_{k+1} − x_k, …, p_{k+N} − x_k]. One controller serves all references. It is trained on tracking error plus penalties for leaving state and control limits, by backpropagating through rollouts of differentiable dynamics.

**2. Unroll the closed loop into one graph (T-NFL).** The Tracking Neural Feedback Loop chains x_{k+1} = f(x_k, π_θ(e_k), w_k) for all K steps (K = 100 in the experiments). Inputs are a box of initial states X_I (an ε-ball around the start), a box of disturbances per step, and the fixed reference τ.

**3. Compute reachable sets.** The graph goes to auto_LiRPA (a CROWN-based tool), which returns affine lower and upper bounds on the state at every step. That gives a box at each time k that is guaranteed to contain every possible trajectory: a **reachable set over-approximation (RSOA)**. If every box is inside the safe set S and the last box is inside the goal set, the plan is certified for all start states in X_I and all disturbances. Splitting X_I into smaller partitions tightens the bounds.

**4. A library to keep multimodality.** Following one certified plan collapses the GMP's diverse behavior (e.g. going above or below an obstacle) into one mode. So, offline, they sample many GMP plans, discard those that collide or whose RSOA collides, and keep C certified (plan, RSOA) pairs. Online, they sample a fresh GMP plan as usual, pick the nearest certified plan in L2 distance, and track that one. A theorem shows the library converges to the GMP: C ≥ log(1/α)/p_samp samples keep the imitation error below ε with probability 1 − α (p_samp: the chance the GMP samples near a given plan).

**5. Large start regions.** A large X_I is gridded, with one certified GMP plan per cell. Checking which cells stay safe for a new obstacle layout then takes only 0.122 s (box-vs-obstacle checks).

## Results

**Why a small controller and formal tools (baselines):**

- Bigger controllers track better but make the RSOA bigger, because the bounds get looser. A neural ODE with three 256-wide layers (756 neurons) gives an RSOA that blows up even though the real trajectories stay bounded.
- Sampling misses rare failures: 10⁶ simulated rollouts of a 2D quadcopter found only 7 collisions, while one RSOA immediately showed the collision was possible.
- A neural CBF filter kept only 46.4% of 1000 unicycle rollouts safe; SaGe-MP certifies 100%.

![Figure 2. Planar quadcopter: certified reachable boxes (colored) around the reference (dashed) for (A) diffusion, (B) flow matching, (C) neural ODE with learned dynamics, (D) neural ODE with analytical dynamics. Every box clears the red obstacle. (From the paper.)](fig2.jpg)

**Planners and dynamics:**

- Unicycle with analytical and with learned MLP dynamics: safety and goal reaching certified in 101 s and 52 s.
- Planar quadcopter (6D): diffusion plans were safe 78.5% of the time open-loop and 94.4% when tracked by the controller without certification (126 samples); flow-matching plans 68.5% and 81.5% (108 samples). With SaGe-MP the whole start set is certified safe (100%).
- 3D quadcopter (12D), diffusion planner, sphere obstacle: certified, but RSOA computation took 2153 s (about 36 min).

![Figure 3. (A) Library building with a VLM planner: unsafe plans (red) and plans whose reachable set collides (purple) are thrown away; certified plans (green) are kept. (B) Running the VLM plans directly is safe only 16% of the time from the green start set. (From the paper.)](fig3.jpg)

**VLM and hardware.** Gemini 2.5 Pro produced waypoints from a scene image and a text prompt. In a unicycle simulation, naively tracking VLM plans was safe in 16% of 50 trials; the certified library gives 100% from the start set. On a real differential-drive robot, the same controller (trained only on neural-ODE plans) tracked VLM plans and stayed inside the computed reachable sets.

**Multimodality.** With a diffusion planner that goes above or below an obstacle, a library of only C = 2 already imitates the GMP well, and the imitation error falls as C grows.

## Limitations

- **Slow verification.** Minutes per plan for small systems, about 36 minutes for the 12D quadcopter. This is offline certification, not online replanning.
- **Static, known world.** The library is built for one environment and start region; the grid trick for new obstacles only scales to low dimensions.
- **The guarantee is only as good as the model.** It holds for the given dynamics f and the given disturbance bound. With a learned dynamics model, it certifies the learned model, not the real robot, unless model error is folded into the disturbance set.
- **Simple scenes.** One or a few circular obstacles, short horizons, no moving agents.
- **The 100% numbers are a certified start set,** not a success rate over many scenarios. The baseline percentages are a surrogate the authors chose.

## DYROS lens

**Where it fits.** This is the missing piece if we ever let a diffusion model propose parking trajectories. Our planners (TargetTree-RRT\*, the narrow-space planner) are trustworthy because every edge is checked for collisions and curvature; a diffusion planner gives no such story. SaGe-MP keeps the generator but executes only trajectories whose *closed-loop tracking* is certified. For a car: a kinematic bicycle model, a small tracking controller, a start-pose box from localization error, and a disturbance bound for tire slip and actuator delay.

**What would break on our car.**

- Parking is long-horizon with gear changes. A reference with a cusp (forward → reverse) is a discontinuity that a small error-tracking MLP may not handle, and the RSOA would need to cover each segment.
- Narrow spaces are the worst case for over-approximation. Boxes in (x, y, ψ) inflate fast with heading uncertainty; a 30 cm margin to a pillar may be impossible to certify with axis-aligned boxes.
- Computation rules out online use, but a parking lot is static enough to certify a library per lot or per slot offline.

**What we could try in a week.** Take a kinematic bicycle model, train an 8×8 tracking MLP on our TargetTree-RRT\* paths, and use auto_LiRPA to compute RSOAs for one forward-only parking maneuver with a ±10 cm, ±2° start box. That tells us how tight the boxes are for car-like dynamics before investing more.

**Paper opportunity.** Make the start set X_I come from SLAM: use the localization covariance (for example from our AVM + weighted GICP SLAM) to size the initial box, and certify only plans whose RSOA stays clear under that uncertainty. "Certified parking under localization uncertainty" joins our two lines, planning and robust SLAM, and the library idea fits naturally with a learned sampler like our IROS 2023 one proposing the candidates.

## Key terms

- **Generative motion planner (GMP)**: a network (diffusion, flow matching, neural ODE, VLM) that outputs reference trajectories from a start, goal and context.
- **Neural network verification (NNV)**: tools that compute guaranteed output bounds of a network for a whole box of inputs.
- **Reachable set over-approximation (RSOA)**: a set guaranteed to contain every state the closed-loop system could reach at a given time.
- **One-shot reachability**: bounding the whole K-step rollout as one graph instead of step by step, which gives tighter sets.
