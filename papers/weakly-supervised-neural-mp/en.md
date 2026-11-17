> **TL;DR** H-NTFields learns a neural "travel-time map" T(start, goal): how long the best collision-free trip between two configurations takes. Planning then means rolling downhill on that map. Earlier physics-informed versions learned it from the Eikonal equation alone and got lost in multi-room buildings. This paper adds cheap, loose labels from a sparse roadmap (a lower and an upper bound on each travel time) and lifts success in 18 Gibson scenes from 76.8% to 90.8%, at about 0.1 s per query.

## Why this paper

Our own learning work (IROS 2023, JKRS 2026, LA3T\*) teaches a network *where to sample* so that a sampling-based planner finds paths sooner. This paper is a third flavor: learn a **cost-to-go** for every pair of configurations. A good cost-to-go is exactly what a planner wants as a heuristic or a sampling bias. The idea worth stealing is more general, though: let a cheap classical planner provide **bounds** instead of exact labels, and let physics fill in the details.

## The problem

Two families of neural planners:

- **Supervised** ones (e.g., MPNet) imitate expert paths. Generating enough expert paths in complex spaces is expensive.
- **Self-supervised, physics-informed** ones (NTFields and successors) need no expert data. They train T to satisfy the **Eikonal equation**, ‖∇T‖ = 1/S(q): the slope of the travel-time map at any point equals one over the local speed. The speed S is set from obstacle distance (slow near obstacles, full speed in open space), so the time map naturally keeps away from walls.

The catch: the Eikonal loss is *local*. It only checks that neighboring values are consistent with each other. In a building with many rooms, the network can find a map that is locally consistent everywhere yet globally wrong: a valley that ends in a dead room, or travel times that are too short because the field "leaks" through a wall. Adding more training samples does not fix it.

Think of a hiking map where every contour line agrees with its neighbors, but two mountains were stitched together in the wrong place. Each small patch looks fine; the route across the map is wrong.

## Key idea

Give the network a **skeleton**: a sparse roadmap whose graph distances say "the trip from here to there takes *at least* this long and *at most* that long." No exact labels are needed; the loss is zero as long as T stays inside the bounds. The roadmap fixes global topology (which room connects to which), and the PDE losses fix local geometry (how the field bends around a table leg).

## How it works

**1. Build a sparse roadmap by sphere packing.**

- Sample collision-free configurations and give each the largest obstacle-free sphere around it.
- Accept a new node only if it lies outside all existing spheres, so nodes spread out without redundancy (5,000 nodes per scene in the experiments).
- Connect nodes with straight-line collision checks.

**2. Make training pairs with bounds.** Pick two roadmap nodes and move each to a random point inside its sphere. The upper bound is the graph distance plus the two sphere radii; the lower bound is the graph distance minus them. A hinge loss L_R = max(0, T − T_ub) + max(0, T_lb − T) only fires when T leaves this corridor.

**3. Add physics losses** (from TD-NTFields):

- **Eikonal loss**: the gradient size of T must match 1/S at start and goal.
- **Temporal-difference loss**: a finite-step Bellman check, T(s, g) ≈ Δt/S + T(s, g + u·Δt), which spreads consistency beyond the infinitesimal.
- **Normal-alignment loss**: near obstacles, the gradient of T must point along the obstacle surface normal.
- **Causality weight** exp(−λT): short trips are learned before long ones, a built-in curriculum. The roadmap bounds follow the same "small values first" logic, which the authors say stabilizes training.

**4. Train** one network per scene (TD-NTFields architecture with PirateNets, AdamW, 2,000 epochs, batch 2,000).

**5. Plan with sampling-based MPC.** At each step, sample candidate actions from a Gaussian, roll them out, score them by predicted travel time, softmax-weight toward the best, and move one step (receding horizon). This is more robust than plain gradient descent on T, which can stall in a local dip.

![Figure 1. Learned speed (color) and travel-time contours around a goal in the middle of a Gibson scene. The full model (top left) closely matches the ground truth from the fast marching method (FMM, top right). PDE-only (bottom left) collapses into a wrong valley in the lower left; roadmap-only (bottom right) gets the global shape but misses obstacle detail. (From the paper.)](fig1.jpg)

## Results

**Setup.** 18 Gibson scenes: three-floor indoor houses with up to 19 rooms over about 700 m², stairs, doors and narrow passages. 100 random start–goal pairs per scene, 1,800 queries in total. Neural methods run on an RTX 3090 GPU, classical ones on the CPU, all with a KD-tree collision checker. Sampling-based planners get a 10 s limit.

**Ablation (50k training pairs).**

| Variant | Success | Time (s) |
|---|---|---|
| PDE only | 60.0% | 0.089 |
| Roadmap only | 68.5% | 0.105 |
| **Both (H-NTFields)** | **90.8%** | 0.115 |

**Data efficiency.** With 10k pairs H-NTFields already reaches 61.4% (PDE-only: 20.0%); with 20k, 82.5%; with 50k, 90.8%. More data barely helps (92.7% at 100k, 92.0% at 200k). PDE-only needs 200k pairs to reach 76.8%.

**Against other planners.**

| Method | Success | Time (s) |
|---|---|---|
| NTFields | 27.2% | 0.076 |
| P-NTFields | 37.2% | 0.076 |
| TD-NTFields | 76.8% | 0.081 |
| **H-NTFields** | **90.8%** | **0.115** |
| RRT-Connect | 98.8% | 1.165 |
| LazyPRM | 99.7% | 0.496 |
| FMM (grid) | 99.0% | 0.822 |

Path lengths are similar across all methods (0.68–0.75 in the paper's units). So H-NTFields is the best of the neural-field planners by a wide margin and 4–10× faster than the classical planners, but the classical planners still solve 8–9 more queries in 100.

![Figure 2. One cross-floor query in a Gibson house. H-NTFields (orange) finds a smooth path in 0.22 s; RRT-Connect (yellow) and LazyPRM (cyan) need 3.33 s and 3.99 s; FMM (green) takes 0.98 s and has grid-induced sharp turns; the earlier neural fields fail. (From the paper.)](fig2.jpg)

**Real robots.** A UR5e arm reaching into a packed cabinet: plan in 0.07 s. A Unitree B1 quadruped with a Z1 arm (10 DoF together) carrying a can around chairs in a kitchen: plan in 0.15 s.

## Limitations

- **Still less reliable than classical planners.** About 1 query in 11 fails, versus about 1 in 100 for RRT-Connect, LazyPRM or FMM. There is no completeness guarantee.
- **One network per scene.** Cross-scene generalization is explicitly left open. For a new building you train again, and training takes real GPU time (the paper reports about 2.7 minutes per epoch).
- **Holonomic, geometric planning.** The Eikonal model assumes the robot can move equally fast in every direction from every configuration. No dynamics, no nonholonomic constraints.
- **Hardware is not comparable.** Neural fields run on a GPU and the classical planners on a CPU.
- Small inconsistencies: the UR5e is called 6-DOF in one place and 7-DOF in another, and roadmap overhead is "a few seconds" in the text but 14–18 s in the roadmap column of Table II.

## DYROS lens

**Would it speed up or robustify a nonholonomic car planner in a narrow slot?** Not as the planner itself. The Eikonal model is isotropic: it assumes you can head off in any direction. A car's motion is strongly anisotropic. A slot 50 cm to the side looks "0.5 m away" to an Eikonal field, while the real answer is a three-point maneuver. In narrow parking, that anisotropy is the whole difficulty, so a field like this would give a misleading cost-to-go right where we need it most.

**Where it does fit.** A multi-level indoor garage looks a lot like a Gibson house: ramps instead of stairs, aisles instead of corridors. At the *garage scale* an isotropic travel time over (x, y) is a reasonable global guide, similar to the obstacle-aware 2D heuristic Hybrid A\* computes with Dijkstra on a grid. A learned field can be queried for any start–goal pair, in big batches on a GPU. One concrete use: the Rural Postman global planner (ITSC 2019) needs travel costs between many points on the parking roads, and a field like this could fill that table fast.

**The trick worth stealing: train with bounds, not labels.** For a car we have both bounds for free:

- Lower bound: the Reeds-Shepp length, which ignores obstacles and can only under-estimate.
- Upper bound: the cost of any path our planner (TargetTree-RRT\*) actually found.

A hinge loss between them can train a *nonholonomic* cost-to-go network T(x, y, θ | slot) without ever computing exact optimal costs.

**What we could try in a week.** In one parking map, sample a few thousand (start pose, slot) pairs, compute the Reeds-Shepp lower bound and run our planner once for an upper bound, and train a small MLP with the hinge loss. Use it as the heuristic or to bias sampling toward low-T regions in RRT\*. Measure time to first solution against the plain Reeds-Shepp heuristic.

**Paper opportunity.** "Learned nonholonomic cost-to-go for parking, weakly supervised by Reeds-Shepp lower bounds and planner upper bounds" connects directly to our learned-sampling line and fixes this paper's biggest gap for cars. Confidence (how tight the bounds were near a query) could set the learned-sample ratio, as LA3T\* does.

## Key terms

- **Eikonal equation**: ‖∇T‖ = 1/S; the travel-time field grows at a rate set by the local speed, like a wavefront spreading out.
- **Travel-time field / cost-to-go**: a function giving the minimum time (or cost) from one configuration to another.
- **Physics-informed learning**: training a network by penalizing violations of a known equation instead of fitting labels.
- **Weak supervision**: training with loose information (here, upper and lower bounds) instead of exact targets.
- **Sparse roadmap**: a small graph of collision-free nodes and edges that captures how regions of free space connect.
- **Sampling-based MPC**: model predictive control that samples many action sequences, scores them, and executes the first step of the best.
