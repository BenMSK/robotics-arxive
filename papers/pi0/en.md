> **TL;DR** π0 takes a 3B vision-language model (VLM) that already understands images and text, and bolts on a 300M "action expert" that turns noise into a 50-step chunk of continuous robot actions with flow matching. Pre-trained on about 10,000 hours of data from 7 robot types and then fine-tuned on curated task data, it folds laundry from a crumpled pile, buses tables and assembles boxes, tasks earlier robot foundation models could not do.
>
> **Why it is a classic:** It fixed the template for vision-language-action (VLA) models that most 2026 work follows: VLM backbone, continuous action chunks from a diffusion-style head, and an LLM-like pre-train / post-train recipe. Its open weights also made "fine-tune a robot foundation model" a realistic option for academic labs.

## Why this paper

Every robotics lab now hears "should we just use a VLA?". π0 is the paper to read before answering. It takes Diffusion Policy's action chunks, swaps diffusion for flow matching, and puts both on top of an internet-pretrained model. For DYROS the relevant side is humanoid manipulation, and the honest question is which parts work at lab scale.

## The problem

Generalist robot policies face little data, poor generalization and brittleness. Two families of earlier models each solved half:

- **Earlier VLAs** (OpenVLA and similar) make a VLM output actions as discrete text-like tokens, one at a time. They inherit semantic knowledge, but token-by-token actions are slow and coarse: no action chunks, no high-frequency control.
- **Small imitation policies** (ACT, Diffusion Policy) produce smooth, dexterous action chunks, but are trained per task and know nothing beyond their demos.

## Key idea

Three ingredients:

1. **Start from a VLM.** PaliGemma (SigLIP image encoder + Gemma 2.6B language model) brings internet-scale knowledge of objects and language.
2. **Add an action expert trained with flow matching.** Robot state and action tokens go through a separate, smaller set of transformer weights (300M) that attends to the VLM's tokens. It outputs a chunk of H = 50 continuous actions.
3. **Pre-train broadly, post-train narrowly.** Like LLMs: diverse, imperfect data teaches broad skills and recoveries; a smaller, high-quality dataset teaches one task done fluently.

**Flow matching** in one paragraph: it is a close cousin of diffusion. Mix the true action chunk A with noise ε as Aτ = τA + (1 − τ)ε. Train the network to predict the straight-line direction A − ε from any mixture. At run time, start from noise and take 10 Euler steps along the predicted direction. Same multimodal power as diffusion, simpler math, few steps.

## How it works

![Figure 1. The π0 model. Two or three camera images (each through a ViT) and the language command go through the pre-trained VLM (SigLIP 400M + Gemma 2.6B). The 300M action expert reads the robot state q and noisy actions, attends to the VLM tokens, and outputs the action chunk a(t)…a(t+H) after 10 flow steps. (From the paper.)](fig1.jpg)

**Observation.** 2–3 RGB images, a language command and joint angles. Images and state are projected into the same token space as words.

**One model, many robots.** State and action vectors are zero-padded to 18 dimensions (two 6-DoF arms, two grippers, a mobile base, a torso lift); missing camera slots are masked. The 7 configurations range from a single UR5e to bimanual and mobile manipulators.

**Data.**

- 903M timesteps of their own dexterous data (106M single-arm, 797M dual-arm), 68 tasks, where a "task" like bussing covers many objects.
- Open datasets (OXE, Bridge v2, DROID) make up 9.1% of the mixture.
- Post-training uses 5 hours for simple tasks and 100+ hours for the hardest.

**Speed.** The image and text prefix is computed once and cached; only the action tokens are re-run for each of the 10 flow steps. On an RTX 4090, one call takes 73 ms (14 ms images, 32 ms prefix, 27 ms for all 10 flow steps). The chunk is executed open-loop: 50 Hz robots re-plan every 0.5 s (25 actions), 20 Hz arms every 0.8 s. Temporal ensembling of overlapping chunks *hurt* performance.

**High-level policy.** For long tasks, a separate VLM splits "bus the table" into commands like "pick up the napkin".

## Results

All evaluations are on real robots, averaged over 10 trials per task, with partial-credit scores.

**Out of the box** (after pre-training only; shirt folding, two bussing tasks, grocery bagging, toast): π0 is best on every task, near perfect on shirt folding and easy bussing. OpenVLA (7B) and Octo (93M) were trained on the same mixture for fewer steps, so the authors also trained a "parity" π0 for 160k steps instead of 700k. It still beats both. Even π0-small (470M, no VLM) beats the 7B OpenVLA, so size alone is not the story.

**Language.** π0 follows step-by-step commands much better than π0-small; commands from a human help most, from the high-level VLM less.

**New tasks with 1–10 hours of data** (e.g. Tupperware in a microwave, paper towel replacement): π0 is generally best against ACT, Diffusion Policy, OpenVLA and Octo. Fine-tuning from pre-training beats training from scratch, sometimes by 2×. Tellingly, the strongest baselines were the from-scratch ones (ACT, Diffusion Policy): pre-trained OpenVLA and Octo did not benefit from their pre-training here.

![Figure 2. Post-training on long, multi-stage tasks (average task progress, 10 trials). Fine-tuned π0 (solid) vs the same model trained only on the task data (hatched) vs the pre-trained model without fine-tuning (outline). Top: tasks present in pre-training; bottom: tasks absent from it. (From the paper.)](fig2.jpg)

**Complex tasks.** Fine-tuned π0 reaches over 50% of the maximum score on all seven, including tasks that take 5–20 minutes. Reading the bars: laundry folding about 0.8 vs about 0.2 from scratch; box building about 0.6 vs about 0.3. On egg packing, training from scratch is about as good as fine-tuning, so pre-training does not help every task.

## Limitations

- **Not reproducible at lab scale.** The 10,000-hour dataset is proprietary, and pre-training ran for 700k steps on a 3.3B model.
- **Small, real-world evaluations.** 10 trials per task, custom scoring rubrics, mostly comparisons to the authors' own ablations on the hardest tasks.
- **Confounded comparison.** π0-small differs from π0 in both size and VLM initialization, so the VLM's exact contribution is unclear.
- **Open questions** (the authors'): which data helps, how much a task needs, how far transfer goes.
- **Scope.** Arms and mobile manipulators only. The authors explicitly leave autonomous driving, navigation and legged locomotion open.
- **Reactivity.** Chunks run open-loop for 0.5–0.8 s.

## DYROS lens

**What to take from it in 2026.** The VLA template: pre-trained VLM, a continuous action head (flow matching or diffusion), action chunks, and pre-train / post-train. With open weights (the openpi release), labs typically *fine-tune* a base model rather than pre-train one.

**Humanoids, realistically.**

- Fine-tuning on 1–10 hours of our own teleoperation data is within reach; the paper shows easy tasks learned with 1–5 hours. Inference fits a single 4090-class GPU at 73 ms.
- A humanoid is a new embodiment. π0's action vector stops at 18 dimensions, and none of its robots balance. Use the VLA for upper-body or end-effector targets and keep a model-based whole-body controller underneath, where DYROS's humanoid control expertise is the asset.

**Cars.** π0 does not cover driving, and a 3B VLM is heavy for an onboard parking computer. What transfers is the action-head idea: a small flow-matching head that outputs a trajectory chunk in about 10 cheap steps could replace a diffusion sampler in a learned parking sampler. The recipe also transfers: pre-train on many simulated lots with planner-generated data, post-train on our real garage logs.

**One-week experiment.** Fine-tune the released π0 base model on about one hour of teleop data for one humanoid upper-body pick-and-place task, and train Diffusion Policy from scratch on the same data. That asks the paper's fine-tuning question on an embodiment π0 has never seen: does pre-training still help?

**Paper opportunity.** A VLA whose actions are the inputs of a whole-body controller, measured by how little fine-tuning data a new humanoid needs.

## Key terms

- **VLA (vision-language-action model)**: a vision-language model extended to output robot actions.
- **Flow matching**: training a network to predict the velocity that moves noise toward data, then integrating it at run time.
- **Action expert**: π0's separate, smaller set of transformer weights for robot state and action tokens.
- **Cross-embodiment training**: training one model on data from many different robots.
- **Pre-training / post-training**: broad training on large, diverse data, then specialization on smaller, high-quality data.
