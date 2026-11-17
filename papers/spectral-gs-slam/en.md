> **TL;DR** Real-time 3D Gaussian Splatting SLAM tracks the camera with either feature matching, which fails on blank walls, or G-ICP, which fails on flat floors. Spectral GS-SLAM runs G-ICP, checks the eigenvalues of its Hessian every frame, and adds a cheap ORB-feature pose estimate only along the weak directions. On TUM RGB-D "no structure" and "no texture" scenes it is the only real-time method that keeps tracking in both, at 40 FPS.

## Why this paper

This is a small, clean example of a pattern we will use again and again in robust SLAM: **add a second source of information only where the first one is blind**. Here the first source is geometry (ICP) and the second is image features, but the formula is sensor-agnostic. It also explains a subtle reason why Gaussian maps make ICP degeneracy *worse*, which matters if we ever put a 3DGS map on the car.

## The problem

**3DGS SLAM in one paragraph.** 3D Gaussian Splatting represents the scene as many small 3D Gaussians, each with a mean, a covariance (its shape), a color and an opacity, and renders them into photorealistic images. Early 3DGS-SLAM systems (SplaTAM, MonoGS) also *track* by rendering and comparing images. That is accurate but slow: 0.5–1.9 FPS in this paper's tests.

**Real-time systems swap the tracker** and inherit its weakness:

- **Photo-SLAM** tracks with ORB features (ORB-SLAM3). On a blank wall there are no keypoints, so tracking is lost.
- **GS-ICP SLAM** tracks with G-ICP on the Gaussians themselves (68 FPS). On a flat floor every surface has the same normal, so ICP cannot tell sideways motion apart. This is degeneracy: the 6×6 Hessian H of the registration has near-zero eigenvalues for "slide along the floor" and "rotate about its normal".

**The Gaussian twist.** The mapper keeps refining each Gaussian over many views, and it squeezes them flat: the thickness along the surface normal shrinks to a tiny ε while the sideways extent stays large. G-ICP weights each match by the inverse covariance, so a flat Gaussian gives a huge weight (1/ε²) along the normal and almost zero sideways. Great for "how far from the plane", useless for "where on the plane". On a scene dominated by one plane, H becomes extremely lopsided. The authors tried forcing Gaussians to stay thicker; it made both tracking and mapping worse, because the mapper and tracker then fight.

## Key idea

Leave the map alone and fix the tracker:

1. **Detect** the weak directions from the eigenvalues of the ICP Hessian.
2. **Inject** information from a feature-based pose estimate *only* into those directions.
3. **Gate** how much to inject with a planarity score computed for free from the Gaussian normals.

Strong ICP directions stay pure ICP, so in normal scenes the system behaves like GS-ICP SLAM.

![Figure 1. Left: in a textureless scene, Photo-SLAM loses tracking. Right: on a flat, posters-on-the-floor scene, GS-ICP SLAM becomes ill-conditioned and its trajectory (red) collapses. Spectral GS-SLAM (blue) follows the ground truth (green) in both. (From the paper.)](fig1.jpg)

## How it works

![Figure 2. Pipeline. Correspondences feed a degeneracy detector on the ICP Hessian. When it is ill-conditioned, information from feature matching (H_prior) is added to H_ICP, gated by the surface normals of the visible Gaussians; mapping is unchanged. (From the paper.)](fig2.jpg)

**1. G-ICP tracking.** Each Gaussian in the new frame is matched to its nearest map Gaussian. The residual is the difference of means, weighted by the summed covariances. Linearizing gives the Hessian H_ICP = Σ Jᵀ Σ⁻¹ J.

**2. Spectral detection.** Eigen-decompose H_ICP = V diag(λ1…λ6) Vᵀ. A classic test flags λk below an absolute threshold, which depends on scene scale and point count. This paper uses a ratio instead: direction k is weak if λmax/λk > τdeg, with τdeg = 200 fixed for every experiment.

**3. Planarity score.** Each Gaussian's normal is simply its thinnest axis, so no neighbor search is needed. The system computes the opacity-weighted covariance of the visible normals and turns its eigenvalues into a score η: high when one surface orientation dominates the view.

**4. Feature prior.** A lightweight frame-to-frame pose from ORB matches with a robust loss. If there are too few matches, the prior is rejected and set to "no motion" rather than trusting a bad RANSAC result.

**5. Injection.** For every weak eigenvector v_k:

H = H_ICP + Σ w_k · (v_kᵀ Ω_prior v_k) · v_k v_kᵀ

Read it as: "measure how much the feature prior knows along v_k, and add exactly that, only along v_k." The weight w_k = exp(−α(1 − η)) is a soft gate in [0, 1]: when the scene is clearly planar, the full prior goes in.

## Results

**TUM RGB-D structure/texture scenes**, ATE RMSE in cm (X = failed):

| Method | FPS | % tracked | NoStruct near | NoStruct far | NoTex near | NoTex far |
|---|---|---|---|---|---|---|
| SplaTAM | 0.54 | 100 | 4.49 | 9.49 | 2.27 | 4.54 |
| MonoGS | 1.89 | 100 | 11.86 | **4.10** | 26.91 | 10.39 |
| Photo-SLAM | 31.48 | 68.4 | **2.27** | 5.36 | X | X |
| GS-ICP SLAM | 67.90 | 41.0 | 194.93 | 116.16 | **1.55** | 6.07 |
| **Spectral GS-SLAM** | 40.14 | **100** | 7.79 | 16.85 | **1.55** | **2.19** |

- It is the only real-time system that survives both scene types.
- On "no structure", feature tracking is still about 3× more accurate (2.27 vs 7.79 cm). The authors accept this trade for not collapsing anywhere.
- Rendering on "no structure near": PSNR 22.61 dB vs 18.02 dB for GS-ICP SLAM, because the map is built on a sane trajectory.

![Figure 3. On the "nostructure near" sequence the condition number (middle, purple) stays above the threshold of 200 (red dashed line) for the whole run, so the prior is active throughout (top bar), and the planarity score (bottom) stays high. (From the paper.)](fig3.jpg)

**Ablations** ("no structure far" / "no texture near", cm):

- Full method: 16.85 / 1.55.
- Without the planarity score (hard eigenvalue switch only): 75.78 / 1.63.
- Without soft gating: 22.43 / 1.88.
- Feature prior alone: 45.29 / 103.67. Constant-velocity model as prior: 55.87 / 1.68. So the gain comes from the *combination*, not from either source alone.

**Cost in normal scenes.** On Replica the average error rises from 0.16 cm (GS-ICP SLAM) to 0.24 cm, because the detector briefly triggers in two rooms. On TUM fr3/office it rises from 2.46 to 3.82 cm, where the prior switches on and off often. A sweep of τdeg shows failures happen when the threshold is too high (the prior never triggers), not when it is too low.

## Limitations

- Small indoor handheld RGB-D sequences only (TUM, Replica). No large scenes, no vehicles, no LiDAR.
- It costs accuracy in ordinary scenes because of false triggers, and it is still well behind feature tracking in textured flat scenes.
- The paper shows how the prior enters the Hessian, but not how it enters the right-hand side of the update. The planarity score is given only up to a proportionality, and α is not reported.
- When both cues fail at once, the pose is locked in place. That is safe only if the camera is actually still; the authors suggest adding an IMU.

## DYROS lens

**The TUM "no structure" scene is basically a garage floor seen by our AVM.** Posters on a flat floor: rich texture, no geometry. A parking deck seen by the around view monitor looks the same: lines, arrows and slot numbers on a plane. On an open deck, the LiDAR side looks like GS-ICP SLAM here: walls are far, the floor dominates, and x, y and yaw go weak.

**The injection formula is the takeaway.** H = H_main + Σ w (vᵀΩv) v vᵀ is a few lines in any Gauss-Newton tracker. For us: LiDAR GICP as the main tracker, and an AVM-based or wheel-odometry relative pose as Ω_prior, injected only into the weak eigen-directions. Our weighted GICP already decides how much each AVM measurement counts; this paper adds the question of *which directions* need help.

**What would break.** τdeg = 200 was set on tabletop RGB-D data; a LiDAR Hessian in a 60 m deck has a very different scale, so expect to re-tune it. The ratio test also cannot see repetitive pillars, where the cost has several good minima instead of a flat one.

**What we could try in a week.** Log λmax/λk of our GICP along one garage loop to see where the decks and ramps cross a threshold. Then add the injection term with wheel odometry as the prior (no feature pipeline needed) and compare drift with and without it.

**Paper opportunity.** "Direction-selective fusion of AVM and LiDAR registration for parking garages": the AVM floor texture fills exactly the in-plane directions that a floor-dominated LiDAR scan leaves weak, and the planarity gate tells you when.

## Key terms

- **3D Gaussian Splatting (3DGS)**: a scene made of many 3D Gaussians that can be rendered fast into realistic images.
- **G-ICP**: generalized ICP; aligns local Gaussian distributions instead of single points.
- **Condition number**: λmax/λmin of the Hessian; here a direction is called weak when λmax/λk exceeds 200.
- **Weak (degenerate) subspace**: the eigen-directions of the Hessian with little information.
- **Soft gating**: a weight between 0 and 1 that blends extra information in gradually instead of switching it on or off.
- **ORB features**: fast binary keypoints used by ORB-SLAM; they need visible texture.
