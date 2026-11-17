> **TL;DR** FAST-LIVO2 (LiDAR + IMU + camera odometry) is very precise locally, but it drifts over long distances and can fail where both geometry and texture are poor. FAST-LIVGO adds raw GNSS Doppler and time-differenced carrier phase into the same Kalman filter. Its key trick is a referee: the smallest eigenvalue of the odometry Hessian decides whether odometry checks GNSS, or GNSS is trusted to rescue odometry.

## Why this paper

Most degeneracy papers ask how to fix a degenerate registration. This one uses the degeneracy signal for a different job: **deciding whom to trust**. That is the real question at the border of an indoor garage. Inside there is no GNSS; on the ramp out, GNSS comes back with multipath from the building, while LiDAR may be staring at a bare concrete wall. A filter that blindly fuses both, or always lets one veto the other, will break in one of those two moments.

## The problem

**Odometry drifts.** LIVO has no global anchor, so error grows with distance. The authors see it mostly in altitude (Z).

**GNSS is precise but dirty.** There are three raw signals:

- **Pseudorange**: the range to each satellite from signal travel time; noise at meter level.
- **Doppler shift**: frequency change caused by relative velocity; gives an instantaneous velocity constraint.
- **Carrier phase**: the phase of the radio wave itself; millimeter precision, but with an unknown whole number of wavelengths (integer ambiguity).

Near buildings, signals arrive by reflection (multipath) or only by reflection (NLOS, non-line-of-sight). Fusing those without checks pulls the trajectory off.

**The chicken-and-egg problem.** The usual way to reject bad GNSS is a chi-square test: predict the measurement from odometry and reject it if the difference is too large compared with the expected covariance. That only works if odometry is right. When LIVO is degenerate, its prediction is wrong, so the test throws away the *good* GNSS exactly when you need it. Turn the test off, and in a GNSS-hostile area multipath goes straight into a tightly coupled filter and can make it diverge.

There is also a timing problem: the GNSS receiver runs on its own clock. At 20 m/s, a 10 ms offset is already 20 cm.

## Key idea

Let the **degeneracy level of LIVO** pick the outlier-rejection mode.

During each LIVO update, take the smallest eigenvalue λmin of the Hessian (how well the worst-constrained motion direction is pinned down; a value near zero means the registration can slide). Average it over a sliding window so the mode does not flicker ("ping-pong").

- **Mode A, LIVO healthy** (average λmin ≥ λth): odometry is the judge. Every GNSS measurement must pass a chi-square test against the LIVO prediction. This removes even small multipath errors.
- **Mode B, LIVO degraded** (average λmin < λth): odometry is no longer a credible judge. Skip the chi-square test and accept every GNSS measurement that passed the receiver-level checks.

Think of two witnesses. When you can see clearly, you cross-examine the witness. When you are blindfolded, you check the witness's ID and then take their word.

## How it works

![Figure 1. System overview. LiDAR, IMU and camera run the FAST-LIVO2 update (point-to-plane residuals against a voxel map, sparse direct photometric residuals). GNSS goes through outlier rejection, DTW time alignment and yaw alignment, then enters the same error-state iterated Kalman filter; LIVO odometry feeds back into the GNSS checks. (From the paper.)](fig1.jpg)

**1. Base filter.** An error-state iterated Kalman filter (ESIKF) with FAST-LIVO2's LiDAR and visual updates. The state adds GNSS receiver clock offsets (one per constellation: GPS, BeiDou, Galileo, GLONASS) and the clock drift.

**2. Time alignment with DTW.** Speed magnitude does not depend on orientation. So the system compares the LIVO speed sequence with the GNSS Doppler speed sequence and aligns them with Dynamic Time Warping (DTW), which tolerates jitter better than plain cross-correlation. The best lag is the time offset.

**3. Spatial alignment.** A least-squares fit estimates the rotation and translation from the local odometry frame to East-North-Up, plus the antenna lever arm.

**4. Doppler update.** Each satellite's Doppler constrains the receiver velocity along the line of sight, plus clock drift. The antenna velocity includes ω × lever arm, since the antenna is not at the IMU.

**5. TDCP update.** Subtract the carrier phase of the same satellite at two consecutive epochs. The unknown integer cancels (if no cycle slip occurred), and atmospheric delays nearly cancel over a short interval. What is left is the change in range to that satellite, at millimeter level. The **fixed-anchor** trick: treat the previous posterior state as a constant, so the residual depends only on the current state. It then acts like a very precise odometry constraint, without keeping old states in the filter.

**6. Integrity checks before anything else** (RAIM-style): drop satellites with C/N0 below 35 dB-Hz or elevation below 15°, run a single-point least-squares fix, and drop satellites whose pseudorange residual stands out. Then apply Mode A or Mode B.

## Results

**M3DGR dataset** (RTK ground truth), ATE RMSE in meters:

| Sequence | RTKLIB | LIO-SAM-GPS | FAST-LIVO2 | LIGO | w/o rejection | **FAST-LIVGO** |
|---|---|---|---|---|---|---|
| Varying-illu03 (open sky) | 7.50 | 1.93 | 1.44 | 0.98 | 1.05 | **0.23** |
| Varying-illu04 | 7.97 | 1.01 | 0.35 | 0.26 | 0.28 | **0.19** |
| Varying-illu05 | 5.62 | 4.83 | 0.16 | 0.28 | 0.13 | **0.09** |
| Outdoor01 | 2.08 | 0.45 | 0.27 | 0.33 | 0.18 | **0.11** |
| Outdoor04 | 3.45 | 0.58 | 0.25 | 0.36 | 0.20 | **0.13** |
| Dark01 (GNSS-denied) | 16.39 | 1.85 | **0.31** | 2.45 | diverged | 0.34 |
| Dark02 (GNSS-denied) | 40.88 | 3.62 | **0.18** | 4.12 | diverged | 0.27 |

What stands out:

- In open sky, GNSS cuts FAST-LIVO2's error by roughly half or more (0.23 m vs 1.44 m on Varying-illu03).
- The **ablation without rejection** is the headline. It is fine in open sky but *diverges* in both GNSS-denied sequences: in a tightly coupled filter, unchecked multipath poisons the state and covariance. The looser LIO-SAM and LIGO do not crash but reach meter-level error.
- In GNSS-denied scenes FAST-LIVGO is slightly worse than pure FAST-LIVO2 (centimeters). The authors call this the price of fusion.

**Fixed-wing UAV** at about 75–80 m altitude and 20 m/s: qualitatively, FAST-LIVO2 drifts in Z and its map shows ghosting where areas overlap; FAST-LIVGO stays on the RTK track. No numbers are given.

![Figure 2. Handheld run mixing three failure types: a LiDAR-degenerate stretch, a camera-degenerate stretch facing blank walls (view 1), and an indoor corridor where GNSS is blocked (view 2, region d). Insets compare FAST-LIVGO (1) with FAST-LIVO2 (2). (From the paper.)](fig2.jpg)

**Handheld indoor-outdoor run:** in the indoor corridor (GNSS blocked), the rejection isolated bad GNSS and the map was as clean as FAST-LIVO2's. Outdoors, with weak geometry and blank walls, GNSS removed FAST-LIVO2's drift and ghosting.

**Runtime:** 25.99 ms for LIVO plus 11.78 ms for all GNSS work, 37.77 ms per frame in total, well below the 100 ms of a 10 Hz LiDAR.

## Limitations

- The threshold λth is never reported, and a single λmin mixes rotation and translation units. It also says *that* something is weak, not *which* direction.
- The switch is all-or-nothing. In Mode B every RAIM-passed measurement is trusted, so the worst case (LIVO degenerate *and* heavy multipath, e.g. an urban canyon) is left to RAIM alone. That combination is not tested.
- Quantitative results are only on M3DGR. The UAV and handheld results are figures and plots without numbers.
- TDCP assumes no cycle slip, and no slip detection is described.
- The ablation removes only the rejection module. The separate value of TDCP, Doppler and DTW is not measured.

## DYROS lens

**Inside the garage the GNSS half is useless, but the logic is not.** The pattern "let the degeneracy level decide who referees whom" transfers directly to our sensors. Replace GNSS with wheel odometry or with AVM parking-slot detections: when LiDAR is well constrained, gate those against the LiDAR prediction; when the LiDAR Hessian goes soft on a ramp or a long aisle, let wheels or AVM lead.

**The garage border is where this matters.** Driving out of an underground garage, odometry has accumulated drift, so a chi-square gate may reject the first good GNSS fixes, while the building causes multipath. A degeneracy-plus-drift aware gate is exactly what our campus-to-garage routes need.

**Make the referee directional.** Combining this with the per-direction analysis from the "Informed, Constrained, Aligned" entry is the obvious upgrade: in a long aisle only the along-aisle direction is weak, so only that component should be handed to the other sensor.

**What we could try in a week.** The DTW trick is cheap and useful on its own: align wheel speed with LiDAR-odometry speed (both orientation-free) to estimate the wheel-to-LiDAR time offset on our car. Then log λmin of our LiDAR registration along a route that enters and leaves a garage, and see where a fixed threshold would switch modes.

**Paper opportunity.** "Per-direction, degeneracy-aware integrity monitoring" for indoor-outdoor parking: GNSS outside, wheel odometry and AVM inside, the Hessian eigen-directions deciding which sensor owns which direction. FAST-LIVGO is a good baseline because it is the scalar, all-or-nothing version.

## Key terms

- **TDCP (time-differenced carrier phase)**: carrier phase differenced between epochs; cancels the integer ambiguity and gives millimeter-level displacement.
- **Doppler shift**: frequency change from relative motion; a direct velocity measurement per satellite.
- **Multipath / NLOS**: signals that arrive reflected, or only reflected; the main GNSS error source near buildings.
- **RAIM**: receiver autonomous integrity monitoring; consistency checks using only the GNSS measurements.
- **Chi-square gating**: reject a measurement whose residual is too large relative to its predicted covariance.
- **ESIKF**: error-state iterated Kalman filter, the estimator behind FAST-LIO and FAST-LIVO2.
