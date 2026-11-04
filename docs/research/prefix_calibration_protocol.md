# GT-prefix calibration diagnostic protocol

Recorded 2026-11-04 before collecting calibrated trajectories. This is a
GT-anchored diagnostic on the already inspected `freiburg1_xyz` sequence,
not a blinded or independent-sequence validation.

- Retain the same 792 sensor-matched RGB input frames and fixed loader GT pose
  associations as the sensor-control study. Preserve all earlier results.
- Use only the first 158 sensor-depth frames to reproduce the previously
  measured scalar: median of per-frame median GT/prediction ratios, over valid
  sensor pixels in the pre-existing 0.1–10 m interval. Prediction values are
  fixed-conversion DA3 z-depth on the original RGB grid. The expected frozen
  value is **1.1093010159475212**; fail on disagreement instead of choosing
  another value using trajectory results. Do not read later sensor pixels for
  fitting or use GT world poses to fit this depth correction.
- Multiply every DA3 prepared depth map by this one fixed scalar and replay
  the entire 792-frame sequence three times using the same RGB-D executable,
  camera/ORB parameters, RNG/thread controls and export policy. No per-frame
  correction, clipping, confidence filtering, parameter sweep or new inference.
- The factor is learned after observing the full prefix and then applied
  retrospectively to the replay, including initialization. This is an offline
  calibrated-prior experiment, not a causal online estimator that discovers
  scale before calibration observations arrive.
- Score only the following 634 selected RGB frames, starting
  `1305031107.643323`. The prefix ends `1305031107.611271`. Exclude prefix poses
  from alignment, ATE and RPE endpoints. A one-second RPE pair must lie entirely
  after the prefix and cannot bridge missing submitted poses/GT.
- Refit SE(3)/Sim(3) for each run on the same intersection of evaluable later
  timestamps across the three new runs and all nine preserved control runs.
  Held-out world poses enter benchmark alignment/scoring; held-out sensor depth
  does not enter the correction fit. These are distinct uses of reference data.
- Rescore all three uncalibrated DA3, monocular and sensor runs on that same
  later set. These controls are historical, not collected in a new balanced
  four-method schedule. Runtime and host-order effects are descriptive only.
- Additionally scale each existing uncalibrated DA3 trajectory's translations
  by the frozen scalar, without a SLAM replay. Label these three derived scores
  as pose-only rescaling, not three extra executed processes. Sim(3) metrics
  should remain numerically invariant under that uniform pre-scaling.
- Preserve every attempt and display individual results plus median/min/max;
  one scene and three processes per executed method do not support independent
  scene confidence intervals or significance tests. Report failures and coverage.
- Calibration may change RGB-D constraint weighting and point selection through
  the fixed virtual baseline / close-far threshold. A replay is not assumed
  equivalent to simply rescaling an already estimated trajectory.

The Experimental Design, Statistical Analysis and Scientific Visualization
skills inform these controls and reporting. The results report will acknowledge
and cite their software source.
