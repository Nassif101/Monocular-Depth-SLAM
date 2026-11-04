# Evaluation methodology

## Comparisons

- Monocular ORB-SLAM3 (no learned depth)
- ORB-SLAM3 RGB-D-style adapter using fixed-conversion DA3 depth
- Oracle depth calibration as a **diagnostic upper-bound** only
- Fixed-budget and adaptive depth scheduling (future)

## Metrics

- **SE(3)-aligned ATE:** no scale fitting; metric-scale accuracy only when input
  translation units are established. Unscaled monocular numeric units remain arbitrary.
- **Sim(3)-aligned ATE:** trajectory shape after scale fitting
- Relative pose error at declared frame/time intervals
- Scale drift, tracking loss/recovery, inference count, GPU time, wall time

## Controls

Use identical image subsets, camera calibration, association tolerance and evaluation timestamps. Report missing/failed trajectories, multiple seeds where stochastic, and both successes and failures. No tuning on the test sequence without disclosure.

## Depth conventions

Document whether predictions are z-depth, range, inverse depth, or disparity; include valid masks, unit conversion, resizing/cropping and pixel alignment. A learned prediction must not be labelled metric without verification.

The first DA3/sensor comparison uses
`configs/evaluations/da3_tum_fr1_depth.yaml`: all matched frames, canonical-depth
conversion using the actual processed focal lengths, bilinear prediction
upsampling onto unchanged sensor pixels, and reference-only range/missing-data
masking. Equal-frame and pixel-pooled metrics are both reported. Per-frame
GT-fitted scaling and a GT-calibrated prefix/test split are separate diagnostic
results; neither is an unfitted metric score or independent-sequence validation.
See [depth findings](research/da3_depth_comparison.md).

The first [DA3 RGB-D protocol](research/da3_rgbd_protocol.md) fixes depth
conversion, invalid-pixel handling and upstream TUM1 RGB-D weighting before
trajectory collection. All 798 RGB frames receive learned depth; no GT depth
scale or reference mask enters SLAM. The initial
[comparison](research/da3_orbslam3_rgbd_baseline.md) additionally refits both
methods on identical evaluable RGB/reference timestamps and uses identical RPE
pairs, so different initialization coverage cannot change the scored sample set.
Full-run coverage and full-run metrics are retained separately. This is a
one-sequence feasibility comparison with one replay per method, without
independent-replicate uncertainty estimates.

The [sensor-depth control](research/sensor_depth_control_protocol.md) adds three
fresh processes per method on the same 792 sensor-matched RGB inputs. Native
sensor holes are retained, and sensor depth is explicitly confined to that
control. Scoring uses the intersection across all successful runs, with all
attempts/failures and individual scores retained. Submitted-timeline indices
distinguish intentionally omitted source images from missing estimates or GT;
only the latter break RPE pairs. Report process-level observed ranges as
descriptive variation on one sequence, without treating frames or repeated
processes as independent scene replications.

The [prefix-calibration protocol](research/prefix_calibration_protocol.md)
freezes the prior first-158-frame depth correction before collecting new
trajectories. It retrospectively applies that scalar to an offline 792-frame
replay and scores only later timestamps; calibration poses enter neither
alignment nor ATE/RPE. Historical controls are refitted on the same later set.
A separate pose-only translation rescale quantifies the mechanical scale effect
without inventing extra SLAM processes. Later GT poses are used for benchmark
alignment/scoring, while later sensor depth does not fit the correction. This
known-sequence, GT-anchored diagnostic is not independent validation or a causal
online scale estimator.
