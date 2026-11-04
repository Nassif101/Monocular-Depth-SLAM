# Research log

## 2026-11-04

Initialized project scaffold for ORB-SLAM3 + DA3 baseline. No benchmark has been executed. Next: pin upstream revisions, prepare TUM RGB-D dataset loader and DA3 prediction metadata format.

Bootstrapped development dependencies on the Apple M4 Mac (16 GB) and Ubuntu
24.04 ARM64 Docker environment. Added locked Python environments, pinned official
ORB-SLAM3/Pangolin/DA3 sources, and tracked compatibility patches. Built ORB-SLAM3
monocular/RGB-D TUM executables; checked linkage, startup and headless Pangolin.
DA3-Small completed a random-weight forward pass on Metal. CUDA dependencies are
resolved for a future Linux/NVIDIA machine but have not been executed there.
No checkpoints, TUM data, trajectories or accuracy benchmarks were produced.
Next: TUM dataset loader and a pinned checkpoint, then the real-data baseline.

Implemented the TUM loader and 41 additional tests (43 total), passing on Mac
and Ubuntu ARM64. After reporting sizes and receiving explicit approval,
downloaded 1,784,939,566 bytes of TUM `freiburg1_xyz` and DA3 Metric Large assets.
All 798 RGB and 798 depth PNGs passed validation. At 20 ms, timestamp association
gave 792 RGB/depth pairs and 796 RGB/pose pairs while preserving all RGB frames.
Pinned checkpoint revision `4010e39f3634a45bc60553321fb49fb760bd594e` loaded and
ran on one real RGB frame using Metal. Saved raw canonical depth and provenance;
no sensor depth or ground-truth poses were passed to inference. No metre
conversion, accuracy evaluation or SLAM trajectory was produced.

Before running ORB, evaluated DA3 Metric Large on all 792 RGB/sensor-depth
pairs. Cached single-image predictions with processed intrinsics and provenance;
converted canonical depth to metres and upsampled predictions onto unchanged
640×480 sensor pixels. All 184,179,792 valid reference pixels had valid predictions.
Fixed-conversion frame means: AbsRel 10.224%, RMSE 0.1569 m, signed error
−0.1098 m. Median frame depth ratio was 0.8943. Per-frame oracle scale gave
3.485% AbsRel. A single scale fitted on the first 158 frames (1.109301×) reduced
held-out 634-frame AbsRel from 10.247% to 4.873%; this is explicitly GT-anchored
and is not independent-sequence validation. Mask/focal sensitivities, distance
bins, per-frame scores and PNG/PDF figures were produced. All 56 tests pass.
See `docs/research/da3_depth_comparison.md`. No ORB trajectory has been generated.

Subsequently built a project-owned headless monocular runner and completed
`freiburg1_xyz` using all 798 RGB frames, with no depth or ground-truth input
to SLAM. Preserved `results/orbslam3_mono_fr1_xyz/run_001/`: 795 optimized poses,
three initialization frames, no later tracking losses, one map and 53 keyframes.
Of these poses, 793 have the fixed loader's ground-truth association. SE(3) ATE
RMSE is 0.049830 m; Sim(3) ATE RMSE is 0.009283 m using GT-fitted scale
1.357883 m/native monocular unit. Sim(3) 1 s RPE RMSE is 0.014720 m and
0.956560 degrees across 733 pairs that do not bridge input/reference gaps.
Official evo ATE CLI results agree exactly with the evaluator. Exploratory
five-second local scale fits and jitter-sensitive path-length diagnostics are
reported separately, with the partial low-motion final window retained.
Tracked patches wait for ORB workers before optimized export; source snapshots,
hashes, logs and PNG/PDF figures are preserved. All 68 macOS tests and Ruff pass.
This single run does not establish metric recovery, repeated-run stability,
controlled real-time throughput or depth-assisted improvement. No additional
assets were downloaded. See `docs/research/orbslam3_monocular_baseline.md`.

Then recorded a fixed-conversion DA3/RGB-D feasibility protocol and inferred
the six RGB frames absent from the original sensor-matched cache, retaining
the original 792 predictions. Prepared all 798 predicted depth maps as aligned
float32 TIFF in nominal metres, without GT scale correction, clipping or
reference masking. Passed them to the existing ORB-SLAM3 RGB-D interface with
upstream TUM1 baseline/close-far parameters and depth factor 1. The first run
exported all 798 poses in one map with 76 keyframes, first-frame initialization
and zero later losses. All OpenCV depth decode checks passed.

Refitted/evaluated both methods on the same 793 RGB/reference timestamps and
733 one-second RPE pairs. Monocular vs DA3: SE(3) ATE RMSE 0.049830 vs
0.022218 m; Sim(3) ATE 0.009283 vs 0.011687 m; Sim(3) translation RPE
0.014720 vs 0.020116 m; rotation RPE 0.956560 vs 1.135417 degrees.
GT-fitted scales are 1.357883 vs 1.113338. Thus nominal scale is closer but
scale-corrected errors worsen in this single comparison. Full-run DA3 scoring
uses 796 poses/736 pairs and is reported separately. Official evo ATE checks
agree exactly. Preserved runs, common-set JSON/CSV, source snapshots and PNG/PDF
figures; 71 tests and Ruff pass. No new assets were downloaded. See
`docs/research/da3_orbslam3_rgbd_baseline.md`. General superiority, robust recovery,
repeated-run variability and end-to-end real-time operation remain unestablished.

Next completed the pre-recorded 792-frame sensor-depth control: three fresh
processes each for monocular, DA3 RGB-D and sensor RGB-D in a balanced Latin-square
order. Sensor PNG /5000 conversion preserves its 24.30% missing pixels, with
no hole filling or extra Freiburg correction. Both RGB-D methods use the same
compiled runner and settings. All nine runs completed, with one map and no
later tracking losses per run. RGB-D exports 792 poses every time; monocular
exports 788 after four-frame initialization. No new assets were downloaded.

On the identical study-wide 786 matched poses and 725 RPE pairs, median
monocular / DA3 / sensor SE(3) ATE is 3.501 / 2.245 / 1.066 cm; Sim(3) ATE
is 9.371 / 11.721 / 10.471 mm; Sim(3) translation RPE is
1.523 / 2.043 / 1.747 cm. GT-fitted scales are 1.2211 / 1.1139 / 1.0108.
Sensor depth narrows the DA3 gap and improves nominal scale, but monocular
retains the lowest scale-corrected errors here. These observed repeat ranges
are not independent-scene confidence intervals. Sensor-repeat-1 official evo
ATE checks agree exactly. All attempts, input/code hashes, full/common scores,
per-run CSV and PNG/PDF figures are preserved under
`results/sensor_depth_control_fr1_xyz/`. All 74 macOS tests and Ruff pass.
See `docs/research/sensor_depth_control.md`. Next: separately labeled prefix
calibration; weighting/independent-sequence experiments remain future work.

Then completed the frozen GT-prefix calibration diagnostic. Recomputing the
median of frame-median depth ratios on only the first 158 selected frames
exactly reproduces 1.1093010159475212×. Applied it to a separate 792-map cache
and completed three offline RGB-D replays; all export 792 poses, one map and
no losses. No new assets or inference. The fixed scalar is applied
retrospectively, including initialization, and is explicitly labeled GT-derived.

All three new runs and nine historical controls were refitted/scored on only
632 identical later matched poses and 572 identical RPE pairs. Raw vs calibrated
DA3 median SE(3) ATE is 2.302 vs 1.066 cm; Sim(3) ATE 11.387 vs 10.660 mm;
Sim(3) translation RPE 2.064 vs 1.960 cm. GT-fitted residual scale is 1.1162
vs 1.0007. Uniformly scaling existing raw DA3 poses by the frozen factor reaches
1.139 cm SE(3) ATE while leaving Sim(3) errors unchanged, verified per repeat.
Most of the metric gain is scale correction. Monocular remains lower in Sim(3)
ATE at 9.050 mm; sensor RGB-D is 10.140 mm on that same later set. These
same-sequence ranges do not establish general superiority or metric observability.
All 80 macOS tests and Ruff pass; calibrated-repeat-1 evo CLI ATE checks agree
exactly. All attempts, source/calibration hashes, per-case scores and PNG/PDF
figures are preserved in `results/da3_prefix_calibration_fr1_xyz/`.
See `docs/research/da3_prefix_calibration.md`. Next: isolate depth weighting and
point selection, then independent-sequence validation with download sizes first.
