# Evaluation

Evaluation utilities and protocols belong here. Use `evo` for trajectory metrics after verifying timestamp association and alignment convention. Store generated trajectories and plots under `results/`, not in Git.

## Monocular trajectory

The completed RGB-only baseline uses
`configs/orbslam3/monocular_baseline.yaml`. Build/replay commands and measured
results are in the [monocular report](../docs/research/orbslam3_monocular_baseline.md).
To rescore and plot the preserved run using existing local assets:

```bash
.venv/bin/python scripts/evaluate_monocular.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_monocular.py
```

Pass `--run-dir <directory>` to evaluate another completed run. Primary poses
come from the optimized largest-map export, filtered by successful online
tracking state. The evaluator checks run/config/trajectory hashes and reuses
the loader's fixed all-RGB, one-to-one reference associations at 20 ms without
interpolation. It reports both SE(3) alignment without scale correction and
Sim(3) with one GT-fitted scale. Native monocular units remain arbitrary;
the Sim(3) score does not establish metric recovery.

RPE uses closest future pairs at 1.0 s ±20 ms and excludes pairs crossing any
missing submitted RGB timeline index, including missing reference matches. Tracking
states, initialization, exported coverage and reference coverage are reported
separately. Five-second local scale fits and path-length ratios are descriptive
GT-based diagnostics; path-length ratios include jitter and are not scale-drift
estimates. This single replay supplies no independent-replicate confidence
intervals. JSON/CSV, aligned trajectories, CLI ATE checks and PNG/PDF figures
are under `results/orbslam3_mono_fr1_xyz/run_001/evaluation/`.

## DA3-assisted trajectory and comparison

The completed integration is specified by
`configs/orbslam3/da3_rgbd_baseline.yaml`. It uses learned nominal-metre float32
depth with ORB-SLAM3's RGB-D interface, without GT depth calibration. Full
reproduction and limitations are in the
[DA3 RGB-D report](../docs/research/da3_orbslam3_rgbd_baseline.md).

```bash
.venv/bin/python scripts/evaluate_monocular.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_001
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_monocular.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_001
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/compare_trajectories.py
```

The evaluator supports both sensor modes despite its original filename.
`compare_trajectories.py` checks identical RGB/calibration/ORB inputs, refits each
method on common RGB/reference timestamps and verifies identical RPE pairs.
It writes separate common-set scores and PNG/PDF plots under
`results/trajectory_comparison_fr1_xyz/`; it preserves full-run evaluations.
For these runs the common set has 793 poses/733 RPE pairs, while full DA3
evaluation has 796 poses/736 pairs. Report exported coverage independently.
Sim(3) includes a GT scale fit. SE(3) compares monocular arbitrary numeric units
with DA3's nominal metre scale, so interpret their difference with that caveat.

## Sensor-depth control and process repeats

The [sensor-control protocol](../docs/research/sensor_depth_control_protocol.md)
uses three repeats per method on 792 identical sensor-matched RGB frames:

```bash
.venv/bin/python scripts/prepare_sensor_rgbd.py
.venv/bin/python scripts/run_sensor_control_study.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/evaluate_sensor_control_study.py
```

Build the control executable with the cached-container command in
[setup](../docs/setup.md) before replay. Existing completed/failed attempts are
preserved. `results/sensor_depth_control_fr1_xyz/` holds the exact balanced
schedule, all nine run directories, a study-wide common timestamp set,
individual/full/common scores, descriptive summary and PNG/PDF comparison.
The completed common set is 786 poses and 725 identical RPE pairs.

Subsetting preserves the original loader's GT associations. Intentional source
RGB omissions are shared inputs; missing exports/GT within the submitted
792-frame timeline still break RPE pairs. All attempts and every repeat are
reported. Observed process ranges are not scene-level confidence intervals.
See the [results report](../docs/research/sensor_depth_control.md).

## GT-prefix calibration diagnostic

The [prefix protocol](../docs/research/prefix_calibration_protocol.md) uses the
previously measured first-158-frame depth correction, followed by later-frame
pose scoring. The completed study is documented in the
[calibration report](../docs/research/da3_prefix_calibration.md).

```bash
.venv/bin/python scripts/prepare_prefix_calibration.py
.venv/bin/python scripts/run_prefix_calibration_study.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/evaluate_prefix_calibration.py
```

Preparation/replay use frozen provenance and separate cache/study paths.
Regenerate current scores with only the last command. The comparison uses
632 identical later matched poses and 572 identical RPE pairs, refitting all
alignments after excluding the prefix. Three new calibrated processes are
compared with nine preserved historical controls. Three pose-only rescaled
scores are derived from raw trajectories and are explicitly not extra replays.
The generic evaluator records any `--translation-scale` pretransform and requires
a separate output directory. Sim(3) invariance under pose-only scaling is checked.

The scalar uses GT sensor depth from the prefix; later GT world poses enter
benchmark alignment/scoring only. Applying the final prefix scalar retrospectively
to initialization makes this an offline diagnostic. It does not establish
causal online operation, independent-sequence validation or unanchored metric
recovery. Outputs are under `results/da3_prefix_calibration_fr1_xyz/`.

## Depth predictions

Depth evaluation is specified by `configs/evaluations/da3_tum_fr1_depth.yaml`.
It uses the already downloaded files and makes no new downloads:

```bash
HF_HUB_OFFLINE=1 .venv-da3-macos/bin/python scripts/cache_da3_depth.py --device mps
.venv/bin/python scripts/evaluate_da3_depth.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_depth_comparison.py
```

Use the configured CUDA environment and `--device cuda` on a future NVIDIA host.
Different package/device/source provenance requires a separate prediction cache.
The script verifies checkpoint hashes, caches canonical depth plus processed
intrinsics, and supports resuming matching verified cache entries. Every RGB
frame with a sensor-depth association is evaluated independently, regardless of
whether a ground-truth world pose is available.

Primary results convert canonical depth to metres with the processed camera
focal length, upsample predictions bilinearly to the original sensor grid, and
exclude only invalid/out-of-range reference pixels. Predictions are not clipped
to the reference interval. Missing predictions reduce reported coverage and
threshold accuracy. Absolute errors on the valid intersection are reported
alongside that coverage.

Reports distinguish unfitted metric scores from per-frame oracle GT scaling,
GT-anchored temporal calibration on the first 20% of frames, mask-erosion
sensitivity, and a fixed nominal-focal-length sensitivity check. Neither oracle
scaling nor temporal GT calibration is a deployable unanchored monocular method.
The one-sequence temporal split does not establish cross-sequence generalization.

Raw caches are under `predictions/`; JSON/CSV summaries and PNG/PDF plots are
under `results/depth_comparison_fr1_xyz/`. Quantitative findings are documented in
`docs/research/da3_depth_comparison.md` after evaluation.
