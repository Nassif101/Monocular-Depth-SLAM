# ORB-SLAM3 monocular baseline on TUM freiburg1_xyz

Date: 2026-11-04. Run: `results/orbslam3_mono_fr1_xyz/run_001`.
This is one completed RGB-only run on the existing TUM sequence. No additional
software, dataset or model assets were downloaded for this experiment.

ORB-SLAM3 produced 795 poses from 798 RGB inputs (99.624% coverage), initialized
on input index 3 (zero-based), and had no subsequent tracking-loss events. Position ATE RMSE
was **0.049830 m with SE(3) alignment** and **0.009283 m with Sim(3) alignment**.
The latter uses a ground-truth-fitted scale of **1.357883 metres per native
monocular unit**. It measures trajectory agreement after scale fitting; it does
not demonstrate recovery of absolute metric scale.

## Inputs and execution

The sequence is `rgbd_dataset_freiburg1_xyz`: all 798 original 640×480 RGB
images and their exact timestamps were submitted, without frame dropping.
The runner passes RGB images and timestamps to `TrackMonocular`; neither sensor
depth, DA3 predictions nor ground-truth poses enter SLAM. Ground truth is used
after the run for association, alignment and scoring.

The protocol is [monocular_baseline.yaml](../../configs/orbslam3/monocular_baseline.yaml).
The [camera settings](../../configs/orbslam3/tum_freiburg1_mono.yaml) use calibrated
RGB intrinsics `fx=517.306408`, `fy=516.469215`, `cx=318.643040`,
`cy=255.313989`, with distortion `(k1,k2,p1,p2,k3)` equal to
`(0.262383,-0.953104,-0.005358,0.002628,1.163314)`. Images are unscaled;
OpenCV-decoded BGR is explicitly converted to RGB for `Camera.RGB: 1`.
ORB settings are 1000 features, scale factor 1.2, eight levels, and FAST
thresholds 20/7. Upstream initialization uses its larger bootstrap extractor.
The viewer is disabled. OpenCV uses one thread; OpenMP/OpenBLAS thread limits
are one. OpenCV/C RNG seeds are zero, and the upstream two-view initializer
also seeds its RANSAC sampler. Asynchronous worker scheduling remains
nondeterministic; no bitwise repeatability or repeated-run variance is claimed.

Source and environment provenance:

| Component | Version or recorded setting |
| --- | --- |
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` |
| Pangolin v0.6 | `dd801d244db3a8e27b7fe8020cd751404aa818fd` |
| Host | Apple M4 MacBook Air, 16 GiB memory, macOS 26.6.2 |
| Execution | Ubuntu 24.04 ARM64 in Docker; CPU, headless |
| Docker runtime observed after the run | 10 virtual CPUs, 8,319,504,384 bytes available memory; engine 29.9.2 |
| Container image ID | `sha256:c1b2a4f1ea12e14077034e59ab69ec582a532b48d6025d3f176a66b89638c2a0` |
| C++ environment | GCC 13.3, OpenCV 4.6.0, Eigen 3.4.0, Boost 1.83 |
| Evaluation on macOS | Python 3.12.14, evo 1.38.1, NumPy 2.5.4, SciPy 1.18.1 |

The [run manifest](../../results/orbslam3_mono_fr1_xyz/run_001/manifest.json)
records the invocation, every RGB input hash, calibration/vocabulary/binary and
shared-library hashes, source revisions, local patch hashes, seed controls and
export policy. Exact runner/orchestrator/converter source snapshots and local
third-party diffs are preserved in the run directory. Hardware was observed
after execution, as labeled in the manifest. Shared-library hashes were also
added after execution; the libraries were not rebuilt between execution and
recording their hashes.
The current wrapper adds more checks and records this metadata before future
runs; the original wrapper snapshot remains available for this run.

## Export and association

The upstream `mono_tum` example exports keyframes only, and its
`SaveTrajectoryTUM` API rejects monocular input. The project-owned
[C++ runner](../../cpp/mono_tum_runner.cc) therefore uses the upstream
`SaveTrajectoryEuRoC` implementation, which supports monocular input and selects
the map with the most keyframes. At this pinned revision its output is
space-delimited `timestamp_ns tx ty tz qx qy qz qw`, despite the EuRoC name.
Camera-to-world poses are reconstructed using optimized keyframes and expressed
relative to the first keyframe. This behavior was checked in the pinned
[System.cc source](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc).

The tracked [shutdown patch](../../patches/ORB_SLAM3/wait-for-shutdown.patch)
waits for mapping/loop-closing workers before export; the upstream wait block
was commented out. Other tracked patches provide build compatibility. SLAM
estimation algorithms were not modified. Decimal timestamp conversion restores
the exact RGB timestamp string within a 1 microsecond tolerance and preserves
the upstream quaternion order. The primary TUM trajectory includes only poses
whose online state was `OK` or `OK_KLT`. Unfiltered exports, online poses,
keyframes and tracking logs are retained as diagnostics. Online poses precede
final optimization and are not the evaluated trajectory.

Ground-truth association reuses the dataset loader's fixed one-to-one,
closest-first matches across the entire RGB timeline, with an inclusive 20 ms
tolerance and no interpolation. The maximum time difference among evaluated
poses was 7.564 ms. Of 796 reference-associable RGB frames, three were used for
initialization, leaving 793 evaluated estimates. Two additional tracked estimates,
at `1305031108.876515` and `1305031108.911364`, lack a reference match. These are
reference gaps, not tracking failures. Plots retain the gaps.

## Accuracy and tracking

Both alignments fit rotation and translation using all 793 matched positions.
Sim(3) additionally fits one global scale. SE(3) leaves native numeric scale
unchanged and treats one native unit as one metre for this diagnostic comparison;
unscaled monocular output does not carry an independently established metre unit.
These conventions follow the [evo alignment documentation](https://github.com/MichaelGrupp/evo/wiki/Metrics).
No held-out alignment set is used.

RPE uses the closest future evaluated pose to a 1.0 s interval, within ±20 ms,
for each eligible anchor. A pair is excluded if it spans any missing original
RGB input index, including a missing reference match. There are 733 overlapping
pairs, with actual intervals of 0.995507–1.004983 s. They are correlated samples,
not independent experimental replicates.

| Metric | SE(3), native numeric scale | Sim(3), GT-fitted scale |
| --- | ---: | ---: |
| Position ATE RMSE (m) | 0.049830 | 0.009283 |
| Position ATE median (m) | 0.040272 | 0.006726 |
| Position ATE maximum (m) | 0.103615 | 0.032973 |
| 1 s translation RPE RMSE (m) | 0.074076 | 0.014720 |
| 1 s rotation RPE RMSE (degrees) | 0.956560 | 0.956560 |
| Applied global scale | 1.000000 | 1.357883 |

| Tracking observation | Result |
| --- | ---: |
| Submitted RGB frames | 798 |
| Initializing frames before first successful track | 3 |
| Successfully tracked frames / optimized primary poses | 795 / 795 |
| Primary trajectory coverage | 99.624% |
| Matched poses used for scoring | 793 |
| `RECENTLY_LOST` / `LOST` frames | 0 / 0 |
| Post-initialization failure events | 0 |
| Maps at export / largest-map keyframes | 1 / 53 |

The primary trajectory runs from `1305031102.275326` to
`1305031128.747363`. All 795 successful tracks are represented in the exported
primary map for this run. A future multi-map run can lose coverage when only
the largest map is evaluated, so tracking success and exported coverage remain
separate measurements.

![Matched ground-truth and Sim3-aligned ORB-SLAM3 trajectory in XY/XZ, SE3 and Sim3 position errors, and the complete tracking-state timeline.](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/figures/trajectory_and_tracking.png)

## Exploratory scale diagnostics

Five-second windows start at the first evaluated estimate. After global Sim(3)
alignment, an additional local Sim(3) position fit gives correction `c`.
The reported residual estimate scale is `(1/c - 1) × 100%`. These are
GT-conditioned descriptive fits, introduced during analysis after observing
path-length sensitivity to jitter. They are not a prespecified hypothesis test,
an online scale estimator, or proof of metric observability.

| Window (s) | Local fitted residual scale (%) | Path-length difference (%) | Maximum reference excursion from window start (m) |
| --- | ---: | ---: | ---: |
| 0–5 | +0.044 | +9.637 | 0.369 |
| 5–10 | −0.803 | +12.327 | 0.348 |
| 10–15 | −0.170 | +10.224 | 0.561 |
| 15–20 | −0.512 | +5.588 | 0.300 |
| 20–25 | +0.730 | +8.608 | 0.457 |
| 25–26.472 (partial) | +1.398 | +68.216 | 0.059 |

Path lengths sum only adjacent original input-frame displacements within each
window. Their ratios include pose jitter and reference noise. The final partial
window has just 0.0671 m of reference path length, making that ratio especially
sensitive to noise; its +68.2% path-length difference must not be labeled 68.2%
scale drift. Local fits are also less informative over short, low-motion or
geometrically weak trajectories. The complete windows show local fitted
residuals between −0.803% and +0.730% for this run, without establishing
cross-sequence stability.

![Local fitted scale residual and adjacent-frame path-length difference in five-second windows, with the short low-motion final window explicitly marked.](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/figures/scale_diagnostics.png)

## Runtime and limits

Mean `TrackMonocular` call time across all 798 frames was 10.825 ms
(median 10.353 ms; maximum 37.817 ms). Docker invocation through final export
took 36.721 s, including startup/vocabulary loading, paced replay and shutdown.
The replay sleeps according to input timestamp intervals without dropping
frames. Mapping and loop closing run asynchronously; per-call timing excludes
image decoding, startup and final worker completion. These measurements do not
establish controlled real-time throughput or NVIDIA/other-machine performance.

This is one run on one short indoor sequence. No repeated-run variability,
confidence intervals, failure-rate generalization or depth-assisted tracking
improvement has been established. Pose errors and the previous per-pixel
[DA3 depth errors](da3_depth_comparison.md) measure different quantities and
cannot be compared as an improvement percentage. The subsequent
[DA3 RGB-D experiment](da3_orbslam3_rgbd_baseline.md) now uses this preserved
RGB-only run for its common-timestamp comparison and reports the mixed result.

## Reproduction and artifacts

From the repository root, with the existing sources, data, Python environment
and cached Docker image:

```bash
docker run --pull=never --rm --network=none \
  --mount "type=bind,source=$PWD,target=/workspace" -e UV_OFFLINE=1 \
  monocular-depth-slam:dev bash scripts/build_monocular_runner.sh

# Preserve run_001; the runner refuses to overwrite an existing run directory.
.venv/bin/python scripts/run_monocular.py \
  --run-dir results/orbslam3_mono_fr1_xyz/run_002
.venv/bin/python scripts/evaluate_monocular.py \
  --run-dir results/orbslam3_mono_fr1_xyz/run_002
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_monocular.py \
  --run-dir results/orbslam3_mono_fr1_xyz/run_002
```

The build/run use `--pull=never` and disable container networking. Source or
package absence must be resolved separately after previewing any new download
sizes. The current `run_001` can be rescored or replotted without replaying it
by using that directory instead. Repeated execution may produce different
trajectories because worker scheduling is asynchronous.

The official evo CLI was also run against the matched reference and original
primary trajectory, with `--align --t_max_diff 0.02` for SE(3) and additionally
`--correct_scale` for Sim(3). Both saved ATE RMSE values match the evaluator
exactly. This is an end-to-end format/association check using the same underlying
evo implementation, not an independent accuracy algorithm.

- [Primary trajectory, TUM format, native monocular units](../../results/orbslam3_mono_fr1_xyz/run_001/trajectory.tum)
- [Run manifest](../../results/orbslam3_mono_fr1_xyz/run_001/manifest.json) and [run log](../../results/orbslam3_mono_fr1_xyz/run_001/run.log)
- [Evaluation summary and hashes](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/summary.json)
- [Per-pose scores/association](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/per_pose.csv) and [Sim3 RPE pairs](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/rpe_sim3.csv)
- [Tracking states/timings](../../results/orbslam3_mono_fr1_xyz/run_001/tracking.csv)
- [SE3 CLI result](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/evo_ape_se3.zip) and [Sim3 CLI result](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/evo_ape_sim3.zip)
- [Trajectory figure PDF](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/figures/trajectory_and_tracking.pdf), [scale figure PDF](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/figures/scale_diagnostics.pdf), and [figure provenance](../../results/orbslam3_mono_fr1_xyz/run_001/evaluation/figures/provenance.json)

Generated artifacts stay local under gitignored `results/`. The report and
reproduction code are tracked. Validation: 68 Python tests passed on macOS,
Ruff passed, and the C++ runner built and completed the real sequence inside
the Ubuntu ARM64 container. Tests cover timestamp restoration, quaternion
ordering, known-scale alignment, rotational RPE, missing-frame pair exclusion
and tracking-event accounting.

## Software acknowledgement

K-Dense's Scientific Visualization skill informed figure design, explicit
alignment/gap labeling and export inspection. This is a workflow acknowledgement,
not evidence for the measured SLAM accuracy: Kassis, T., Agarwal, V., He, Y.,
Patel, D., & Brueckner, A. M. (2026). *Scientific Agent Skills: A Library of
Procedural Knowledge for Research Agents*. arXiv:2609.00065.
[https://doi.org/10.48550/arXiv.2609.00065](https://doi.org/10.48550/arXiv.2609.00065).
