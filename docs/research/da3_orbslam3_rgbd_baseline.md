# DA3 Metric Large through ORB-SLAM3 RGB-D

Date: 2026-11-04. First depth-assisted run:
`results/orbslam3_da3_rgbd_fr1_xyz/run_001`.

The pipeline completed all **798 RGB frames**, initialized on the first frame,
and exported all 798 poses in one map, with no later tracking-loss events.
It uses independently inferred DA3 depth through ORB-SLAM3's existing RGB-D
interface. No sensor depth, ground-truth poses or GT-fitted depth correction
enters inference or SLAM. No additional assets were downloaded.

The result is mixed: nominal scale is closer to the reference than the
monocular run's arbitrary numeric scale, but **trajectory agreement after
global scale fitting and relative pose errors are worse** in this single run.
This establishes a working integration and a comparison baseline, not a
general accuracy or tracking-robustness improvement.

## Comparison on identical timestamps

The [protocol](da3_rgbd_protocol.md) was recorded before collecting the
depth-assisted trajectory. Both runs submitted the same 798 RGB images with
identical calibration, timestamps and ORB settings. The comparison refits and
scores both trajectories on their identical intersection of **793 RGB/reference
pose pairs**, with the same **733 one-second RPE pairs**. Thus first-frame RGB-D
initialization does not change the accuracy comparison's sample set.

| Measurement | ORB-SLAM3 monocular | DA3 + ORB-SLAM3 RGB-D |
| --- | ---: | ---: |
| Exported poses / all RGB inputs | 795 / 798 | 798 / 798 |
| Frames before first `OK` state | 3 | 0 |
| Later tracking-loss events | 0 | 0 |
| Maps / keyframes at export | 1 / 53 | 1 / 76 |
| Common-pose SE(3) ATE RMSE (m) | 0.049830 | 0.022218 |
| Common-pose Sim(3) ATE RMSE (m) | 0.009283 | 0.011687 |
| Common-pair SE(3) translation RPE RMSE, 1 s (m) | 0.074076 | 0.035018 |
| Common-pair Sim(3) translation RPE RMSE, 1 s (m) | 0.014720 | 0.020116 |
| Common-pair rotation RPE RMSE, 1 s (degrees) | 0.956560 | 1.135417 |
| Sim(3) GT-fitted scale correction | 1.357883 | 1.113338 |

SE(3) fits rotation and translation only. It treats native monocular numeric
units as metres for that diagnostic, while RGB-D uses DA3's nominal metre
conversion. The smaller RGB-D SE(3) error therefore does not by itself show an
improvement in scale-independent geometry over monocular SLAM. Sim(3) additionally
fits one GT scale independently for each method using all common matched poses;
its comparison gives higher RGB-D ATE and translation RPE. Rotation RPE is also
higher. Both methods have zero post-initialization losses on this sequence;
the extra three RGB-D poses demonstrate earlier initialization, not stronger
failure recovery.

The learned-depth run still needs a GT scale multiplier of 1.113338 to best
match the common reference positions. Expressed as `1 / fitted_scale - 1`, its
global estimated scale is about **10.18% below** that reference. This is
consistent with the earlier [DA3 depth underestimation](da3_depth_comparison.md),
but this experiment does not isolate depth bias as the only cause of trajectory
error. The fitted correction is evaluation output; it was not applied to the
depth fed into ORB-SLAM3.

![The two Sim3-aligned XY trajectories and their SE3 position errors, Sim3 position errors, and one-second Sim3 translation RPE on identical timestamps. DA3 reduces unscaled error but increases scale-corrected errors in this run.](../../results/trajectory_comparison_fr1_xyz/comparison.png)

The [comparison JSON](../../results/trajectory_comparison_fr1_xyz/comparison.json)
links both run manifests and common-set summaries. All scores use the loader's
fixed one-to-one closest-first GT associations, inclusive 20 ms tolerance, no
interpolation. The maximum evaluated time difference is 7.564 ms. RPE selects
the closest future pose at 1.0 s ±20 ms and excludes any pair crossing missing
original input indices, including reference gaps. Figures break lines at those
gaps. Frames and overlapping RPE pairs are correlated, not independent replicates.

## Depth preparation and interface assumptions

The pinned DA3 Metric Large checkpoint is
`depth-anything/DA3METRIC-LARGE` at
`4010e39f3634a45bc60553321fb49fb760bd594e`, with source revision
`3d835ec1a5802d64a8b8b15f817a1ab54809bfe4`.
The original cache contains 792 sensor-matched RGB predictions. The six missing
RGB timestamps were inferred locally into
`predictions/da3_metric_large/fr1_xyz_extra_rgb/`, using the same checkpoint,
patched source, macOS Metal environment, package versions, seed and single-image
processing. Original cache files and its manifest were preserved. Sensor
association selected the old cache's frames; sensor measurements never entered
the network or the depth adapter.

For all 798 frames, the adapter:

1. Verifies RGB/cache hashes and matching checkpoint, processing, source and
   environment provenance across both prediction caches.
2. Converts canonical z-depth using `mean(fx_processed, fy_processed) / 300`.
   The processed 378×504 intrinsics must describe a pure resize of the original
   RGB calibration.
3. Upsamples predicted z-depth bilinearly at pixel centres to the unchanged
   480×640 RGB grid. It does not resample sensor depth or undistort RGB/depth
   separately. ORB samples depth at the original distorted keypoint location
   before unprojecting its undistorted keypoint.
4. Conservatively invalidates nonfinite/nonpositive depth contributions and
   writes invalid output as zero. No confidence filtering, reference mask,
   depth-range clipping, oracle/prefix scale or temporal smoothing is used.
5. Writes single-channel float32 TIFF without integer quantization. Files hold
   nominal metres directly, and `RGBD.DepthMapFactor=1.0` avoids TUM's sensor
   PNG factor of 5000. All 798 maps had zero invalid pixels; their observed
   predicted range was 0.5036–3.8828 nominal metres.

The generated TIFF files total 980,689,332 bytes of local output. The runner
checks OpenCV decoded type, dimensions, finite/nonnegative values, and compares
each frame's minimum, maximum and centre sample against preparation records.
All checks passed for all 798 inputs. The
[prepared manifest](../../predictions/da3_metric_large/fr1_xyz_orb_rgbd/manifest.json)
records source-cache hashes, conversion code, image/depth hashes and statistics.

The C++ runner calls `TrackRGBD` with synchronized RGB and learned depth, viewer
disabled. RGB intrinsics/distortion, original dimensions, BGR-to-RGB conversion,
1000-feature ORB configuration and seed/thread controls match the monocular
experiment. ORB's RGB-D and monocular initialization paths inherently differ;
RGB-D initializes from one depth-bearing frame, while monocular uses its
larger bootstrap extractor and multiple views.

The RGB-D adapter retains the pinned upstream TUM1 settings `Stereo.b=0.07732`
and `Stereo.ThDepth=40`, yielding nominal close/far threshold 3.0928 m and
`bf = fx × 0.07732`. ORB forms a virtual right-image coordinate `u_right =
u_undistorted - bf / depth`. These settings influence depth weighting and point
selection; the parameter is not a measured physical stereo baseline for DA3.
The implementation has no explicit learned-depth uncertainty/correlation model.
The initial settings were not tuned after observing the result. Their behavior
was checked in the pinned local
[Settings.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc),
[Tracking.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc), and
[Frame.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Frame.cc).

## Full-run results, timing and provenance

The primary trajectory spans `1305031102.175304`–`1305031128.747363`.
Of its 798 estimates, 796 have reference matches; two have the same missing
GT matches as the monocular run. On all 796 matched poses, SE(3) ATE RMSE is
0.022197 m and Sim(3) ATE RMSE is 0.011689 m, with fitted scale 1.113281.
Its 736 RPE pairs give SE(3) translation RMSE 0.035013 m, Sim(3) translation
RMSE 0.020085 m and rotation RMSE 1.133585 degrees. The common-set table above
uses 793/733 observations instead of comparing these unequal sample sets.

Five-second local Sim(3) and adjacent-frame path-length diagnostics are retained
in each evaluation summary. They condition on GT and are exploratory. Short,
low-motion windows amplify jitter in path-length ratios; those ratios must not
be called metric scale drift. The common-set summaries use the same window
origin for both methods. No uncertainty bounds or observability conclusion is
derived from these local fits.

SLAM ran on the Apple M4 MacBook Air (16 GiB, macOS 26.6.2) in Ubuntu 24.04
ARM64 Docker. Runtime reports 10 virtual CPUs, 8,319,504,384 bytes memory and
engine 29.9.2. Cached image ID:
`sha256:c1b2a4f1ea12e14077034e59ab69ec582a532b48d6025d3f176a66b89638c2a0`.
ORB-SLAM3 is pinned to `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` and
Pangolin to `dd801d244db3a8e27b7fe8020cd751404aa818fd`.
The existing build-compatibility and worker-shutdown patches are unchanged;
no SLAM estimation algorithm was modified for this run. The largest-map
optimized export, successful-online-state filtering, exact timestamp restoration
and xyzw quaternion conventions match the
[monocular baseline](orbslam3_monocular_baseline.md).

Mean `TrackRGBD` call time was 10.412 ms (median 10.393 ms; maximum 13.481 ms).
Mean RGB/depth decoding and pre-call validation time was 7.833 ms. Docker
invocation through paced replay/shutdown/export took 38.251 s. Across the
original and supplemental inference sessions, the recorded per-frame DA3
inference durations sum to 384.855 s (mean 0.4823 s/frame), excluding model
loading and cache export. These are asynchronous, sequentially collected,
offline measurements; cached SLAM replay does not establish end-to-end
real-time operation or a controlled speed comparison. CUDA remains untested.

The [run manifest](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/manifest.json)
records inputs, exact source snapshots/patches, binary and library hashes,
calibration, cache provenance, hardware and invocation. Evaluation uses Python
3.12.14, evo 1.38.1, NumPy 2.5.4 and SciPy 1.18.1. Official evo CLI SE(3) and
Sim(3) ATE results match the full-run evaluator exactly; this crosschecks
format/association using the same evo implementation, not an independent
algorithm. Original monocular artifacts remain preserved, with source snapshots
for its earlier evaluator and figure script retained as those scripts evolve.

## Reproduce with existing local assets

Run from the Mac repository root. The supplemental prediction step uses the
existing Metal checkpoint and `local_files_only`; the Docker build/run disable
networking and image pulling.

```bash
HF_HUB_OFFLINE=1 .venv-da3-macos/bin/python scripts/cache_da3_missing_rgb.py
.venv/bin/python scripts/prepare_da3_rgbd.py
docker run --pull=never --rm --network=none \
  --mount "type=bind,source=$PWD,target=/workspace" -e UV_OFFLINE=1 \
  monocular-depth-slam:dev bash scripts/build_monocular_runner.sh

# Preserve the completed run_001; select another unused directory for a replay.
.venv/bin/python scripts/run_da3_rgbd.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_002
.venv/bin/python scripts/evaluate_monocular.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_002
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_monocular.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_002
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/compare_trajectories.py \
  --depth-run results/orbslam3_da3_rgbd_fr1_xyz/run_002 \
  --output results/trajectory_comparison_fr1_xyz_run_002
```

`evaluate_monocular.py` now handles both sensor modes while retaining its original
entrypoint. Comparison writes separate common-set evaluations; original
full-run results are preserved. Rerunning SLAM can change results through worker
scheduling. On a future NVIDIA host, rebuild the C++ environment for that CPU
architecture, generate both prediction caches with the same CUDA environment,
and use separate cache directories/protocols for the different provenance.
Do not mix Metal and CUDA caches under one preparation contract. Preview any
missing software/data download sizes first.

Validation: **71 tests pass on macOS**, including fixed metre conversion,
float32 TIFF round trips, invalid-depth handling and rejection of cropped or
misaligned processed intrinsics. Ruff and the C++ build pass. Actual OpenCV
decoding is checked during the completed real sequence, and the common-set
comparison verifies identical RGB/reference associations and RPE pairs.

Useful local artifacts:

- [Primary DA3-assisted TUM trajectory](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/trajectory.tum), [tracking log](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/tracking.csv), [run log](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/run.log)
- [Full-run summary](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/evaluation/summary.json), [SE3 CLI check](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/evaluation/evo_ape_se3.zip), [Sim3 CLI check](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/evaluation/evo_ape_sim3.zip)
- [Common-set comparison](../../results/trajectory_comparison_fr1_xyz/comparison.json), [monocular common scores](../../results/trajectory_comparison_fr1_xyz/monocular/summary.json), [DA3 common scores](../../results/trajectory_comparison_fr1_xyz/da3_rgbd/summary.json)
- [Comparison PDF](../../results/trajectory_comparison_fr1_xyz/comparison.pdf), [figure provenance](../../results/trajectory_comparison_fr1_xyz/figure_provenance.json), [DA3 trajectory figure](../../results/orbslam3_da3_rgbd_fr1_xyz/run_001/evaluation/figures/trajectory_and_tracking.png)

Outputs remain gitignored. One short indoor sequence and one replay per method
cannot establish statistical superiority, repeated-run stability, general metric
calibration, failure recovery or other-hardware performance. A sensor-depth
RGB-D control and repeated runs would help separate interface behavior from
learned-depth errors before tuning. Any GT-anchored depth calibration must be a
separate experiment.

The subsequent [sensor-depth control](sensor_depth_control.md) now completes
three processes per method on a shared 792-frame input timeline, preserving
these original 798-frame results separately. It confirms the ordering of
scale-corrected errors within the observed repeat ranges on this sequence.

## Software acknowledgement

K-Dense's Experimental Design and Scientific Visualization skills informed the
fixed controls, replication limits, explicit alignment labels and figure
inspection. They are workflow assistance, not evidence for the measured result:
Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026).
*Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents*.
arXiv:2609.00065. [https://doi.org/10.48550/arXiv.2609.00065](https://doi.org/10.48550/arXiv.2609.00065).
