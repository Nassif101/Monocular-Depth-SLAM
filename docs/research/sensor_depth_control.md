# Sensor-depth control and repeated monocular / DA3 / RGB-D runs

Date: 2026-11-04. Study: `results/sensor_depth_control_fr1_xyz/`.
Protocol: [recorded before replay](sensor_depth_control_protocol.md).

All **nine scheduled runs completed**: three processes each for monocular,
DA3 RGB-D and sensor-depth RGB-D. Every process received the same **792 RGB
frames**. There were no post-initialization tracking-loss events. Both RGB-D
methods exported 792/792 poses in every repeat; monocular exported 788/792,
using four frames for initialization in every repeat. Each run retained one map.
No additional assets were downloaded, and the DA3 predictions were reused.

The sensor control brings SE(3) error and fitted scale much closer to the
reference than DA3. However, **monocular has the lowest Sim(3) ATE and
scale-corrected translation RPE in all three repeats**. Sensor RGB-D lies
between monocular and DA3 for those errors. This preserves the earlier finding
that the direct learned-depth adapter has not improved trajectory agreement
after scale correction on this sequence.

## Common-set accuracy and observed variation

The primary comparison independently refits every alignment using the identical
intersection of **786 matched RGB/reference poses across all nine runs**.
It also uses **725 identical RPE pairs** at 1.0 s ±20 ms. There are 790
reference-associable frames in the 792-frame input; four are excluded from the
common score by monocular initialization. Two additional inputs have no GT
pose match. Full-run scores and coverage are retained separately per process.

Entries below are **median [minimum, maximum] across three process replays**.
Ranges describe these observations; they are not confidence intervals.

| Measurement | Monocular | DA3 RGB-D | Sensor RGB-D |
| --- | ---: | ---: | ---: |
| SE(3) ATE RMSE (cm) | 3.501 [3.354, 3.534] | 2.245 [2.165, 2.285] | 1.066 [1.027, 1.073] |
| Sim(3) ATE RMSE (mm) | 9.371 [9.170, 9.407] | 11.721 [11.672, 11.921] | 10.471 [10.239, 10.525] |
| SE(3) 1 s translation RPE RMSE (cm) | 5.216 [4.985, 5.261] | 3.580 [3.412, 3.592] | 1.766 [1.741, 1.782] |
| Sim(3) 1 s translation RPE RMSE (cm) | 1.523 [1.471, 1.539] | 2.043 [2.002, 2.158] | 1.747 [1.739, 1.761] |
| 1 s rotation RPE RMSE (degrees) | 0.966 [0.960, 0.966] | 1.165 [1.111, 1.182] | 1.155 [1.103, 1.164] |
| GT-fitted global scale correction | 1.2211 [1.2095, 1.2239] | 1.1139 [1.1084, 1.1180] | 1.0108 [1.0046, 1.0114] |

SE(3) fits rotation/translation without scale correction. Its monocular column
uses arbitrary native numerical units as metres for a diagnostic comparison;
it does not measure independently calibrated monocular metric scale. DA3 uses
its fixed nominal metre conversion; sensor RGB-D uses measured z-depth metres.
Sim(3) additionally fits one GT scale per run using the entire common matched
set. No held-out alignment set is used.

The DA3 correction remains near 1.11, consistent with its previously observed
depth underestimation. Sensor corrections are near 1.005–1.011. This supports
the unit conversion and the usefulness of a sensor-depth scale anchor, but
does not prove that DA3 bias alone causes the difference in trajectory error.
The RGB-D interface, depth weighting, different valid-pixel patterns and temporal
depth errors may contribute. Sensor and DA3 rotation-error ranges overlap;
this small study does not isolate a reliable rotation advantage between them.

![All three repeats for each method: SE3 ATE, Sim3 ATE, one-second Sim3 translation RPE and GT-fitted scale. Circle, square and triangle identify repeat number; lines show medians and observed ranges, not confidence intervals.](../../results/sensor_depth_control_fr1_xyz/repeat_comparison.png)

All individual scores, mean, sample SD, median and observed extrema are retained
in the [summary](../../results/sensor_depth_control_fr1_xyz/summary.json) and
[per-run CSV](../../results/sensor_depth_control_fr1_xyz/per_run.csv).
No outlier removal, best-run selection, significance test or interval estimate
was performed. These are three fixed-seed processes on **one independent
sequence**, not three independent scenes. Frames and overlapping RPE pairs are
correlated observations, not additional experimental replications.

## Inputs, sensor depth and controls

The subset contains the original loader's 792 RGB/sensor-depth matches under
an inclusive 20 ms one-to-one, closest-first association. The native sequence
contains 798 RGB frames. Associations to GT poses are reused from that complete
loader timeline and are never recomputed after subsetting. The maximum
RGB/sensor-depth time difference is 17.230 ms; reference pose association uses
the same 20 ms protocol as earlier reports, without interpolation.

All three methods receive the same selected RGB images, paths, hashes and exact
timestamp strings. RPE gap checks use consecutive indices in the **submitted
792-frame timeline**. The six intentionally excluded source RGB images are not
tracking failures. Missing estimates or reference poses within the submitted
timeline still break RPE pairs. Per-pose CSVs additionally retain the original
798-frame dataset index so the selection can be audited.

Sensor-control depth is converted from the existing registered 16-bit PNGs by
dividing by 5000. Missing sensor pixels remain zero; there is no interpolation,
hole filling, range clipping, extra Freiburg correction factor or scale fitting.
There are 59,122,608 invalid pixels among 243,302,400 input pixels, about 24.30%.
The observed maximum nonzero sensor depth is 7.1748 m. Sensor depth is an
explicit measurement control with noise, holes and timing offsets, not a
noise-free oracle or a deployable RGB-only method.

Prepared sensor depth is single-channel float32 TIFF in metres, matching the
DA3 input format and `RGBD.DepthMapFactor=1`. Its 792 files total 973,315,728
bytes of generated local output. The
[preparation manifest](../../predictions/sensor_depth/fr1_xyz_orb_rgbd/manifest.json)
records original sensor timestamps/hashes, conversion code, output hashes,
invalid counts and decoded value checks. All RGB-D runs passed float32 type,
640×480 dimensions, finite/nonnegative values, minimum/maximum/centre sample
checks for every submitted frame. No GT world poses enter SLAM. Sensor depth
enters only the explicitly labeled sensor control; DA3 receives no sensor values
or GT-fitted depth correction.

The two RGB-D methods use the **identical compiled executable**, shared
libraries, RGB camera/distortion parameters, original image grid, RGB color
order, ORB settings and upstream TUM1 depth parameters. `Stereo.b=0.07732`
and `Stereo.ThDepth=40` retain the earlier virtual-baseline weighting and
3.0928 m close/far threshold. This is still the standard RGB-D-style adapter,
without a learned-depth uncertainty model. No settings were tuned after
observing these runs. Monocular retains its own inherent initialization path.

## Schedule, provenance and runtime

The nine-job schedule was archived before replay. A seeded Latin square puts
every method once in each block and once in each within-block position:

| Block | First | Second | Third |
| --- | --- | --- | --- |
| 1 | DA3 repeat 1 | Monocular repeat 1 | Sensor repeat 1 |
| 2 | Sensor repeat 2 | DA3 repeat 2 | Monocular repeat 2 |
| 3 | Monocular repeat 3 | Sensor repeat 3 | DA3 repeat 3 |

Schedule seed is 20261104. SLAM/OpenCV/C RNG controls remain zero for every
fresh process, so repeats expose asynchronous processing variation rather than
a deliberate seed sweep. This balanced order reduces systematic position effects
but does not control every host-load or thermal effect. Every attempt is retained;
all nine succeeded. Keyframe counts were 53–55 for monocular, 75–79 for DA3
and 73–76 for sensor depth, despite zero tracking-loss events in every run.

The host remains the Apple M4 MacBook Air with 16 GiB memory and macOS 26.6.2.
Execution used the same cached Ubuntu 24.04 ARM64 Docker image, 10 virtual CPUs
and approximately 7.75 GiB runtime memory. ORB-SLAM3 is pinned to
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`; Pangolin to
`dd801d244db3a8e27b7fe8020cd751404aa818fd`. Build/shutdown patches and shared
library hashes are unchanged. The control RGB-D executable differs from the
earlier DA3 runner only in its neutral names/log banner, not estimation behavior.
The original monocular and DA3 798-frame runs remain preserved.

Evaluation used Python 3.12.14, evo 1.38.1, NumPy 2.5.4 and SciPy 1.18.1.
The official evo CLI reproduces both common-set SE(3) and Sim(3) ATE RMSE
exactly for sensor repeat 1; see the
[CLI check](../../results/sensor_depth_control_fr1_xyz/cli_crosscheck.json).
This checks format/association using the same evo implementation. Every run
has a manifest, input hashes, source snapshots, tracking log, optimized poses,
full evaluation and a separately fitted common evaluation. Analysis, plot and
configuration hashes are included in the study summary and figure provenance.

Replay invocation through paced processing/export took 34.67–35.31 s for
monocular, 36.33–37.64 s for DA3 and 36.46–37.22 s for sensor depth.
Per-run mean tracking-call durations are in the CSV. All depth was prepared
offline; these timings exclude DA3 inference and are descriptive, not a
controlled real-time throughput or hardware comparison. CUDA remains untested.

## Reproduction and validation

From the repository root, with the existing assets and cached image:

```bash
.venv/bin/python scripts/prepare_sensor_rgbd.py
docker run --pull=never --rm --network=none \
  --mount "type=bind,source=$PWD,target=/workspace" -e UV_OFFLINE=1 \
  monocular-depth-slam:dev bash scripts/build_monocular_runner.sh
.venv/bin/python scripts/run_sensor_control_study.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/evaluate_sensor_control_study.py
```

The run script preserves completed/failed attempts and only executes pending
jobs under an unchanged study contract. Existing run directories cannot be
overwritten. Archive any additional study under new output paths/configuration.
The evaluation command can regenerate scores and figures without rerunning SLAM.
Sensor-control TIFF and run outputs are gitignored. No asset downloads are
part of these commands; preview sizes separately before obtaining anything
missing on another machine.

Validation: **74 macOS tests pass**, including subset selection with preserved
GT associations, exact input-timeline checks, and RPE treatment of intentional
source omissions versus missing submitted poses. Ruff and the C++ build pass.
The real runs check identical inputs and RGB-D decode values; analysis verifies
identical associations and RPE pairs across all nine common evaluations.

- [Study schedule, attempts and run links](../../results/sensor_depth_control_fr1_xyz/study.json)
- [Numerical summary](../../results/sensor_depth_control_fr1_xyz/summary.json), [per-run CSV](../../results/sensor_depth_control_fr1_xyz/per_run.csv), [common timestamp set](../../results/sensor_depth_control_fr1_xyz/common_stamps.json)
- [Figure PDF](../../results/sensor_depth_control_fr1_xyz/repeat_comparison.pdf), [figure provenance](../../results/sensor_depth_control_fr1_xyz/figure_provenance.json)
- [Monocular repeat 1 trajectory](../../results/sensor_depth_control_fr1_xyz/monocular/repeat_001/trajectory.tum), [DA3 repeat 1 trajectory](../../results/sensor_depth_control_fr1_xyz/da3_rgbd/repeat_001/trajectory.tum), [sensor repeat 1 trajectory](../../results/sensor_depth_control_fr1_xyz/sensor_rgbd/repeat_001/trajectory.tum)

## Interpretation and next experiment

The differences exceed the small observed run-to-run ATE ranges here, but one
scene and three processes do not establish statistical or cross-scene
superiority. Earlier 798-frame numerical scores should not be compared as if
they used this 792-frame input: removing six images changes initialization and
optimization, including arbitrary monocular scale.

Sensor depth reduces both DA3's scale discrepancy and its scale-corrected errors,
while monocular retains the lowest Sim(3) errors. This makes depth quality and
the RGB-D depth constraint assumptions useful investigation targets rather than
establishing a single causal explanation. The next diagnostic is the previously
measured fixed prefix calibration, scored on later frames and explicitly labeled
GT-anchored. Calibration may reduce scale bias; whether it improves trajectory
geometry still needs measurement. Robust depth weighting and additional sequences
follow as separate experiments, with new download sizes reported first.

The subsequent [prefix-calibration diagnostic](da3_prefix_calibration.md) has
now completed three calibrated replays and a separately labeled pose-only
rescale check. It refits/scores only later poses, leaving this full-cohort
control study unchanged.

## Software acknowledgement

K-Dense's Experimental Design, Statistical Analysis and Scientific Visualization
skills informed the fixed controls, sampling-unit limits, descriptive reporting
and figures. They are workflow assistance, not evidence for the measured result:
Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026).
*Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents*.
arXiv:2609.00065. [https://doi.org/10.48550/arXiv.2609.00065](https://doi.org/10.48550/arXiv.2609.00065).
