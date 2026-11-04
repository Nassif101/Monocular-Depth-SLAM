# DA3 GT-prefix calibration: later-frame trajectory diagnostic

Date: 2026-11-04. Study: `results/da3_prefix_calibration_fr1_xyz/`.
The [protocol](prefix_calibration_protocol.md) was recorded before collecting
the calibrated trajectories. No assets were downloaded or DA3 inference rerun.

Using the frozen **1.1093010159475212×** depth correction from the first 158
sensor-matched frames, all three calibrated RGB-D replays exported **792/792
poses**, initialized on the first frame, retained one map and had zero later
tracking losses. Scoring excludes the calibration prefix.

On **632 identical later GT-matched poses**, median SE(3) ATE falls from
**2.302 cm to 1.066 cm**. Most of this difference is explained by correcting
scale: applying the same scalar to existing DA3 poses, without replay, already
gives **1.139 cm**. Calibrated depth replay also gives a modest observed Sim(3)
ATE reduction, **11.387 mm to 10.660 mm**, while monocular remains lower at
**9.050 mm**. These are observations on one already inspected sequence,
not evidence of unanchored metric recovery or general superiority.

## Matched later-frame comparison

Every executed method used the same 792 RGB frames and fixed reference
associations. The first 158 frames are calibration/warm-up only. Of the next
634 RGB inputs, two lack a GT pose association, leaving 632 common poses across
all twelve successful executed runs. The comparison uses **572 identical RPE
pairs** at 1.0 s ±20 ms. Both endpoints must follow the prefix, and missing
submitted poses/GT still break pairs. No interpolation is used.

All alignments are fitted independently on that same later-pose set. SE(3)
fits rotation and translation; Sim(3) additionally fits one GT scale. Prefix
poses enter neither alignment nor the reported ATE/RPE. Later GT world poses
are used for benchmark alignment/scoring, while later sensor depth is excluded
from fitting the depth correction.

Values below are medians across three processes per executed method. The
pose-rescale row instead derives three evaluations from the three existing raw
DA3 trajectories; it is not three additional SLAM processes.

| Case | SE(3) ATE RMSE (cm) | Sim(3) ATE RMSE (mm) | Sim(3) 1 s translation RPE RMSE (cm) | Additional GT-fitted scale |
| --- | ---: | ---: | ---: | ---: |
| Monocular, historical | 3.600 | 9.050 | 1.487 | 1.2220 |
| DA3 raw depth, historical | 2.302 | 11.387 | 2.064 | 1.1162 |
| DA3 poses × prefix scalar, derived | 1.139 | 11.387 | 2.064 | 1.0062 |
| DA3 depth × prefix scalar, new replay | 1.066 | 10.660 | 1.960 | 1.0007 |
| Sensor RGB-D, historical | 1.049 | 10.140 | 1.690 | 1.0138 |

Monocular native translation units remain arbitrary, so its unscaled SE(3)
column is a numerical diagnostic rather than independently calibrated metric
accuracy. The calibrated DA3 method receives an explicit GT-derived anchor;
its near-one fitted factor does not show that biased monocular geometry alone
identified metric scale.

Observed process ranges for the main raw-versus-calibrated comparison:

| Measurement | Raw DA3 minimum–maximum | Prefix-calibrated depth replay minimum–maximum |
| --- | ---: | ---: |
| SE(3) ATE RMSE (cm) | 2.211–2.344 | 1.066–1.111 |
| Sim(3) ATE RMSE (mm) | 11.220–11.509 | 10.660–11.098 |
| Sim(3) 1 s translation RPE RMSE (cm) | 2.044–2.214 | 1.878–2.100 |
| 1 s rotation RPE RMSE (degrees) | 1.126–1.207 | 1.107–1.184 |
| Additional GT-fitted scale | 1.10975–1.12031 | 1.00044–1.00256 |

The median SE(3) difference is about 53.7%, but most of that gain also occurs
under pose-only rescaling. Uniform pose scaling leaves Sim(3) ATE, Sim(3)
translation RPE and rotation RPE unchanged, verified numerically for each raw
trajectory within `1e-10`. This separates the mechanical effect of correcting
units from the observed changes after depth replay.

The calibrated replay's Sim(3) translation/rotation RPE ranges overlap the raw
ones; no statistical or independent-scene conclusion follows from these three
processes. Monocular still has lower scale-corrected ATE and translation RPE
than all calibrated repeats on these later timestamps. Sensor depth also has
lower scale-corrected ATE/translation RPE in this comparison. The result supports
investigating fusion assumptions beyond a shared depth-scale bias.

![All repeat scores for monocular, raw DA3, pose-only rescaling, calibrated-depth replay and sensor RGB-D on the identical later timestamps. Panels show SE3 ATE, Sim3 ATE, Sim3 one-second translation RPE and additional fitted scale. Raw and pose-only Sim3 scores coincide.](../../results/da3_prefix_calibration_fr1_xyz/heldout_comparison.png)

Every individual value, median, minimum/maximum, mean and sample SD is retained
in the [summary](../../results/da3_prefix_calibration_fr1_xyz/summary.json) and
[per-evaluation CSV](../../results/da3_prefix_calibration_fr1_xyz/per_run.csv).
No observations were removed or best run selected. Lines in the figure denote
the observed range and median, not confidence intervals. There is one independent
sequence, three new processes, nine historical control processes and three
derived pose-only evaluations. Correlated frames/RPE pairs are not additional
replications. Controls were not recollected in a new four-method balanced order.

## Calibration provenance and temporal scope

The 792-frame cohort is the original loader's sensor-matched RGB selection.
The exact calibration prefix is the first 158 selected frames, from
`1305031102.175304` through **`1305031107.611271`**. The evaluation input begins
**`1305031107.643323`** and ends `1305031128.747363`.

For each prefix image, compute the median sensor-GT/prediction depth ratio over
finite positive predictions and valid sensor depths in the previously specified
0.1–10 m reference interval. Then take the median of those 158 frame ratios.
Sensor PNGs are converted by /5000 with holes invalidated; predictions reuse
fixed focal-based canonical conversion and bilinear alignment. No per-frame
correction is applied to the replay.

The preparation script opens sensor-depth pixels only in its prefix-fitting
loop. It reproduces **exactly** the scalar stored in the earlier
[depth comparison](da3_depth_comparison.md), and refuses to proceed if it differs
from the frozen value. All 158 input sensor timestamps/hashes, prediction hashes,
pixel counts and ratios are recorded in
[calibration.json](../../predictions/da3_metric_large/fr1_xyz_prefix_calibrated_orb_rgbd/calibration.json).
Later sensor pixels and GT world poses do not fit this scalar. Prefix length,
reference mask/range, statistic and factor were not tuned using the new
trajectory outcomes.

The sequence's later depth and earlier trajectory results were already inspected
before choosing this diagnostic. Consequently, the later split is held out from
the correction fit, but this is not a blinded experiment or independent-sequence
validation. Its existing later-frame depth scores improve from 10.247% to 4.873%
AbsRel under that scalar; those prior depth scores are separate quantities from
the new pose errors and were not recomputed as new evidence here.

The factor becomes available after observing the full prefix. We apply it
retrospectively to the entire 792-frame replay, including initialization, and
score later frames only. This is an **offline calibrated-prior diagnostic**,
not a causal online estimator that possessed the fitted value before the
calibration observations arrived. It assumes a valid prefix sensor reference.

## Execution and unit checks

Prepared DA3 float32 TIFF maps are multiplied by the scalar in float32, preserving
zero invalid pixels. There is no clipping, spatial correction, confidence mask,
new depth inference or temporal smoothing. The calibrated 792-map cache totals
973,315,728 bytes of generated local output and is separate from raw predictions.
Its [manifest](../../predictions/da3_metric_large/fr1_xyz_prefix_calibrated_orb_rgbd/manifest.json)
links raw preparation, calibration evidence, protocol, image/depth hashes and
code. Every new run explicitly records `groundtruth_depth_used_for_calibration`
and `groundtruth_calibration` as true, preserves a copy of calibration evidence,
and records that world poses/sensor depth images are not tracker inputs.

The calibrated runs use the same `control_rgbd_tum` executable and shared-library
hashes as the raw DA3 and sensor controls. RGB intrinsics/distortion, 640×480
grid, BGR-to-RGB conversion, 1000-feature ORB configuration, seed zero, thread
controls, paced replay and optimized largest-map export are unchanged.
`RGBD.DepthMapFactor=1`, `Stereo.b=0.07732` and `Stereo.ThDepth=40` remain fixed.
Each frame passes OpenCV float32 type/dimension, finite/nonnegative value and
minimum/maximum/centre checks against prepared records.

Scaling depth while holding the virtual baseline and 3.0928 m close/far
threshold fixed can alter disparity residuals, relative depth weighting and
point selection. The calibrated replay therefore need not equal a uniformly
rescaled raw trajectory. This experiment measures the outcome of that replay
without isolating which internal mechanism contributes to the small shape change.

All three new runs remained `OK` for all 792 inputs, with one map and
77, 75 and 77 keyframes. Invocation through paced replay/export took
39.452, 37.307 and 38.700 s, respectively. DA3 inference is excluded because
depth is cached; these are descriptive offline times, not end-to-end real-time
performance. Historical controls were collected earlier, so timing comparisons
would additionally mix run order and host conditions.

The host is the Apple M4 MacBook Air (16 GiB, macOS 26.6.2), executing the
cached Ubuntu 24.04 ARM64 container. ORB-SLAM3 remains pinned to
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, Pangolin to
`dd801d244db3a8e27b7fe8020cd751404aa818fd`. No C++ estimation or library changes
were needed. Evaluation uses Python 3.12.14, evo 1.38.1, NumPy 2.5.4 and
SciPy 1.18.1. CUDA remains untested.

The official evo CLI reproduces both later-set SE(3) and Sim(3) ATE exactly
for calibrated repeat 1; see the
[CLI crosscheck](../../results/da3_prefix_calibration_fr1_xyz/cli_crosscheck.json).
This is a format/association check using the same evo implementation. Evaluation
also verifies identical RGB/reference associations and RPE pairs across all
fifteen scored cases and preserves all prior control artifacts.

## Reproduction and artifacts

For a fresh unused study output with the existing assets and executable:

```bash
.venv/bin/python scripts/prepare_prefix_calibration.py
.venv/bin/python scripts/run_prefix_calibration_study.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/evaluate_prefix_calibration.py
```

Completed directories and frozen provenance are preserved; additional replays
need separate study/cache paths and configuration. To regenerate the present
scores/figure without replaying SLAM or rewriting prepared caches, run only the
evaluation command. The generic evaluator's explicit `--translation-scale`
option writes pose-only diagnostics to separate output directories and does
not create new SLAM runs. Raw-source hashes and the scalar are recorded.

Validation: **80 macOS tests pass**, including temporal split separation,
prefix-only fitting despite pathological later ratios, invalid/finite depth
correction handling and preservation of zeros and original inputs. Ruff passes.
Actual replays check decoded units, and actual evaluations verify pose-rescale
Sim(3) invariance, exact common sets and CLI ATE agreement. PNG/PDF figures were
visually inspected as provisional internal research figures.

- [New replay attempts](../../results/da3_prefix_calibration_fr1_xyz/study.json), [full summary](../../results/da3_prefix_calibration_fr1_xyz/summary.json), [per-case CSV](../../results/da3_prefix_calibration_fr1_xyz/per_run.csv)
- [Later common timestamps](../../results/da3_prefix_calibration_fr1_xyz/heldout_common_stamps.json), [figure PDF](../../results/da3_prefix_calibration_fr1_xyz/heldout_comparison.pdf), [figure provenance](../../results/da3_prefix_calibration_fr1_xyz/figure_provenance.json)
- [Calibrated repeat 1 trajectory](../../results/da3_prefix_calibration_fr1_xyz/da3_prefix_rgbd/repeat_001/trajectory.tum), [run manifest](../../results/da3_prefix_calibration_fr1_xyz/da3_prefix_rgbd/repeat_001/manifest.json), [later evaluation](../../results/da3_prefix_calibration_fr1_xyz/evaluations/da3_prefix_rgbd/repeat_001/summary.json)
- [Raw DA3 later evaluation](../../results/da3_prefix_calibration_fr1_xyz/evaluations/da3_rgbd/repeat_001/summary.json), [derived pose-only evaluation](../../results/da3_prefix_calibration_fr1_xyz/evaluations/da3_pose_rescale/repeat_001/summary.json)

The next controlled investigation is depth-constraint weighting and point
selection. These results do not establish the correct uncertainty model,
independent-scene calibration, failure recovery or real-time throughput.
Additional sequences must keep calibration and tuning assumptions explicit,
with download sizes reported before obtaining new assets.

## Software acknowledgement

K-Dense's Experimental Design, Statistical Analysis and Scientific Visualization
skills informed the controls, sampling-unit limits and figures. They are
workflow assistance, not evidence for the measured result: Kassis, T., Agarwal,
V., He, Y., Patel, D., & Brueckner, A. M. (2026). *Scientific Agent Skills:
A Library of Procedural Knowledge for Research Agents*. arXiv:2609.00065.
[https://doi.org/10.48550/arXiv.2609.00065](https://doi.org/10.48550/arXiv.2609.00065).
