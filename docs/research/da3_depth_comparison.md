# DA3 Metric Large versus TUM sensor depth

Date: 2026-11-04. This is a real-data **depth** comparison on one sequence,
performed before running ORB-SLAM3. It does not establish a SLAM improvement.

DA3 reproduces much of the scene's depth structure, but its fixed metric output
systematically underestimates sensor depth on `freiburg1_xyz`. Across all 792
matched frames, frame-averaged AbsRel is **10.22%**, RMSE is **0.1569 m**, and
signed error is **−0.1098 m**. The median frame's median prediction/reference
ratio is **0.8943**. A substantial component of the error is scale bias, although
scalar correction does not remove all spatial errors.

## Data and reproducible protocol

- Sequence: `rgbd_dataset_freiburg1_xyz`, 798 RGB and 798 sensor-depth images.
  All **792** RGB frames with a depth match were evaluated, including frames
  without a matched world pose. The six unmatched RGB frames were excluded from
  this depth comparison and remain present in the dataset loader.
- Time span of evaluated RGB frames: **26.5721 s**. One-to-one closest-first
  association uses a declared 20 ms limit. The observed median RGB/depth time
  difference was **7.32 ms**, maximum **17.23 ms**.
- Model: `depth-anything/DA3METRIC-LARGE`, checkpoint revision
  `4010e39f3634a45bc60553321fb49fb760bd594e`, DA3 source revision
  `3d835ec1a5802d64a8b8b15f817a1ab54809bfe4`. Weight SHA-256 and local source
  patch hash are recorded in the cache manifest.
- Each RGB image is inferred independently, with no sensor depth or world poses
  passed into the network. Camera intrinsics are used for unit conversion, not
  for fitting the primary prediction to reference depth.
- Preprocessing: upstream `upper_bound_resize`, longest side 504; the resulting
  image/depth grid is 378×504. The cache runner reproduces the earlier public-API
  first-frame smoke output exactly (maximum canonical-depth difference 0).
- Known Freiburg 1 RGB calibration comes from the pinned ORB-SLAM3 `TUM1.yaml`.
  The upstream processor transforms that camera matrix alongside the image.
  Processed focal lengths are approximately **407.3788 and 406.7195 px**.
- Conversion is `z_metres = canonical_depth * mean(fx_processed, fy_processed) / 300`,
  giving a factor of **1.3568305**. Metre-valued predictions are then upsampled
  bilinearly with pixel-centre coordinates to the original 480×640 grid. Sensor
  depth is never interpolated. No crop, undistortion or pose-based warping is
  applied.
- Sensor PNG values are divided by 5000. Zero/no-data pixels are excluded; the
  pre-existing Freiburg correction is not applied again. The reference range is
  0.1–10 m; all observed positive sensor values fall within it. Predictions are
  not clipped to this range for scoring.

The comparison scores **184,179,792 reference pixels**, approximately **75.70%**
of the matched RGB image pixels. Predictions were finite and positive at all
scored locations: coverage **100%**, inference failures **0/792**.

Sources: [pinned upstream canonical-depth conversion](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/3d835ec1a5802d64a8b8b15f817a1ab54809bfe4/README.md#-faq),
[TUM sensor formats and calibration](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats),
[pinned RGB calibration](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Monocular/TUM1.yaml).

## Primary result: fixed conversion, no reference scale fitting

The primary aggregation weights every frame equally. Pixel-pooled figures are
also provided so mask-area differences and RMSE aggregation remain explicit.
These figures include every selected frame and all valid reference pixels.

| Metric | Equal-frame mean | Pixel-pooled |
| --- | ---: | ---: |
| AbsRel | **10.224%** | 10.226% |
| MAE | 0.1212 m | 0.1211 m |
| RMSE | **0.1569 m** | 0.1620 m |
| Signed error, prediction − sensor | **−0.1098 m** | −0.1096 m |
| Squared relative error | 0.01781 m | 0.01782 m |
| Natural-log RMSE | 0.12294 | 0.12607 |
| δ₁ | 98.302% | 98.290% |
| δ₂ | 99.415% | 99.411% |
| δ₃ | 99.863% | 99.862% |

Definitions: `AbsRel = mean(|p−g|/g)`, `SqRel = mean((p−g)²/g)`, and
`δ_k = fraction(max(p/g, g/p) < 1.25^k)`. RMSE and log-RMSE are computed per frame
before equal-frame averaging; pixel-pooled RMSE uses the combined squared-error
sum. Invalid predictions would reduce coverage and count as failures for δ;
absolute-error metrics would use the valid intersection. No predictions were
invalid in this run.

Every frame's median prediction/reference ratio was below one. The range was
**0.8205–0.9861**, with an interquartile range of **0.8771–0.9228**. This is
view-dependent single-image depth-scale variability, not a measured SLAM scale
drift trajectory.

## Ground-truth scale diagnostics

These results explicitly use sensor depth to estimate scale. They are not the
unfitted metric baseline and must not be presented as deployable RGB-only
absolute-scale recovery.

| Same 792-frame set | AbsRel | RMSE | Signed error |
| --- | ---: | ---: | ---: |
| Fixed metric conversion | 10.224% | 0.1569 m | −0.1098 m |
| Oracle median-ratio scale fitted separately on each evaluated frame | 3.485% | 0.1044 m | +0.0049 m |

Per-frame oracle scaling reduces AbsRel by approximately **65.9%**, supporting
the interpretation that scale bias is a major error component. The remaining
3.49% AbsRel and 10.44 cm RMSE show that a scalar does not remove all spatial
discrepancies. Median-ratio fitting is not optimal for every metric: for example,
δ₃ decreases slightly, from 99.863% to 99.787%, after this correction.

A second diagnostic predeclared the first 20% of matched frames as calibration:
**158 calibration frames**, followed by **634 held-out temporal frames**. The
scale is the median of the calibration frames' median `sensor/prediction` ratios.
The fitted multiplier was **1.1093010**. Calibration ends at RGB timestamp
`1305031107.611271`; held-out evaluation begins at `1305031107.643323`.

| Same held-out 634-frame set | AbsRel | RMSE | Signed error |
| --- | ---: | ---: | ---: |
| Fixed conversion | 10.247% | 0.1541 m | −0.1092 m |
| One GT-anchored prefix scale | **4.873%** | **0.1053 m** | +0.0061 m |

This demonstrates useful transfer across later views of this same desk scene.
It does not establish transfer to a different sequence, camera or environment.
The calibration anchor is sensor ground truth; it does not resolve the
unanchored monocular scale/bias gauge through regularization alone.

## Distance and sensitivity findings

Absolute error increases with sensor distance. Within well-populated bins,
relative error stays roughly 10–12%, consistent with a substantial multiplicative
bias component.

| Sensor depth | Scored pixels | MAE | Mean signed error | AbsRel |
| --- | ---: | ---: | ---: | ---: |
| 0.5–1.0 m | 96,476,408 | 0.0847 m | −0.0776 m | 9.86% |
| 1.0–1.5 m | 61,027,814 | 0.1233 m | −0.1099 m | 10.31% |
| 1.5–2.0 m | 12,070,253 | 0.2079 m | −0.1792 m | 12.29% |
| 2.0–3.0 m | 11,375,160 | 0.2478 m | −0.2256 m | 10.18% |
| 3.0–4.0 m | 3,230,050 | 0.3947 m | −0.3940 m | 12.02% |

**85.52%** of scored pixels lie between 0.5 and 1.5 m. Only **107** scored sensor
pixels exceed 4 m; their errors remain in all primary metrics, but are far too
sparse to support a far-range accuracy claim. The overview's distance-trend
panel omits bins with fewer than 1000 pixels and says so explicitly; full bins
remain in `summary.json`.

Two separate sensitivity checks were fixed in the evaluation script:

- Eroding the reference validity mask by one pixel, including the image border,
  changes AbsRel from **10.224% to 10.175%** and RMSE from 0.1569 to 0.1523 m.
  Pixels adjacent to sensor holes alone therefore do not explain the scale bias.
  This erosion does not remove every object-depth boundary.
- Using a fixed nominal original focal length of 525 px instead of the selected
  measured RGB calibration increases depths by **1.5694%**. AbsRel becomes
  **8.966%**, RMSE 0.1445 m and signed error −0.0933 m. This calibration choice
  matters, but does not eliminate the observed underestimation. It is a reported
  sensitivity, not a camera calibration selected by optimizing test results.

## Figures and practical implications

Local generated figures are gitignored, with PNG and PDF versions:

- [Overview](../../results/depth_comparison_fr1_xyz/figures/depth_comparison_overview.png)
  ([PDF](../../results/depth_comparison_fr1_xyz/figures/depth_comparison_overview.pdf)):
  depth scatter density, per-frame bias ratio, populated distance bins and signed
  relative errors.
- [First/middle/last examples](../../results/depth_comparison_fr1_xyz/figures/depth_comparison_examples.png)
  ([PDF](../../results/depth_comparison_fr1_xyz/figures/depth_comparison_examples.pdf)):
  RGB, sensor metres, predicted metres and signed relative errors. Frame indices
  0/395/791 were chosen in the protocol before scoring. Depth display is clipped
  to 0–4 m and relative-error display to ±50%; metrics are not clipped.

The examples show large desk/monitor surfaces in the predictions and predominantly
negative relative error. Object edges have localized errors. Dense predictions
inside sensor holes are visible but cannot be validated using this reference.

For the upcoming ORB comparison, keep the fixed-conversion run as the metric
baseline. Any GT-calibrated run must be a separately labelled diagnostic or
explicitly anchored method. A single scalar improves this sequence substantially,
but viewpoint variability and residual spatial errors remain. Reject invalid
predictions and retain the conversion/alignment metadata when preparing RGB-D
inputs. These results do not demonstrate that depth assistance improves tracking.

Limitations include one short, highly correlated indoor sequence, a mostly
near-range reference, missing sensor depth, RGB/depth time offsets and residual
sensor/registration error. No independent sequence, ground-truth re-warping,
CUDA execution, confidence calibration or SLAM accuracy was evaluated. No
checkpoint training-set overlap audit with TUM was performed; "held-out" here
refers only to the temporal scale-calibration split. No
pass/fail threshold for declaring general metric accuracy was pre-registered;
`metric_scale_validated` remains false in the model configuration.

## Reproduce and inspect artifacts

```bash
HF_HUB_OFFLINE=1 .venv-da3-macos/bin/python scripts/cache_da3_depth.py --device mps
.venv/bin/python scripts/evaluate_da3_depth.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_depth_comparison.py
.venv/bin/python -m pytest -q
```

The protocol is `configs/evaluations/da3_tum_fr1_depth.yaml`. Prediction files and
their provenance manifest are under `predictions/da3_metric_large/fr1_xyz/`.
Full scores, per-frame JSON/CSV, histograms and figures are under
`results/depth_comparison_fr1_xyz/`. File/config/source hashes preserve the
evaluated revisions. Cache reuse verifies both input and prediction hashes.

All **56 tests pass** on the Mac. The inference run used native Metal on the
Apple M4 and PyTorch 2.15.0. Recorded inference/preprocessing wall time totals
381.05 s, with median 0.4634 s/frame. This excludes loading/serialization, mixes
cold and warm calls across the initial two-frame pilot and resumed run, and is
not a controlled real-time pipeline benchmark. No additional assets were
downloaded for this comparison.
