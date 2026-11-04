# Baseline 001: ORB-SLAM3 monocular vs DA3-assisted RGB-D

**Status:** data/depth preparation, monocular trajectory and the first
DA3-assisted RGB-D comparison complete on one sequence, one run per method.

Dataset: TUM RGB-D `rgbd_dataset_freiburg1_xyz` (downloaded and validated).
Controls: identical RGB frames and timestamps; report SE(3) and Sim(3) separately.

Deliverables: trajectories, depth metadata, association file, evaluation JSON, timing logs and a failure report.

Data preparation: 798 RGB and 798 depth PNGs decoded; 3000 ground-truth poses.
At 20 ms one-to-one association tolerance, 792 RGB/depth and 796 RGB/pose pairs
were matched. The loader retains all RGB frames. Association/inspection outputs
are under `results/tum_freiburg1_xyz/`.

DA3 Metric Large is pinned in `configs/da3/metric_large.yaml`. Predictions for all
792 matched frames are cached under `predictions/da3_metric_large/fr1_xyz/`.
Metre conversion using processed intrinsics and bilinear prediction alignment
to unchanged sensor pixels are evaluated in `results/depth_comparison_fr1_xyz/`.
Fixed-conversion frame means are 10.22% AbsRel and 0.1569 m RMSE. GT-fitted
oracle/temporal-scale diagnostics are reported separately. See
`docs/research/da3_depth_comparison.md` for findings and limitations. RGB-D
tracking improvement is not established by this depth comparison.

The RGB-only run is under `results/orbslam3_mono_fr1_xyz/run_001/` using
`configs/orbslam3/monocular_baseline.yaml`. It produced 795 poses from 798 RGB
inputs, with three initialization frames and no later tracking loss. SE(3) ATE
RMSE is 0.049830 m; Sim(3) ATE RMSE is 0.009283 m after GT-fitted scale
1.357883 m/native unit. Sim(3) 1 s RPE RMSE is 0.014720 m / 0.956560 degrees.
The [monocular report](../../docs/research/orbslam3_monocular_baseline.md)
documents export, association, coverage, scale diagnostics, runtime and limits.
Use new run directories for repeats; run_001 is preserved.

The depth-assisted run is under `results/orbslam3_da3_rgbd_fr1_xyz/run_001/`
using `configs/orbslam3/da3_rgbd_baseline.yaml`. It consumes all 798 RGB frames
and fixed-conversion learned depth, including six supplemental predictions for
previously uncached RGB frames. It exports 798 poses with first-frame
initialization and no later loss. No GT depth scale is applied.

On the common 793 matched timestamps, DA3-assisted SE(3) ATE is 0.022218 m,
Sim(3) ATE is 0.011687 m, and Sim(3) 1 s RPE is 0.020116 m / 1.135417 degrees.
Its GT-fitted scale is 1.113338. Scale-corrected errors exceed the monocular
run's despite closer nominal scale. See the
[comparison report](../../docs/research/da3_orbslam3_rgbd_baseline.md) and the
[protocol recorded before the run](../../docs/research/da3_rgbd_protocol.md).

The subsequent [sensor-depth control](../../docs/research/sensor_depth_control.md)
uses a separate 792-frame input timeline shared by all methods. Three processes
per method completed successfully. The common set across all nine runs has
786 GT-matched poses and 725 identical RPE pairs. Median Sim(3) ATE is
9.371 mm monocular, 11.721 mm DA3 and 10.471 mm sensor RGB-D; all later loss
counts are zero. These results stay separate from the earlier 798-frame runs.
Study schedule, per-run outputs and summary are under
`results/sensor_depth_control_fr1_xyz/`.

The [GT-prefix calibration diagnostic](../../docs/research/da3_prefix_calibration.md)
adds three calibrated DA3 replays and derives pose-only rescaled scores from
the raw runs. Prefix length 158 and factor 1.1093010159475212 are frozen;
scoring/refitting uses only 632 common later poses and 572 RPE pairs. Raw vs
calibrated DA3 median SE(3) ATE is 2.302 vs 1.066 cm, Sim(3) ATE 11.387 vs
10.660 mm. The pose-only diagnostic reaches 1.139 cm SE(3) ATE with unchanged
Sim(3) errors. The GT-anchored offline result stays separate from unfitted
depth and the earlier all-sequence trajectory scores.
