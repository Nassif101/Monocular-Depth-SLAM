# Codex handoff: ORB-SLAM3 + DA3 baseline

**Date:** 2026-11-04
**Status:** development dependencies bootstrapped on the Mac and Ubuntu ARM64
container; TUM loader, full matched-sequence depth comparison and RGB-only
ORB-SLAM3 trajectory/evaluation and the first DA3-assisted RGB-D comparison
complete. The 792-frame sensor-depth control also completed nine runs (three
per method). Uncertainty-aware fusion and broader validation remain future work.
The GT-prefix calibration diagnostic also completed three new runs with separate
later-frame scoring. See [prefix calibration](research/da3_prefix_calibration.md),
[sensor-control results](research/sensor_depth_control.md),
[DA3 RGB-D results](research/da3_orbslam3_rgbd_baseline.md),
[monocular results](research/orbslam3_monocular_baseline.md), [tested environment](environment.md)
and [data/checkpoint setup](data_and_checkpoint.md).

## Objective

Create a reproducible **real-data** baseline using ORB-SLAM3 monocular and Depth Anything 3 (DA3) on TUM RGB-D sequences. The initial depth-assisted experiment may use ORB-SLAM3's RGB-D interface, but its depth conversion and calibration must be documented. This is **not** the final research factor graph.

## Required reading

- `README.md`, `docs/setup.md`, `docs/methodology.md`
- `docs/research/README.md`, `docs/research/findings.md`, `docs/research/experiments.md`, `docs/research/literature.md`
- `experiments/baseline_001/README.md`

## Implementation order

1. **Container verified locally:** Ubuntu 24.04 ARM64 on the Apple M4. Docker build, bind-mount writes, Python tests and headless Pangolin passed. NVIDIA runtime remains untested.
2. **ORB-SLAM3 pinned and built:** source revisions, build scripts and explicit compatibility/shutdown patches are checked in; vocabulary SHA-256 and installed package versions are recorded under `results/setup/`. The project-owned headless runner builds under `build/monocular_runner/` and has completed the real sequence.
3. **TUM loader validated:** `datasets/tum.py` parses indices/poses, keeps every RGB frame, associates one-to-one with declared tolerances, validates camera/image metadata and converts sensor depth correctly. All 1596 PNGs from `freiburg1_xyz` decoded; 792 RGB/depth matches, 796 RGB/pose matches. 43 tests pass on Mac and Ubuntu ARM64.
4. **Monocular baseline complete:** `scripts/run_monocular.py` generated 795 poses from all 798 RGB inputs in `results/orbslam3_mono_fr1_xyz/run_001/`. Three initializing frames, no later tracking losses, one map and 53 keyframes. Optimized largest-map exports use restored exact RGB timestamps; logs and source snapshots are retained. Never overwrite this run.
5. **DA3 predictions cached:** all 792 sensor-matched RGB frames have canonical depth and exact processed intrinsics in `predictions/da3_metric_large/fr1_xyz/`. The six remaining RGB frames are now cached separately in `fr1_xyz_extra_rgb/` using the same environment/checkpoint. Verified fixed-conversion float32 depth for all 798 frames is in `fr1_xyz_orb_rgbd/`. Cache provenance checks revisions, weights, inputs and output hashes; no sensor depth/poses enter inference or conversion.
6. **Depth comparison complete on fr1_xyz:** fixed conversion gives 10.22% AbsRel, 0.1569 m frame-averaged RMSE and −0.1098 m signed error. Every frame's median depth ratio is below one. Per-frame oracle scaling gives 3.48% AbsRel; a GT prefix scale fitted on 158 frames gives 4.87% on the remaining 634. Those calibrated scores are separate GT-anchored diagnostics. No independent-sequence/general metric validation or SLAM improvement is established. See `docs/research/da3_depth_comparison.md`.
7. **Depth-assisted baseline complete:** `scripts/run_da3_rgbd.py` produced 798 poses, first-frame initialization, no later losses, one map and 76 keyframes in `results/orbslam3_da3_rgbd_fr1_xyz/run_001/`. Fixed DA3 conversion, no GT depth correction; aligned float32 TIFF uses `RGBD.DepthMapFactor=1` and upstream TUM1 RGB-D parameters. All OpenCV depth decode checks passed. Do not overwrite run_001.
8. **Common-set comparison complete:** 793 identical reference-matched poses and 733 identical RPE pairs. Monocular vs DA3: SE(3) ATE 0.049830 vs 0.022218 m; Sim(3) ATE 0.009283 vs 0.011687 m; Sim(3) 1 s translation RPE 0.014720 vs 0.020116 m; rotation RPE 0.956560 vs 1.135417 degrees. GT-fitted scale factors 1.357883 vs 1.113338. Nominal scale is closer, but scale-corrected errors worsen in this single comparison. Full DA3 scoring uses 796 poses and 736 RPE pairs, reported separately. Manifests, source snapshots, CLI ATE crosschecks, per-pose scores and PNG/PDF figures exist; 71 tests pass on macOS. Common comparison is under `results/trajectory_comparison_fr1_xyz/`.
9. **Ablations**: fixed-budget DA3 inference vs information-aware scheduling with **equal inference calls**, only after the basic baseline works.

## Sensor-depth control completed

`configs/evaluations/sensor_depth_control.yaml` schedules three fresh processes
per method on the identical 792 sensor-matched RGB frames, in balanced order.
All nine completed under `results/sensor_depth_control_fr1_xyz/`. Both RGB-D
methods export 792 poses in every run; monocular exports 788 after four-frame
initialization. All runs have one map and zero later tracking losses.

Study-wide comparison refits all runs on 786 identical GT-matched poses and
725 identical RPE pairs. Median monocular / DA3 / sensor Sim(3) ATE is
9.371 / 11.721 / 10.471 mm; median SE(3) ATE is 3.501 / 2.245 / 1.066 cm.
Median Sim(3) translation RPE is 1.523 / 2.043 / 1.747 cm. GT-fitted scales
are 1.2211 / 1.1139 / 1.0108. Individual scores, medians, observed ranges,
mean/sample SD and PNG/PDF figures are preserved. These are process repeats on
one scene, not independent-scene replications. Tests: 74 pass on macOS.

Input selection keeps full-loader GT associations fixed. RPE gap indices now
refer to the submitted timeline; intentional source-image omissions are shared
inputs, while missing submitted estimates/GT still break pairs. Per-pose CSVs
retain original source RGB indices separately. Sensor PNG depth is converted
by /5000 to float32 TIFF with holes preserved. Sensor depth enters only the
explicitly labeled sensor control, and world poses enter scoring only.

## GT-prefix calibration diagnostic completed

`configs/evaluations/prefix_calibration.yaml` freezes the previously measured
1.1093010159475212× correction from the first 158 sensor-matched frames. The
preparer recomputes it using only prefix sensor pixels, verifies exact agreement,
and writes a separate labeled cache. Three new offline RGB-D replays in
`results/da3_prefix_calibration_fr1_xyz/` export 792 poses each with one map and
no losses. The factor is applied retrospectively, including initialization;
this is not a causal online or unanchored metric method.

The following 634 RGB frames give 632 common GT matches and 572 identical RPE
pairs across the 3 new and 9 historical processes. All alignments/scores exclude
the prefix. Raw vs prefix-replayed DA3 median SE(3) ATE is 2.302 vs 1.066 cm;
Sim(3) ATE is 11.387 vs 10.660 mm; Sim(3) translation RPE is 2.064 vs 1.960 cm.
Additional GT-fitted scale is 1.1162 vs 1.0007. Pose-only rescaling of the three
existing DA3 trajectories gives 1.139 cm SE(3) ATE and invariant Sim(3) metrics,
so most of the metric gain is explained by scale correction. Monocular and
sensor later-set Sim(3) ATE remain lower, at 9.050 and 10.140 mm. Means/SD,
individual scores, ranges, PNG/PDF figures and CLI checks are retained; 80 tests
pass on macOS. Old control artifacts are preserved.

## Acceptance criteria for first PR

- Dataset loader tested on at least one TUM RGB-D sequence.
- ORB-SLAM3 monocular trajectory generated and evaluated with reproducible commands.
- Manifest records exact dependency revisions and hardware.
- No ground-truth depth is silently used for deployable inference.
- A fresh checkout can follow documented steps; no claim of successful DA3 integration until demonstrated.

## Scientific constraints

The scale/bias gauge is real: freely optimizing depth prediction bias does not make metric scale observable. Do not infer absolute metric scale from regularization alone. A prior or external metric anchor must be explicit. See `docs/research/findings.md`.

## Operational notes

- New GitHub repo was found **public**; avoid secrets and unpublished sensitive materials.
- Template repo remains separate. Its README was updated to generic robotics documentation.
- Default dev container is CPU-only and intentionally stripped of unrelated Unity/MoveIt/Gazebo dependencies. ORB-SLAM3 is built in ignored `third_party/`; native DA3 is installed in `.venv-da3-macos`. The pretrained Metric Large checkpoint is under `.cache/models/`; TUM data is under `datasets/tum_rgbd/`. CUDA dependencies are locked for a later Linux x86_64 NVIDIA host and remain untested there.
- User preference: report download sizes before any additional data/checkpoint downloads. The TUM `freiburg1_xyz` archive plus pinned Metric Large files (1,784,939,566 bytes total) were explicitly approved and downloaded. `download_baseline.py` previews sizes without network access unless `--download` is supplied.
- Research skills: 10 selected K-Dense skills (including scientific-writing, critical thinking and SymPy) plus OpenAI PDF/Jupyter skills are installed at user scope. See `docs/research_skills.md` and `configs/research_skills.json`. Their runtime dependencies/API services were not installed or connected; use appropriate isolated environments when needed. Skill installation does not change the evidence standards or authorize external publication/uploads.
- Start with offline cached depth and evaluation; real-time ROS 2 integration comes later.
- Next evidence: depth-constraint weighting/point-selection ablations and independent-sequence validation. Prefix calibration, sensor and repeat controls are complete; they do not isolate one causal source of error or establish general tracking robustness/real-time throughput. Preview any further download sizes first.
