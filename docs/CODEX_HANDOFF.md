# Codex handoff: ORB-SLAM3 + DA3 baseline

**Date:** 2026-11-04
**Status:** scaffold committed; no functional SLAM/depth pipeline yet.

## Objective

Create a reproducible **real-data** baseline using ORB-SLAM3 monocular and Depth Anything 3 (DA3) on TUM RGB-D sequences. The initial depth-assisted experiment may use ORB-SLAM3's RGB-D interface, but its depth conversion and calibration must be documented. This is **not** the final research factor graph.

## Required reading

- `README.md`, `docs/setup.md`, `docs/methodology.md`
- `docs/research/README.md`, `docs/research/findings.md`, `docs/research/experiments.md`, `docs/research/literature.md`
- `experiments/baseline_001/README.md`

## Implementation order

1. **Verify the container** on the actual machine, including Docker build, permissions and optional NVIDIA runtime. Fix build issues, document tested platform.
2. **Pin ORB-SLAM3** source revision, build dependencies and vocabulary; provide reproducible build script. Do not vendor large upstream sources or change upstream silently.
3. **TUM loader**: parse `rgb.txt`, `depth.txt`, `groundtruth.txt`; timestamp association with tolerance; correct camera intrinsics, image size and TUM depth scaling; tests for missing and mismatched timestamps.
4. **Monocular baseline**: execute ORB-SLAM3 on RGB only; export trajectory in TUM format and record tracking failures.
5. **DA3 inference**: pin upstream commit, checkpoint, model variant, input transforms, output representation and units. Cache predictions by frame timestamp with metadata; handle invalid depth and masks.
6. **Depth validation**: evaluate DA3 against TUM sensor depth; document whether calibration uses ground truth (oracle). Avoid leakage between calibration and test sequences.
7. **Depth-assisted baseline**: prepare aligned depth input for ORB-SLAM3 RGB-D. Confirm scaling, pixel alignment, invalid pixels and depth range. Track all patches to ORB-SLAM3.
8. **Evaluation**: compute SE(3) ATE, Sim(3) ATE, RPE, scale drift, trajectory coverage, failure rate and inference runtime. Store per-run manifest with dataset, sequence, commits, checkpoint, calibration, hardware, seeds and alignment.
9. **Ablations**: fixed-budget DA3 inference vs information-aware scheduling with **equal inference calls**, only after the basic baseline works.

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
- Default dev container is CPU-only and intentionally stripped of unrelated Unity/MoveIt/Gazebo dependencies. ORB-SLAM3, DA3 and GPU runtime are **not** installed.
- Start with offline cached depth and evaluation; real-time ROS 2 integration comes later.
