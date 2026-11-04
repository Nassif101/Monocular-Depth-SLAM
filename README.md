# Monocular Depth SLAM

Research repository for benchmarking **ORB-SLAM3 + Depth Anything 3 (DA3)** against monocular ORB-SLAM3, then investigating depth calibration, uncertainty-aware fusion, and scale observability.

**Status:** development dependencies bootstrapped on Apple Silicon and the Ubuntu
ARM64 container; ORB-SLAM3 builds.
The TUM loader is validated on `freiburg1_xyz`, and a pinned pretrained DA3 Metric
Large checkpoint has been evaluated against all 792 sensor-matched frames using
Metal: 10.22% AbsRel and 0.1569 m frame-averaged RMSE with fixed metre conversion.
See [depth findings](docs/research/da3_depth_comparison.md). The RGB-only
[ORB-SLAM3 baseline](docs/research/orbslam3_monocular_baseline.md) now tracks
795/798 frames, with 0.04983 m SE(3) ATE RMSE and 0.00928 m Sim(3) ATE RMSE
(GT-fitted scale; one run). The first
[DA3 + ORB-SLAM3 RGB-D run](docs/research/da3_orbslam3_rgbd_baseline.md) tracks
798/798 frames. On identical timestamps it gives 0.02222 m SE(3) ATE and
0.01169 m Sim(3) ATE: closer nominal scale, but worse scale-corrected ATE/RPE
in this single comparison. A Linux/NVIDIA setup is prepared
but has not been tested on GPU hardware.

The [sensor-depth control](docs/research/sensor_depth_control.md) now repeats
all three methods three times on the same 792 RGB frames. On 786 common poses,
median Sim(3) ATE is 9.37 mm monocular, 11.72 mm DA3 RGB-D and 10.47 mm sensor
RGB-D. Sensor depth gives the closest metric scale; monocular retains the
lowest scale-corrected errors on this sequence. All nine runs completed.

The [GT-prefix calibration diagnostic](docs/research/da3_prefix_calibration.md)
then fits depth scale on the first 158 frames and scores 632 later matched
poses. Calibrated replay gives 1.07 cm median SE(3) ATE versus 2.30 cm raw;
pose-only rescaling already gives 1.14 cm. Sim(3) ATE improves modestly from
11.39 to 10.66 mm, with monocular still lower at 9.05 mm. This uses an explicit
sensor-GT calibration anchor and offline replay.

## Project origin and expanded scope

This repository is a **remake and continuation of my earlier Construction Robotics project**, rebuilt with a broader research scope. Rather than reproducing the original construction-robotics implementation unchanged, it establishes a dedicated, reproducible experimental platform for visual perception and SLAM.

The initial focus is **monocular SLAM augmented with learned depth**, starting with ORB-SLAM3 and Depth Anything 3. The extended scope includes metric-scale observability, depth calibration and bias, uncertainty-aware fusion, robust estimation, adaptive depth-inference scheduling, and systematic benchmarking. These methods may ultimately support robotic perception and navigation in construction environments, but this repository evaluates the underlying SLAM methods independently first.

## First experiment

1. Run ORB-SLAM3 monocular on TUM RGB-D RGB frames.
2. Cache DA3 depth predictions with timestamps, intrinsics, model revision, depth convention, and units.
3. Check prediction-to-ground-truth alignment and bias. Oracle depth calibration must be labelled as such.
4. Explore ORB-SLAM3's RGB-D path with converted DA3 predictions, documenting any changes and limitations.
5. Compare SE(3) ATE, Sim(3) ATE, RPE, scale drift, tracking failures, and runtime.

**Caution:** DA3 predictions are not automatically metric or compatible with ORB-SLAM3 RGB-D depth. Validate model output and scale before use.

## Structure

- `configs/`: camera, dataset, SLAM and depth settings
- `datasets/`: local dataset files (gitignored)
- `predictions/`: generated depth maps (gitignored)
- `evaluations/`: evaluation protocols and scripts
- `experiments/`: baseline and ablation manifests
- `docs/`: setup, architecture, methodology, research log
- `scripts/`: CLI entrypoints and dataset utilities
- `src/monocular_depth_slam/`: adapters and future fusion code
- `tests/`: reproducible checks
- `third_party/`: external code (not vendored)
- `results/`: run artifacts (gitignored)

## Quick start

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) first
(`brew install uv` on macOS), then install the locked Python 3.12 environment:

```bash
bash scripts/setup_environment.sh
source .venv/bin/activate
pytest
python scripts/check_environment.py
```

For the Ubuntu C++ environment, open in VS Code and choose **Reopen in
Container**. It installs development and evaluation packages into `.venv-linux`
separately from the macOS `.venv`. The image includes OpenCV, Eigen, Boost and
build tools; ORB-SLAM3 and DA3 require the additional steps in
[docs/setup.md](docs/setup.md). The headless monocular runner, evaluation commands
and measured results are documented in the
[monocular baseline report](docs/research/orbslam3_monocular_baseline.md).
The [DA3 RGB-D report](docs/research/da3_orbslam3_rgbd_baseline.md) documents
depth preparation, replay and comparison using the same local assets.

See [data and checkpoint setup](docs/data_and_checkpoint.md) for download sizes,
the TUM loader, real-data validation commands and the pinned pretrained model.

## Research discipline

Keep model and dependency revisions pinned in run metadata; never evaluate metric scale only after Sim(3) alignment. Do not commit datasets, checkpoint weights, or generated outputs.

The installed [research skills](docs/research_skills.md) cover scientific writing,
claim/citation review, experimental design, statistics, figures and symbolic math.
Their selected source revisions are recorded in `configs/research_skills.json`.

## License

MIT. See [LICENSE](LICENSE). Third-party components have independent licenses.
