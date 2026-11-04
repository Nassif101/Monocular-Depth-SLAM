# Monocular Depth SLAM

Research repository for benchmarking **ORB-SLAM3 + Depth Anything 3 (DA3)** against monocular ORB-SLAM3, then investigating depth calibration, uncertainty-aware fusion, and scale observability.

**Status:** repository scaffold. No baseline has been run or validated.

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

Open in VS Code with the Dev Containers extension and choose **Reopen in Container**. The lightweight container includes Python and C++ build tools, not ORB-SLAM3 or DA3 itself.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
pytest
python scripts/check_environment.py
```

See [docs/setup.md](docs/setup.md) for external dependency and GPU instructions. The baseline runner is not yet implemented.

## Research discipline

Keep model and dependency revisions pinned in run metadata; never evaluate metric scale only after Sim(3) alignment. Do not commit datasets, checkpoint weights, or generated outputs.

## License

MIT. See [LICENSE](LICENSE). Third-party components have independent licenses.
