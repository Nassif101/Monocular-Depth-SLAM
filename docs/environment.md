# Tested development environment

Date: 2026-11-04. Hardware: MacBook Air, Apple M4, 16 GB unified memory,
macOS 26.6.2. Setup commands are in [setup.md](setup.md).

| Environment | Installed and checked |
| --- | --- |
| macOS development, `.venv` | Python 3.12.14; NumPy 2.5.4; pytest 9.2.0; Ruff 0.17.3; evo 1.38.1; SciPy 1.18.1; Matplotlib 3.11.2 |
| Ubuntu 24.04 ARM64, `.venv-linux` | Python 3.12.3; the same locked development/evaluation packages; GCC 13.3; CMake 3.28.3; OpenCV 4.6.0; Eigen 3.4.0; Boost 1.83; OpenSSL 3.0.13; Xvfb |
| macOS DA3, `.venv-da3-macos` | PyTorch 2.15.0; torchvision 0.29.2; NumPy 1.26.4; OpenCV Python 4.11.0.86; Open3D 0.20.0; native Metal (`mps`) |
| Future Linux x86_64 NVIDIA, `.venv-da3-cuda` | Lock resolves PyTorch 2.11.0+cu128, torchvision 0.26.0+cu128 and xFormers 0.0.35. Not installed or executed on this Mac. |

Host tools installed with Homebrew: uv 0.12.19, CMake 4.4.3, Ninja 1.13.2,
pkgconf 3.0.7 and libusb 1.0.30. Docker Desktop and VS Code were already present.
The project-recommended VS Code extensions are installed.

Source pins (full hashes are in `configs/dependencies.json`):

- ORB-SLAM3: `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`
- Pangolin v0.6: `dd801d244db3a8e27b7fe8020cd751404aa818fd`
- Depth Anything 3: `3d835ec1a5802d64a8b8b15f817a1ab54809bfe4`

Validation performed:

- TUM/setup suite: 43 passed on macOS and Ubuntu ARM64, including parsing,
  timestamp association, missing files, sensor depth units and calibration.
  The expanded depth/trajectory/adapter suite has 80 passing tests on macOS.
- Synthetic scale/bias gauge sanity check: passed on both platforms.
- Project Ruff checks and both lock freshness checks: passed.
- Mac development and DA3 package compatibility checks: passed.
- ORB-SLAM3 `mono_tum`, `rgbd_tum` and shared library: compiled; linkage and
  executable startup checked in Ubuntu.
- The project-owned headless monocular runner completed all 798 RGB inputs from
  `freiburg1_xyz` in Ubuntu ARM64. It exported 795 tracked poses, with three
  initialization frames and no later losses. The [trajectory report](research/orbslam3_monocular_baseline.md)
  records SE(3)/Sim(3) scoring and the exact cached container/binary provenance.
- The headless DA3 RGB-D runner completed the same 798 RGB inputs with 798
  learned-depth maps. It exported all poses, initialized on the first frame,
  and had no later losses. Float32 OpenCV depth decode checks passed for all
  frames. The [comparison report](research/da3_orbslam3_rgbd_baseline.md) shows
  closer nominal scale but worse scale-corrected ATE/RPE in this single run.
- The [sensor-depth control](research/sensor_depth_control.md) completed all nine
  sequential 792-frame replays: three per method, identical inputs and balanced
  order. Sensor/DA3 depth use one shared control executable with float32 metre
  input; all decode checks passed. All processes had no later tracking loss.
  Observed repeat ranges are documented; general repeatability across scenes,
  seeds or machines remains unestablished.
- Three additional GT-prefix-calibrated DA3 runs completed on the same 792
  inputs without changing the C++ executable/libraries. All poses exported and
  all decode checks passed. The [later-frame diagnostic](research/da3_prefix_calibration.md)
  excludes calibration poses from alignment/scoring and verifies pose-only
  Sim(3) rescaling invariance. It remains an offline GT-anchored experiment.
- Pangolin: a small software OpenGL window opened and finished a frame under
  Xvfb. Mesa reported a framebuffer fallback; hardware rendering was not tested.
- DA3 API import, Metal matrix operation and DA3-Small random-weight forward pass:
  passed, returning finite depth with shape `[1, 1, 56, 56]`.
- Open3D import: passed after installing libusb.
- Real-data TUM validation: all 798 RGB and 798 sensor depth PNGs decoded;
  index/association details are in [data and checkpoint setup](data_and_checkpoint.md).
- Pinned pretrained DA3 Metric Large: one real TUM RGB frame ran on Metal,
  producing finite canonical depth at 378×504. No metre conversion or accuracy
  evaluation was performed in that original smoke check. The subsequent
  [792-frame comparison](research/da3_depth_comparison.md) implements conversion
  and reports 10.22% AbsRel and 0.1569 m frame-mean RMSE without reference scaling.

Compatibility patches are preserved under `patches/`. DA3's pycolmap exporter is
loaded lazily to avoid a Mac OpenMP conflict; COLMAP export and upstream
reconstruction benchmarks remain unvalidated. Gaussian-splatting extras are not
installed. See [setup.md](setup.md) for details.

Local reports/logs live under `results/` and `results/setup/`, including source
diffs, package versions, vocabulary checksum and smoke-check output. They are
gitignored. The approved TUM sequence and pinned Metric Large checkpoint are now
downloaded and verified. Depth accuracy has now been characterized on this
sequence, and an RGB-only monocular trajectory has been generated and evaluated.
The first fixed-conversion DA3 RGB-D comparison is complete. Independent-sequence
metric generalization, general repeated-run stability, controlled real-time pipeline
performance and CUDA execution remain unvalidated.
