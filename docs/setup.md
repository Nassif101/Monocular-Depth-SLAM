# Setup

Use native macOS for Python development and DA3 inference, and the Ubuntu 24.04
dev container for ORB-SLAM3. The same container also builds on a Linux PC. DA3
has a separate Linux x86_64 CUDA environment for the later NVIDIA machine.

Source revisions are in [`configs/dependencies.json`](../configs/dependencies.json).
Python packages are pinned in `uv.lock` and `environments/da3/uv.lock`. Keep these
files when moving between machines; copy neither virtual environments nor compiled
third-party libraries between operating systems or CPU architectures.

## macOS tools and Python

Install Docker Desktop, VS Code and Apple's Command Line Tools if missing. With
Homebrew available:

```bash
brew install uv cmake ninja pkgconf libusb
bash scripts/setup_environment.sh
source .venv/bin/activate
pytest
python scripts/check_environment.py
```

The setup script installs Python 3.12 when needed and installs development and
evaluation dependencies, including NumPy, PyYAML, pytest, Ruff, evo, SciPy and
Matplotlib. Downloaded runtimes/caches live in the ignored `.cache/` directory;
the system Python is not used for project packages.

## Ubuntu development container

Start Docker Desktop on macOS. In VS Code, install the **Dev Containers**
extension, then choose **Dev Containers: Reopen in Container**. Python, C++,
CMake and YAML extensions are declared in the container configuration.

The image includes GCC, CMake, Ninja, pkg-config, OpenCV, Eigen, OpenGL/GLEW,
Boost serialization, OpenSSL, and Xvfb for a headless display. Its post-create command
installs the locked Python dependencies into `.venv-linux`, preserving the
native macOS `.venv`.

For a terminal-only session on this Mac:

```bash
docker build -t monocular-depth-slam:dev -f .devcontainer/Dockerfile .
docker run --rm -it \
  --mount "type=bind,source=$PWD,target=/workspace" \
  monocular-depth-slam:dev bash
bash scripts/setup_environment.sh
source .venv-linux/bin/activate
pytest
```

On a Linux host, add `--user "$(id -u):$(id -g)"` to the `docker run` command
to write bind-mounted files as the host user. Dev Containers handles user ID
mapping automatically.

## ORB-SLAM3 and Pangolin

Run inside the Ubuntu container, after Python setup:

```bash
bash scripts/setup_slam.sh
python scripts/check_slam.py
python scripts/check_environment.py
```

This fetches the pinned official sources, builds Pangolin 0.6 into a local
`third_party/install/<OS>-<architecture>/` prefix, builds ORB-SLAM3's bundled
DBoW2/g2o/Sophus dependencies, extracts the vocabulary, and builds `mono_tum`
and `rgbd_tum`. It defaults to two compile jobs for the 16 GB Mac; adjust with
`BUILD_JOBS=4 bash scripts/setup_slam.sh` on a larger machine.

Compatibility changes are tracked in [`patches/ORB_SLAM3/`](../patches/ORB_SLAM3/).
The C++14 patch matches Pangolin 0.6's language requirements. The shutdown patch
waits for mapping/loop-closing workers before exporting optimized poses; patches in
[`patches/Pangolin/`](../patches/Pangolin/) add standard integer headers required
by GCC 13. The script checks
the pinned source revision and applies patches idempotently; it does not reset
existing checkouts. It records the vocabulary SHA-256, local source diff and
environment report under `results/setup/`. No upstream build changes are hidden.

The upstream TUM examples enable the viewer. For offline runs in the container,
use `xvfb-run -a <executable> <arguments>` to provide a virtual display. Software
OpenGL in the Mac's Linux VM is separate from native DA3 Metal inference.

Reference: [official ORB-SLAM3 build instructions](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites).

## Headless monocular baseline

With the existing cached image, pinned sources and TUM data, run from the Mac
repository root:

```bash
docker run --pull=never --rm --network=none \
  --mount "type=bind,source=$PWD,target=/workspace" -e UV_OFFLINE=1 \
  monocular-depth-slam:dev bash scripts/build_monocular_runner.sh
.venv/bin/python scripts/run_monocular.py \
  --run-dir results/orbslam3_mono_fr1_xyz/run_002
.venv/bin/python scripts/evaluate_monocular.py \
  --run-dir results/orbslam3_mono_fr1_xyz/run_002
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/plot_monocular.py \
  --run-dir results/orbslam3_mono_fr1_xyz/run_002
```

The runner refuses to overwrite an existing directory. `run_001` is the preserved
first experiment; select another unused directory for each replay. This runner
disables the viewer and needs no Xvfb display. The cached build/replay commands
disable container networking. Preview download sizes before obtaining missing
sources/packages/assets on a fresh machine.

The [baseline report](research/orbslam3_monocular_baseline.md) documents the
monocular export conversion, complete RGB timeline, calibration, tracking logs,
SE(3)/Sim(3) and RPE evaluation, scale diagnostics and limitations. It ran on
Ubuntu ARM64 on this Mac; rebuild sources and the container for Linux x86_64
rather than copying compiled libraries to the NVIDIA PC. ORB's RGB-only runner
does not need CUDA.

## DA3 depth through ORB-SLAM3 RGB-D

The first depth-assisted experiment is complete. With the existing Metal
environment, cached weights and original matched-frame predictions:

```bash
HF_HUB_OFFLINE=1 .venv-da3-macos/bin/python scripts/cache_da3_missing_rgb.py
.venv/bin/python scripts/prepare_da3_rgbd.py
.venv/bin/python scripts/run_da3_rgbd.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_002
.venv/bin/python scripts/evaluate_monocular.py \
  --run-dir results/orbslam3_da3_rgbd_fr1_xyz/run_002
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/compare_trajectories.py \
  --depth-run results/orbslam3_da3_rgbd_fr1_xyz/run_002 \
  --output results/trajectory_comparison_fr1_xyz_run_002
```

Build with `scripts/build_monocular_runner.sh` inside the cached container as
above; it now builds both replay executables. The six supplemental predictions
extend the 792-frame depth-study cache to all 798 RGB frames in a separate
directory. Conversion writes about 981 MB of local float32 TIFF output, with
no new asset downloads. The RGB-D run uses depth factor 1 for nominal metres,
without a GT-fitted correction. Preserve completed run directories.
See the [protocol](research/da3_rgbd_protocol.md) and
[results/reproduction report](research/da3_orbslam3_rgbd_baseline.md) for exact
units, pixel alignment, parameters, commands, provenance and common-set metrics.

## Sensor-depth control and repeated replays

The completed control uses existing registered sensor PNGs and the existing
DA3 cache on 792 shared RGB inputs. Sensor preparation writes about 973 MB of
local float32 TIFF files, preserving holes and dividing PNG values by 5000.
The common build command above also builds `control_rgbd_tum` for both depth
sources. Then:

```bash
.venv/bin/python scripts/prepare_sensor_rgbd.py
.venv/bin/python scripts/run_sensor_control_study.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/evaluate_sensor_control_study.py
```

The study schedules three processes per method sequentially in balanced order;
it preserves existing attempts and never overwrites a run directory. Its output
is `results/sensor_depth_control_fr1_xyz/`. Scoring/plotting can be regenerated
separately after replay. The [control report](research/sensor_depth_control.md)
documents the shared timestamp set, per-run results, unit checks and limits.
No downloads are needed with the existing assets. A separate study requires
separate configuration/output paths.

## Offline prefix-calibration diagnostic

This optional, explicitly GT-anchored experiment reproduces the previously
frozen 1.1093010159475212× depth factor using the first 158 sensor-matched
frames, then scores later poses. It reuses the compiled control executable:

```bash
.venv/bin/python scripts/prepare_prefix_calibration.py
.venv/bin/python scripts/run_prefix_calibration_study.py
MPLCONFIGDIR="$PWD/.cache/matplotlib-macos" .venv/bin/python scripts/evaluate_prefix_calibration.py
```

Preparation generates about 973 MB of separate local float32 TIFF output;
there is no new inference or asset download. Preserve completed outputs and
frozen contracts. Additional studies require separate paths/configuration;
run only the evaluation command to regenerate current scores/figures without
replaying or rewriting caches. The report distinguishes three new calibrated
replays from the three derived pose-only rescaling evaluations and excludes
prefix poses from all primary alignment/scoring. See the
[protocol](research/prefix_calibration_protocol.md) and
[results](research/da3_prefix_calibration.md) for the temporal and GT-use limits.

## DA3 on the Mac

Run on the native Mac, outside the Linux container:

```bash
bash scripts/setup_da3.sh
source .venv-da3-macos/bin/activate
python scripts/check_da3.py --device mps
python scripts/check_da3.py --device mps --forward
```

The DA3 environment is isolated because upstream requires NumPy below 2.
Apple's Metal backend is named `mps`; `--device cpu` selects CPU explicitly.
The `--forward` check initializes DA3-Small with random weights and processes
a tiny synthetic tensor. It validates execution without downloading a checkpoint
and does **not** validate depth quality or metric scale.

Upstream unconditionally lists `xformers`, whose supported prebuilt wheels target
Linux/Windows CUDA. The tracked
[`platform-xformers.patch`](../patches/Depth-Anything-3/platform-xformers.patch)
restricts that dependency to Linux x86_64 for this project's supported NVIDIA
path. The upstream DA3 attention implementation uses PyTorch's
`scaled_dot_product_attention`; its model code is unchanged by this patch.
Gaussian-splatting extras are not installed.
The runtime also declares `addict` and `scikit-learn`, which the pinned upstream
source imports but omits from its package requirements.

The tracked [`lazy-colmap.patch`](../patches/Depth-Anything-3/lazy-colmap.patch)
loads pycolmap only when COLMAP export is requested. Eagerly importing it with
PyTorch aborts on this Mac because the wheels initialize conflicting OpenMP
runtimes. Depth inference and NPZ export do not need it. COLMAP export and the
broader upstream reconstruction benchmarks remain unvalidated; do not enable
`KMP_DUPLICATE_LIB_OK` to suppress this conflict. Open3D's Mac wheel also needs
the Homebrew `libusb` library installed above.

References: [official DA3 installation](https://github.com/ByteDance-Seed/Depth-Anything-3#-installation),
[PyTorch MPS](https://docs.pytorch.org/docs/stable/notes/mps.html),
[xFormers installation](https://github.com/facebookresearch/xformers#installing-xformers).

## DA3 on the future NVIDIA PC

Target Ubuntu 24.04 x86_64, either directly or under a supported WSL2 NVIDIA
setup. Install a compatible NVIDIA driver and verify `nvidia-smi` first.
Install uv and Git, clone this repository, then:

```bash
bash scripts/setup_environment.sh
bash scripts/setup_da3.sh
source .venv-da3-cuda/bin/activate
python scripts/check_da3.py --device cuda
python scripts/check_da3.py --device cuda --forward
```

The shared DA3 lock selects native PyTorch wheels on Apple Silicon and compatible
PyTorch/torchvision/xFormers CUDA 12.8 wheels from the official PyTorch index on
Linux x86_64. The NVIDIA driver and GPU must support those wheels. The CUDA path
must be validated on that PC; this Mac cannot verify it.

For GPU access from the container, install
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
on the Linux host and add `--gpus all` when starting the same image. The default
VS Code container has no GPU request, so it remains usable on the Mac.

## Checkpoints and data

The environment setup scripts do not download model checkpoints or datasets.
The explicit download command and its preview, the selected DA3 Metric Large
checkpoint, and the TUM loader are documented in
[data and checkpoint setup](data_and_checkpoint.md). That step downloaded the
approved 1.785 GB and verified a pretrained one-frame Metal inference. The later
[full depth comparison](research/da3_depth_comparison.md) evaluates all 792
sensor-matched frames with explicit metre conversion and GT-scale diagnostics.
General metric accuracy and SLAM improvement remain unestablished.

Place TUM RGB-D sequences under `datasets/tum_rgbd/<sequence>/`, starting with
`rgbd_dataset_freiburg1_xyz`. Preserve RGB timestamps, camera intrinsics and TUM
depth scaling metadata. Sources, environments, datasets, weights and generated
outputs are gitignored. Allow additional free disk space for datasets, model
weights and C++ builds.

## Validation and metadata

```bash
python -m pytest
python experiments/synthetic/scale_bias_gauge_demo.py
python scripts/check_environment.py --json
```

Run checks with the intended environment activated. `check_environment.py`
reports package versions, exact source revisions, and whether the vocabulary,
library and executables exist. Package/source presence alone does not validate
SLAM, DA3 accuracy, a TUM sequence, or real-time performance. The separate
[depth comparison](research/da3_depth_comparison.md) and
[monocular trajectory report](research/orbslam3_monocular_baseline.md) document
the completed real-data checks. The
[DA3 RGB-D report](research/da3_orbslam3_rgbd_baseline.md) adds the first
depth-assisted comparison. Controlled real-time performance and broader
accuracy/robustness validation remain pending.
