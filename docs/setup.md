# Setup

## Development environment

Ubuntu 24.04 or the supplied VS Code Dev Container. The container deliberately avoids the unrelated Unity/Gazebo/MoveIt packages from the source template.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev,eval]'
pytest
```

## ORB-SLAM3

Clone the upstream ORB-SLAM3 repository into `third_party/ORB_SLAM3`, pin a specific commit, and follow its build instructions. Its dependencies, including Pangolin and the vocabulary, must be built/configured separately. Record all revisions and local patches. Do not assume the scaffold Dockerfile builds ORB-SLAM3.

## Depth Anything 3

Install the selected DA3 implementation from its official upstream instructions in a separate environment if its CUDA/PyTorch dependencies conflict with ORB-SLAM3. Pin the repository revision, checkpoint ID, preprocessing, output representation and units. No DA3 inference runner is included yet.

## Datasets

Place downloaded TUM RGB-D sequences under `datasets/tum_rgbd/<sequence>/`. Do not commit dataset files. Preserve RGB timestamps, camera intrinsics and depth scale metadata.

## GPU

Install compatible NVIDIA drivers and NVIDIA Container Toolkit on the host before enabling GPU access. The default dev container is CPU-only; DA3 inference will require a separate GPU-enabled setup.

## Validation

```bash
pytest
python scripts/check_environment.py
```

These tests validate the repository scaffold, **not** SLAM or DA3 functionality.
