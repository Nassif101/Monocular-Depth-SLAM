#!/usr/bin/env bash
# Build the pinned SLAM dependencies in Ubuntu, including on Apple Silicon Docker.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "$(uname -s)" != Linux ]]; then
    echo "Run this script in the Ubuntu dev container; see docs/setup.md." >&2
    exit 1
fi
jobs="${BUILD_JOBS:-2}"
prefix="$PWD/third_party/install/$(uname -s)-$(uname -m)"
python3 scripts/fetch_dependencies.py ORB_SLAM3 Pangolin

for project in ORB_SLAM3 Pangolin; do
    for patch in "$PWD"/patches/"$project"/*.patch; do
        if ! git -C "third_party/$project" apply --reverse --check "$patch" 2>/dev/null; then
            git -C "third_party/$project" apply --check "$patch"
            git -C "third_party/$project" apply "$patch"
        fi
    done
done

cmake -S third_party/Pangolin -B third_party/Pangolin/build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF
cmake --build third_party/Pangolin/build --parallel "$jobs"
cmake --install third_party/Pangolin/build

for dependency in DBoW2 g2o Sophus; do
    source_dir="third_party/ORB_SLAM3/Thirdparty/$dependency"
    options=()
    if [[ "$dependency" == Sophus ]]; then
        options=(-DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF)
    fi
    cmake -S "$source_dir" -B "$source_dir/build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release "${options[@]}"
    cmake --build "$source_dir/build" --parallel "$jobs"
done
if [[ ! -f third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt ]]; then
    tar -xf third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz \
        -C third_party/ORB_SLAM3/Vocabulary
fi
cmake -S third_party/ORB_SLAM3 -B third_party/ORB_SLAM3/build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix"
cmake --build third_party/ORB_SLAM3/build --target mono_tum rgbd_tum --parallel "$jobs"
mkdir -p results/setup
sha256sum third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt > results/setup/vocabulary.sha256
git -C third_party/ORB_SLAM3 diff > results/setup/orbslam3-local.patch
git -C third_party/Pangolin diff > results/setup/pangolin-local.patch
sha256sum patches/*/*.patch > results/setup/patches.sha256
python3 scripts/check_environment.py --json > results/setup/environment-linux.json
echo "Built ORB-SLAM3 mono_tum and rgbd_tum. No dataset baseline has been run."
