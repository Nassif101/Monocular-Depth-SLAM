#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
bash scripts/setup_slam.sh
cmake -S cpp -B build/monocular_runner -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PWD/third_party/install/$(uname -s)-$(uname -m)"
cmake --build build/monocular_runner --parallel "${BUILD_JOBS:-2}"
