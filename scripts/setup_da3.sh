#!/usr/bin/env bash
# Separate native Metal and Linux CUDA runtimes; never install CUDA in the Mac VM.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
case "$(uname -s)-$(uname -m)" in
    Darwin-arm64) runtime=macos ;;
    Linux-x86_64)
        if ! command -v nvidia-smi >/dev/null 2>&1; then
            echo "The Linux DA3 environment requires an NVIDIA driver (nvidia-smi)." >&2
            exit 1
        fi
        nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
        runtime=cuda
        ;;
    *) echo "DA3 setup supports Apple Silicon macOS or NVIDIA Linux x86_64." >&2; exit 1 ;;
esac
export UV_PROJECT_ENVIRONMENT="${DA3_ENVIRONMENT:-$PWD/.venv-da3-$runtime}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$PWD/.cache/uv-$(uname -s | tr '[:upper:]' '[:lower:]')}"
export UV_PYTHON_INSTALL_DIR="${UV_PYTHON_INSTALL_DIR:-$PWD/.cache/python}"
python3 scripts/fetch_dependencies.py Depth-Anything-3
for patch in "$PWD"/patches/Depth-Anything-3/*.patch; do
    if ! git -C third_party/Depth-Anything-3 apply --reverse --check "$patch" 2>/dev/null; then
        git -C third_party/Depth-Anything-3 apply --check "$patch"
        git -C third_party/Depth-Anything-3 apply "$patch"
    fi
done
uv sync --project environments/da3 --locked --python 3.12
mkdir -p results/setup
git -C third_party/Depth-Anything-3 diff > results/setup/da3-local.patch
"$UV_PROJECT_ENVIRONMENT/bin/python" scripts/check_environment.py --json \
    > "results/setup/environment-da3-$runtime.json"
printf '\nActivate with: source %s/bin/activate\n' "$UV_PROJECT_ENVIRONMENT"
echo "Model checkpoints and datasets are separate downloads."
