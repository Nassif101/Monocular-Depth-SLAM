#!/usr/bin/env bash
# Install the locked development and evaluation dependencies without system pip.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if ! command -v uv >/dev/null 2>&1; then
    echo "Install uv first: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
fi

platform="$(uname -s | tr '[:upper:]' '[:lower:]')"
if [[ "$platform" == linux ]]; then
    export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-$PWD/.venv-linux}"
else
    export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-$PWD/.venv}"
fi
export UV_CACHE_DIR="${UV_CACHE_DIR:-$PWD/.cache/uv-$platform}"
export UV_PYTHON_INSTALL_DIR="${UV_PYTHON_INSTALL_DIR:-$PWD/.cache/python}"
uv sync --locked --python 3.12 --extra dev --extra eval
printf '\nActivate with: source %s/bin/activate\n' "$UV_PROJECT_ENVIRONMENT"
