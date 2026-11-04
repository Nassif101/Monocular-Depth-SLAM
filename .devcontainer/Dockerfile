FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git curl sudo build-essential cmake ninja-build pkg-config \
    python3 python3-pip python3-venv python3-dev \
    libopencv-dev libeigen3-dev libglew-dev libgl1-mesa-dev \
    libboost-serialization-dev libssl-dev && \
    rm -rf /var/lib/apt/lists/*
RUN useradd -m -s /bin/bash vscode && echo 'vscode ALL=(ALL) NOPASSWD:ALL' >/etc/sudoers.d/vscode
# Provide an X display for Pangolin/OpenCV when running offline baselines headlessly.
RUN apt-get update && apt-get install -y --no-install-recommends xvfb xauth && \
    rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:0.12.19 /uv /usr/local/bin/uv
ENV UV_PROJECT_ENVIRONMENT=/workspace/.venv-linux \
    UV_PYTHON_DOWNLOADS=never \
    PATH="/workspace/.venv-linux/bin:${PATH}"
USER vscode
WORKDIR /workspace
