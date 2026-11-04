# MA Robot Sim devcontainer

This devcontainer targets:

- Ubuntu 24.04 base through ROS Jazzy images
- ROS 2 Jazzy
- Gazebo Harmonic through `ros-jazzy-ros-gz`
- Nav2
- MoveIt 2
- RViz2
- optional noVNC browser desktop
- optional host GUI forwarding
- optional NVIDIA GPU forwarding

## Default VS Code mode

Open the repository in VS Code and run:

```text
Dev Containers: Reopen in Container
```

The default compose file is portable and should work on Ubuntu Docker and Windows 11 with Docker Desktop/WSL2. It mounts the repository at:

```text
/mt-mohamad-nassif
```

GUI applications are available through noVNC by default, so WSLg and NVIDIA support are optional overlays.

## Build workspace

```bash
cb
srcws
```

`cb` runs:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${ROS2_WS_CMAKE_BUILD_TYPE:-RelWithDebInfo}"
```

Set the low-memory build args in `docker-compose.yml` if Docker Desktop runs out of RAM during large ROS builds.

## noVNC browser desktop

Inside the container:

```bash
novnc
```

Then open:

```text
http://localhost:6080/vnc.html
```

You can run GUI apps inside that desktop:

```bash
rviz2
```

## WSLg GUI forwarding

For direct WSLg windows instead of noVNC, edit `devcontainer.json` and use:

```jsonc
"dockerComposeFile": [
  "docker-compose.yml",
  "docker-compose.wslg.yml"
]
```

Then test:

```bash
glxinfo -B
```

## NVIDIA GPU forwarding

First verify on the host:

```bash
docker run --rm --gpus all nvidia/cuda:12.5.1-base-ubuntu24.04 nvidia-smi
```

Then add:

```jsonc
"dockerComposeFile": [
  "docker-compose.yml",
  "docker-compose.nvidia.yml"
]
```

or for WSL2:

```jsonc
"dockerComposeFile": [
  "docker-compose.yml",
  "docker-compose.wslg.yml",
  "docker-compose.nvidia.yml"
]
```

Check rendering:

```bash
glxinfo -B
```
