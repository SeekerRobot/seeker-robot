# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Seeker Robot — a ROS 2 Jazzy robotics project with ESP32 microcontrollers communicating via micro-ROS. The system runs inside Docker and spans two workspaces: a ROS 2 workspace for high-level autonomy and a PlatformIO workspace for MCU firmware.

## Architecture

- **`ros2_ws/`** — ROS 2 colcon workspace. Packages live in `ros2_ws/src/`:
  - `mcu_msgs` — Custom ROS 2 message/service definitions (`.msg`/`.srv` files) shared between ROS 2 nodes and micro-ROS MCUs. Also symlinked into `mcu_ws/extra_packages/` so micro-ROS firmware can use the same interfaces.
  - `test_package` — Minimal C++ ROS 2 node for workflow verification.
- **`mcu_ws/`** — PlatformIO workspace for ESP32 firmware. Uses micro-ROS over serial transport (Jazzy distro). Key subdirectories:
  - `libs_external/esp32/micro_ros_platformio/` — micro-ROS PlatformIO library (pre-vendored).
  - `extra_packages/` — Extra ROS packages (including `mcu_msgs`) needed at micro-ROS build time.
  - `platformio.ini` — Build config. Targets extend `esp32_microros` base; C++17 enforced.
- **`docker/`** — Containerized dev environment:
  - `Dockerfile` — Multi-stage build (`base` → `dev`/`prod`). Base installs micro-ROS agent, PlatformIO, ROS 2 Jazzy. `dev` adds Gazebo Harmonic, RViz, rqt.
  - `Dockerfile.init-bootstrap` — One-shot init container that chowns named volumes and seeds `libs_external` into the `mcu_lib_external` volume.
  - `docker-compose.yml` — Defines `ros2` (main dev container) and `init-bootstrap` services. Build artifacts (build/install/log, .pio, platformio cache) are stored in named Docker volumes to avoid polluting the host.
  - `.env.example` — Copy to `.env`. Set `BUILD_TARGET=dev|prod` and display/network config for your OS.

## Docker / Build Commands

All docker compose commands must be run from the `docker/` directory (or use `-f docker/docker-compose.yml`).

```bash
# Setup: copy env file
cp docker/.env.example docker/.env

# Build and start (force rebuild init-bootstrap)
docker compose -f docker/docker-compose.yml build --no-cache init-bootstrap
docker compose -f docker/docker-compose.yml up init-bootstrap
docker compose -f docker/docker-compose.yml up -d ros2

# Shell into the dev container
docker exec -it docker-ros2-1 bash

# Inside the container — build ROS 2 packages
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Build a single package
colcon build --packages-select mcu_msgs

# Run tests
colcon test
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## MCU Firmware (PlatformIO)

```bash
# Inside the container
cd ~/mcu_workspaces/seeker_mcu
pio run                    # build default env
pio run -t upload          # flash via serial
```

## Volume Layout (inside container)

| Host path | Container path | Notes |
|---|---|---|
| `ros2_ws/src/` | `~/ros2_workspaces/src/seeker_ros/` | Source code (bind mount) |
| `mcu_ws/` | `~/mcu_workspaces/seeker_mcu/` | MCU firmware (bind mount) |
| Named volumes | `~/ros2_workspaces/{build,install,log}` | colcon artifacts |
| Named volume | `~/.platformio` | PlatformIO cache |

## Conventions

- **Commit messages**: Conventional Commits enforced via commitlint (`@commitlint/config-conventional`). Use prefixes like `feat:`, `fix:`, `docs:`, `chore:`, etc.
- **ROS 2 distro**: Jazzy (matches micro-ROS distro setting in `platformio.ini`).
- **`mcu_msgs` is shared**: Any changes to message definitions in `ros2_ws/src/mcu_msgs/` must be rebuilt on both the ROS 2 side (`colcon build --packages-select mcu_msgs`) and the MCU side (`pio run` with the extra_packages symlink).
