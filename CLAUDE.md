# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Seeker Robot — a ROS 2 Jazzy robotics project with ESP32 microcontrollers communicating via micro-ROS. The system runs inside Docker and spans two workspaces: a ROS 2 workspace for high-level autonomy and a PlatformIO workspace for MCU firmware.

## Architecture

- **`ros2_ws/`** — ROS 2 colcon workspace. Packages live in `ros2_ws/src/`:
  - `mcu_msgs` — Custom ROS 2 message/service definitions (`.msg`/`.srv` files) shared between ROS 2 nodes and micro-ROS MCUs. Also mounted into `mcu_ws/extra_packages/` by Docker so micro-ROS firmware can use the same interfaces.
  - `test_package` — Minimal C++ ROS 2 node for workflow verification.
- **`mcu_ws/`** — PlatformIO workspace for ESP32 firmware. Uses micro-ROS WiFi transport (Jazzy distro). Multi-project layout:
  - `platformio/platformio.ini` — Shared base config (board environments, build flags, library deps). All sketches inherit from this via `extra_configs`.
  - `platformio/network_config.ini` — Local network settings (WiFi creds, agent IP, static IP). Gitignored; copy from `network_config.example.ini`.
  - `src/<sketch>/` — Each sketch is a standalone PlatformIO project with its own `platformio.ini` and `src/` directory.
  - `lib/` — Shared libraries available to all sketches via `lib_extra_dirs`.
  - `libs_external/esp32/micro_ros_platformio/` — micro-ROS PlatformIO library (pre-vendored).
  - `extra_packages/` — Extra ROS packages (including `mcu_msgs`) needed at micro-ROS build time.
- **`docker/`** — Containerized dev environment:
  - `Dockerfile` — Multi-stage build (`base` → `dev`/`prod`). Base installs micro-ROS agent, PlatformIO, ROS 2 Jazzy. `dev` adds Gazebo Harmonic, RViz, rqt.
  - `Dockerfile.init-bootstrap` — One-shot init container that chowns named volumes and seeds `libs_external` into the `mcu_lib_external` volume.
  - `docker-compose.yml` — Defines `ros2` (main dev container) and `init-bootstrap` services. `COMPOSE_PROJECT_NAME` in `.env` isolates containers/volumes per worktree.
  - `.env.example` — Copy to `.env`. Set `COMPOSE_PROJECT_NAME`, `BUILD_TARGET=dev|prod`, and display/network config for your OS.

## Docker / Build Commands

All docker compose commands must be run from the `docker/` directory (or use `-f docker/docker-compose.yml`).

```bash
# Setup: copy env files
cp docker/.env.example docker/.env
cp mcu_ws/platformio/network_config.example.ini mcu_ws/platformio/network_config.ini

# Build and start (force rebuild init-bootstrap)
docker compose -f docker/docker-compose.yml build --no-cache init-bootstrap
docker compose -f docker/docker-compose.yml up init-bootstrap
docker compose -f docker/docker-compose.yml up -d ros2

# Shell into the dev container (container name uses COMPOSE_PROJECT_NAME from .env)
docker compose -f docker/docker-compose.yml exec ros2 bash

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

Each sketch under `mcu_ws/src/` is its own PlatformIO project. Build from within the sketch directory:

```bash
# Inside the container
cd ~/mcu_workspaces/seeker_mcu/src/main
pio run                        # build default board env
pio run -e esp32dev            # build for specific board
pio run -e esp32dev -t upload  # flash via serial
```

## Volume Layout (inside container)

| Host path | Container path | Notes |
|---|---|---|
| `ros2_ws/src/` | `~/ros2_workspaces/src/seeker_ros/` | Source code (bind mount) |
| `mcu_ws/` | `~/mcu_workspaces/seeker_mcu/` | MCU firmware (bind mount) |
| Named volumes | `~/ros2_workspaces/{build,install,log}` | colcon artifacts |
| Named volume | `~/.platformio` | PlatformIO cache |
| Named volume | `~/mcu_workspaces/seeker_mcu/libs_external` | Seeded micro-ROS lib |

## Conventions

- **Commit messages**: Conventional Commits enforced via commitlint (`@commitlint/config-conventional`). Use prefixes like `feat:`, `fix:`, `docs:`, `chore:`, etc.
- **ROS 2 distro**: Jazzy (matches micro-ROS distro setting in `platformio.ini`).
- **`mcu_msgs` is shared**: Any changes to message definitions in `ros2_ws/src/mcu_msgs/` must be rebuilt on both the ROS 2 side (`colcon build --packages-select mcu_msgs`) and the MCU side (`pio run` with the extra_packages symlink).
