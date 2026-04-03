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
cd ~/mcu_workspaces/seeker_mcu/src/<sketch_name>
pio run                        # build default env (esp32s3sense)
pio run -e esp32dev            # build for specific board
pio run -e esp32dev -t upload  # flash via serial
pio run -t upload              # flash default env
```

Start the micro-ROS agent on the host before flashing any `test_bridge_*` sketch:

```bash
# Inside the container — runs the micro-ROS WiFi agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
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

## MCU Library Architecture

### Subsystem Pattern

All hardware abstractions follow the same pattern. A subsystem:

1. Inherits from `Subsystem::ThreadedSubsystem` (which wraps FreeRTOS `xTaskCreatePinnedToCore`).
2. Uses a singleton via `static getInstance(const XxxSetup&)`.
3. Implements `init()` (called from Arduino `setup()`), `begin()` (called once from the task before the loop), and `update()` (called in the task loop at the configured interval).
4. Stores its state behind a `Threads::Mutex` (from `hal_thread.h`) and exposes thread-safe getters for other tasks to read.
5. Is configured via an `XxxSetup` struct (inherits `Classes::BaseSetup`).

Start a subsystem with:
```cpp
auto& sub = Subsystem::XxxSubsystem::getInstance(setup);
sub.init();
sub.beginThreadedPinned(stackWords, priority, delayMs, core);
```

### Threading Conventions

| Resource | Core | Priority | Notes |
|---|---|---|---|
| WiFi | 1 | 3 | |
| micro-ROS manager | 1 | 4 | |
| Gyro | 1 | 5 | Semaphore-blocked on interrupt |
| Battery | 1 | 2 | 100 ms sampling |
| LiDAR | **0** | 4 | Needs its own core; drains serial at 1 ms |
| Blink | 1 | 1 | |

Use `Threads::Scope lock(mutex_)` (RAII) for all critical sections. Minimize lock hold time by reading into a local, then updating protected state in a short critical section.

Stack sizes (in words, 1 word = 4 bytes): 2048 for blink; 4096 for WiFi/gyro/battery; 6144 for LiDAR; 8192 for micro-ROS manager.

### Pin Definitions

`mcu_ws/lib/RobotConfig/RobotConfig.h` defines all GPIO pins for both board targets, selected at compile time by `-DENV_ESP32DEV` or `-DENV_ESP32S3SENSE`.

- **ESP32DEV**: standard WROOM-32 pinout
- **ESP32S3SENSE**: Seeed XIAO ESP32S3

`Config::tx`/`Config::rx` are the UART2 pins used for the LiDAR on both targets.

### MicroRosBridge — Adding a New Publisher

`mcu_ws/lib/MicroRosBridge/MicroRosBridge.h/.cpp` is a "God bridge" `IMicroRosParticipant` that owns all ROS publishers. Each publisher is gated by a compile-time flag:

```
-DBRIDGE_ENABLE_HEARTBEAT=1
-DBRIDGE_ENABLE_GYRO=1
-DBRIDGE_ENABLE_BATTERY=1
-DBRIDGE_ENABLE_LIDAR=1
```

To add a new publisher:
1. Add `#ifndef BRIDGE_ENABLE_FOO / #define BRIDGE_ENABLE_FOO 0` in the header.
2. Conditionally include the subsystem header and the ROS message header.
3. Define `FooPublisherState` struct (`pub`, `msg`, `elapsedMillis`). If the message has dynamic arrays (like `LaserScan`), add pre-allocated backing buffers in the struct and wire them in `onCreate()` after calling `__init()` on the msg.
4. Add `FooSubsystem* foo = nullptr;` and topic/interval fields to `MicroRosBridgeSetup`.
5. Add `std::conditional_t<BridgeConfig::kEnableLidar, FooPublisherState, EmptyState> foo_;` to `MicroRosBridge`.
6. Implement `#if BRIDGE_ENABLE_FOO` blocks in onCreate / onDestroy / publishAll.

**Critical**: For any message with dynamic sequences (e.g., `sensor_msgs/LaserScan`), call `__init()` before use (it allocates `header.frame_id`), then replace `.data` pointers with struct-allocated buffers. Before calling `__fini()` in `onDestroy()`, null those pointers first to prevent `free()` of stack/struct memory.

The `MicroRosBridgeSetup` struct holds raw pointers to subsystems — construct it in `setup()` after calling `getInstance()`, not at global-static init time.

### Test Sketch Conventions

Two kinds of test sketches:

- **`test_sub_*`** — Serial-only hardware tests. No micro-ROS, no WiFi. The `platformio.ini` excludes `libs_external/esp32` (the micro-ROS library) and only links `common.lib_base`. Interactive command parser over Serial at 921600 baud.
- **`test_bridge_*`** — Full integration tests with WiFi + micro-ROS agent. Inherits from `esp32_microros_wifi` and enables bridge flags. `test_bridge_all` is the canonical integration test enabling all four publishers.

When writing a new `test_sub_*`, do not include `../../libs_external/esp32` in `lib_extra_dirs` — that pulls in micro-ROS which requires network config macros to be defined.

### LDS LiDAR Library

The LDS library (kaiaai/LDS, pinned in `platformio.ini`) uses plain C function-pointer callbacks with no context parameter. The `LidarSubsystem` bridges them via a `static LidarSubsystem* instance_` pointer set in `init()`. Include `<LDS_LDROBOT_LD14P.h>` directly (not `<lds_all_models.h>`) to avoid compiling all 25+ drivers.

`LidarScanData` is ~8.6 KB. When using it as a local variable in a function called from the Arduino main task (default stack ~8 KB), declare it `static` to keep it in BSS.

The `scan_completed=true` flag in the scan point callback fires on the **first point of the new revolution**, not the last of the completed one.
