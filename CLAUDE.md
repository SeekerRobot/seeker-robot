# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Seeker Robot — a ROS 2 Jazzy robotics project with ESP32 microcontrollers communicating via micro-ROS. The system runs inside Docker and spans two workspaces: a ROS 2 workspace for high-level autonomy and a PlatformIO workspace for MCU firmware.

## Architecture

- **`ros2_ws/`** — ROS 2 colcon workspace. Packages live in `ros2_ws/src/`:
  - `mcu_msgs` — Custom ROS 2 message/service definitions (`.msg`/`.srv` files) shared between ROS 2 nodes and micro-ROS MCUs. Also mounted into `mcu_ws/platformio/extra_packages/` by Docker so micro-ROS firmware can use the same interfaces.
  - `seeker_description` — URDF/Xacro hexapod model and `robot_state_publisher` launch.
  - `seeker_gazebo` — Gazebo Harmonic simulation, sensor bridges, and simulation launch files.
  - `seeker_navigation` — Nav2, SLAM Toolbox, EKF configs, and `ball_searcher` mission planner.
  - `seeker_sim` — `fake_mcu_node`: simulates ESP32 gait for testing without hardware.
  - `seeker_display` — OLED display nodes: `oled_sine_node` (animated sine wave demo) and `lcd_http_server` (shared helper that serves SSD1306 framebuffers over HTTP on port 8390 for the ESP32 `OledSubsystem`).
  - `seeker_media` — MP4 media player node (`mp4_player_node`): decodes video to 128×64 SSD1306 framebuffers streamed over HTTP and audio to 16 kHz PCM streamed to the ESP32 speaker, with A/V sync.
  - `seeker_tts` — Fish Audio TTS node plus a local-WAV playback topic, both re-served as an HTTP PCM stream for the ESP32 `SpeakerSubsystem`.
  - `seeker_vision` — YOLO object detection (`vision_node`, `gazebo_vision_node`), DeepFace emotion detection (`emotion_node`), and an MJPEG camera proxy (`cam_proxy`) that bridges the ESP32 camera stream to localhost. Three launch files: `mcu_cam.launch.py` (ESP32 camera via proxy), `gazebo_cam.launch.py` (Gazebo `/camera/image`), `local_cam.launch.py` (host webcam).
  - `seeker_web` — Browser-based robot controller (`web_node`). Serves an HTML dashboard on port 8080 with WebSocket and REST bridges to ROS 2 topics. Provides a virtual joystick, real-time IMU/LiDAR visualization, MJPEG camera feed, mic audio playback, TTS input, WAV playback, and live log tailing. Launch: `web.launch.py`.
  - `test_package` — Minimal C++ ROS 2 node for workflow verification.
- **`mcu_ws/`** — PlatformIO workspace for ESP32 firmware. Uses micro-ROS WiFi transport (Jazzy distro). Multi-project layout:
  - `platformio/platformio.ini` — Shared base config (board environments, build flags, library deps). All sketches inherit from this via `extra_configs`.
  - `platformio/network_config.ini` — Local network settings (WiFi creds, agent IP, static IP). Gitignored; copy from `network_config.example.ini`.
  - `src/<sketch>/` — Each sketch is a standalone PlatformIO project with its own `platformio.ini` and `src/` directory.
  - `lib/` — Shared libraries available to all sketches via `lib_extra_dirs`.
  - `libs_external/esp32/micro_ros_platformio/` — micro-ROS PlatformIO library (pre-vendored).
  - `platformio/extra_packages/` — Extra ROS packages (including `mcu_msgs`) needed at micro-ROS build time.
- **`docker/`** — Containerized dev environment:
  - `Dockerfile` — Multi-stage build (`base` → `dev`/`prod`). Base installs micro-ROS agent, PlatformIO, ROS 2 Jazzy, and vision dependencies (ultralytics, deepface, tensorflow+CUDA). `dev` adds Gazebo Harmonic, RViz, rqt.
  - `Dockerfile.init-bootstrap` — One-shot init container that chowns named volumes and seeds `libs_external` into the `mcu_lib_external` volume.
  - `docker-compose.yml` — Defines `ros2` (main dev container, `cpu` profile), `init-bootstrap` (one-shot volume init), and GPU-enabled profile services `ros2-nvidia` (`nvidia` profile) and `ros2-amd` (`amd` profile). Set `COMPOSE_PROFILES=cpu` (or `nvidia`/`amd`) in `.env` so all `docker compose` commands pick up the correct service. `COMPOSE_PROJECT_NAME` in `.env` isolates containers/volumes per worktree.
  - `.env.example` — Copy to `.env`. Set `COMPOSE_PROJECT_NAME`, `BUILD_TARGET=dev|prod`, `COMPOSE_PROFILES=cpu|nvidia|amd`, and display/network config for your OS.

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
cd ~/mcu_workspaces/seeker_mcu/src/<sketch>
pio run                        # build default env (esp32s3sense)
pio run -e esp32dev            # build for specific board
pio run -e esp32dev -t upload  # flash via serial
```

## Micro-ROS Agent

The agent runs inside the dev container alongside the firmware. Start it before flashing:

```bash
# WiFi (UDP) transport — matches board_microros_transport = wifi in platformio.ini
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Serial transport — matches board_microros_transport = serial
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Volume Layout (inside container)

| Host path | Container path | Notes |
|---|---|---|
| `ros2_ws/src/` | `~/ros2_workspaces/src/seeker_ros/` | Source code (bind mount) |
| `mcu_ws/` | `~/mcu_workspaces/seeker_mcu/` | MCU firmware (bind mount) |
| `ros2_ws/src/mcu_msgs/` | `~/mcu_workspaces/seeker_mcu/platformio/extra_packages/mcu_msgs/` | Bind mount for micro-ROS build |
| `mcu_ws/platformio/network_config.ini` | `~/mcu_workspaces/seeker_mcu/platformio/network_config.ini` | Network config (via parent mcu_ws mount) |
| `scripts/` | `~/scripts/` | Utility scripts (bind mount) |
| Named volumes | `~/ros2_workspaces/{build,install,log}` | colcon artifacts |
| Named volume | `~/.platformio` | PlatformIO cache |
| Named volume | `~/mcu_workspaces/seeker_mcu/libs_external` | Seeded micro-ROS lib |

## MCU Library Architecture

### ThreadedSubsystem (`mcu_ws/lib/ThreadedSubsystem/`)
Base class for all hardware subsystems. Call `beginThreadedPinned(stackWords, priority, updateDelayMs, core)` to spawn a pinned FreeRTOS task. The task calls `begin()` once, then repeatedly calls `update()` with the specified delay.

Task pinning conventions used across the codebase:
- Core 0: LiDAR (time-critical serial drain, 1 ms cadence)
- Core 1: WiFi (100 ms), Gyro (0 ms, semaphore-blocked on interrupt), Battery (50 ms), micro-ROS manager (10 ms)
- All subsystems use `Threads::Mutex` / `Threads::Scope` (from `hal_thread.h`) for shared-resource protection (e.g., I2C bus, published data buffers).

### MicrorosManager + IMicroRosParticipant (`mcu_ws/lib/Microros/`)
The manager runs a 4-state reconnection machine: `WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED`. In `update()` it pings the agent, spins the XRCE-DDS executor, and calls `publishAll()` on all registered participants.

`IMicroRosParticipant` is the interface for anything that owns ROS publishers/subscribers:
- `onCreate(MicroRosContext& ctx)` — create RCL entities (publishers, subscribers); return false to abort
- `onDestroy()` — zero-init publishers (RCL session already torn down by manager before this fires)
- `publishAll()` — called in the manager's loop under a transport mutex; must be non-blocking

`MicroRosContext` exposes `createPublisherBestEffort()`, `createPublisherReliable()`, and subscription variants. Register participants before calling `manager.init()`.

### MicroRosBridge — compile-time plugin pattern (`mcu_ws/lib/MicroRosBridge/`)
`MicroRosBridge` implements `IMicroRosParticipant` and is the sole owner of all hardware publishers. Non-ROS-aware subsystems (gyro, battery, lidar) expose thread-safe getters; the bridge reads them and publishes at configured rates.

Each publisher is gated by a preprocessor flag (default 0): `BRIDGE_ENABLE_HEARTBEAT`, `BRIDGE_ENABLE_GYRO`, `BRIDGE_ENABLE_BATTERY`, `BRIDGE_ENABLE_SERVO`, `BRIDGE_ENABLE_LIDAR`, `BRIDGE_ENABLE_DEBUG`. Disabled publishers cost zero RAM — the state struct becomes an `EmptyState` placeholder via `std::conditional_t`. The OLED display is **not** part of the bridge — `OledSubsystem` runs its own HTTP client that fetches 1024-byte SSD1306 framebuffers from the ROS 2 host at `GET /lcd_out` (port 8390, served by `seeker_display` or `seeker_media`). To add a new subsystem publisher:

1. Add `#ifndef BRIDGE_ENABLE_FOO / #define BRIDGE_ENABLE_FOO 0` in `MicroRosBridge.h`
2. Conditionally include the subsystem header and define `FooPublisherState`
3. Add `kEnableFoo` to `BridgeConfig` and fields to `MicroRosBridgeSetup`
4. Add `std::conditional_t<..., FooPublisherState, EmptyState> foo_` member
5. Add `#if BRIDGE_ENABLE_FOO` blocks in `.cpp`: `onCreate`, `onDestroy`, `publishAll`
6. Enable via `-DBRIDGE_ENABLE_FOO=1` in the sketch's `platformio.ini`

### Sketch naming convention
- `test_sub_*` — tests a single subsystem in isolation (serial only, no micro-ROS)
- `test_bridge_*` — exercises the full micro-ROS stack via WiFi transport
- `test_raw_*` — low-level hardware tests with no subsystem abstraction (e.g., raw camera I2C, raw PDM mic)
- `test_all` — integration test for all subsystems together
- `test_threaded_blink` — ThreadedSubsystem / FreeRTOS task smoke test
- `build_microros` — placeholder sketch used only to pre-build the micro-ROS library
- `main` — full integration firmware. Default env `esp32s3sense_offload` runs all subsystems except camera/mic (offloaded to `main_satellite`). Also has `esp32s3sense_main` (all-in-one) and `esp32dev` variants.
- `main_add` — incremental modular rebuild of `main` (phases 1–6). All subsystems enabled, used for staged bring-up.
- `main_satellite` — camera/sensor offload board for dual-board architecture. Default env `esp32cam_satellite` (AI-Thinker ESP32-CAM); also supports `esp32s3sense_satellite`.

## Conventions

- **Commit messages**: Conventional Commits enforced via commitlint (`@commitlint/config-conventional`). Use prefixes like `feat:`, `fix:`, `docs:`, `chore:`, etc.
- **ROS 2 distro**: Jazzy (matches micro-ROS distro setting in `platformio.ini`).
- **`mcu_msgs` is shared**: Any changes to message definitions in `ros2_ws/src/mcu_msgs/` must be rebuilt on both the ROS 2 side (`colcon build --packages-select mcu_msgs`) and the MCU side (`pio run` — the bind-mount at `platformio/extra_packages/mcu_msgs` picks up changes automatically).
- **Board environments**: `esp32s3sense` (Seeed XIAO ESP32-S3, default), `esp32dev` (generic ESP32-WROOM-32), and `esp32cam` (AI-Thinker ESP32-CAM). Pin definitions are in `mcu_ws/lib/RobotConfig/RobotConfig.h`, gated by `ENV_ESP32S3SENSE` / `ENV_ESP32DEV` / `ENV_ESP32CAM` macros set by the board's build flags.
- **`test_sub_*` sketches** exclude `libs_external/esp32` from `lib_extra_dirs` to avoid pulling in micro-ROS; they set their own minimal `platformio.ini` env blocks with only `${common.lib_base}` and `../../lib/`.
