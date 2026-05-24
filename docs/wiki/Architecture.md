# Architecture

Seeker Robot is split across two workspaces that talk to each other over **micro-ROS** (XRCE-DDS on WiFi/UDP). High-level autonomy — SLAM, Nav2, mission planning, TTS — lives in the **ROS 2 Jazzy** workspace inside a Docker container on the host. Low-level hardware control — IMU, LiDAR, servos, camera, mic, battery — lives in the **PlatformIO / Arduino** workspace that cross-compiles firmware for an ESP32-S3.

This page explains how those pieces interlock at runtime, how the code is laid out, and where the interesting seams are.

---

## High-level topology

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ESP32-S3 (Sesame V2 PCB)                           │
│                                                                             │
│  BNO085 IMU    ──┐                                                          │
│  LD14P LiDAR   ──┤──► micro-ROS (WiFi UDP) ────────────────────────────┐    │
│  PCA9685 Servo ──┤                                                     │    │
│  Battery ADC   ──┘                                                     │    │
│                                                                        │    │
│  Camera ──┐                                                            │    │
│  PDM Mic  ┴──► Web servers (HTTP)                                      │    │
│                  :80 /stream (MJPEG)                                   │    │
│                  :81 /audio  (PCM)                                     │    │
└────────────────────────────────────────────────────────────────────────┼────┘
                                                                         │
                                  WiFi / LAN                             │
                                                                         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     ROS 2 Jazzy (Docker dev container)                      │
│                                                                             │
│  micro_ros_agent ◄──────────────────────────────────────────────────────────┤
│       │                                                                     │
│       │  Publishes              Subscribes                                  │
│       │  /mcu/imu                /cmd_vel           ◄── Nav2 / teleop       │
│       │  /mcu/scan               /mcu/hexapod_cmd   ◄── mission planner     │
│       │  /mcu/battery_voltage                                               │
│       │  /mcu/heartbeat                                                     │
│       │  /mcu/log                                                           │
│       ▼                                                                     │
│  robot_localization EKF  ──►  /odom (+ TF odom → base_footprint)            │
│       │                                                                     │
│       ├──► SLAM Toolbox  ──►  /map (+ TF map → odom)                        │
│       │                                                                     │
│       └──► robot_state_publisher (URDF) ──► TF base_link → {lidar,cam,imu}  │
│                                                                             │
│  Nav2 (BT navigator, controller, planner, behaviors, smoother)              │
│       ◄── /map, /odom, TF     ──► /cmd_vel                                  │
│                                                                             │
│  seeker_vision  ──► /vision/detections, /object_found, /emotion_detail       │
│       ◄── ESP32 cam proxy or Gazebo /camera/image or local webcam           │
│                                                                             │
│  object_seeker (SeekObject Action Server — WANDER/SEEK/PERFORM_MOVE)        │
│       ◄── /map, /vision/detections   ──► NavigateToPose goals               │
│  ball_searcher (legacy mission planner — frontier + red ball)               │
│       ◄── /map, camera feed   ──► NavigateToPose goals                      │
│                                                                             │
│  seeker_voice (Brain) ──► SeekObject goals ──► object_seeker                │
│       ◄── /audio_transcription (from Whisper)  ──► /audio_tts_input (TTS)   │
│                                                                             │
│  seeker_tts     ──► HTTP :8383 /audio_out ──► ESP32 SpeakerSubsystem         │
│  seeker_display ──► HTTP :8390 /lcd_out   ──► ESP32 OledSubsystem           │
│  seeker_media   ──► HTTP :8383 + :8390    ──► ESP32 speaker + OLED          │
│                                                                             │
│  seeker_web     ──► HTTP :8080  (browser dashboard)                         │
│       WebSocket + REST bridge to all /mcu/* topics + /cmd_vel + TTS/WAV     │
└─────────────────────────────────────────────────────────────────────────────┘
```

Key observations:

- The **micro-ROS agent runs inside the Docker container**, not on the ESP32. The firmware acts as an XRCE-DDS client. This is why you must start `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` **before** the firmware tries to connect.
- **Camera, mic, speaker, and OLED do not go through micro-ROS.** They are too fat for XRCE-DDS to handle on an ESP32-S3. The firmware exposes HTTP endpoints for camera (`/stream` :80) and mic (`/audio` :81), while the speaker and OLED act as HTTP *clients* that pull data from the host: `SpeakerSubsystem` fetches PCM from `:8383/audio_out` (served by `seeker_tts` or `seeker_media`), and `OledSubsystem` fetches framebuffers from `:8390/lcd_out` (served by `seeker_display` or `seeker_media`).
- **TF compensation for hexapod body tilt is implicit.** The BNO085 game rotation vector feeds roll/pitch into EKF, which bakes them into `odom → base_footprint`. SLAM Toolbox then un-tilts LiDAR rays automatically when projecting into the `map` frame — no custom filter node is required.

---

## TF tree

```
map
 └── odom                    (from SLAM Toolbox: map → odom correction)
      └── base_footprint     (from EKF: roll/pitch from BNO085 game rotation vector)
           └── base_link     (fixed, from URDF via robot_state_publisher)
                ├── laser_link   (LiDAR mount position)
                ├── camera_link  (camera mount position)
                └── imu_link     (IMU co-located with LiDAR)
```

The `map → odom` correction is published by SLAM Toolbox at its update rate (~0.2 Hz). `odom → base_footprint` is published by the EKF at 50 Hz and carries the tilt. URDF-fixed joints below `base_link` come from `robot_state_publisher`.

---

## Directory layout

```
seeker-robot/
├── ros2_ws/
│   └── src/
│       ├── mcu_msgs/            # HexapodCmd.msg + other shared .msg/.srv
│       ├── seeker_description/  # URDF (Xacro) + display.launch.py
│       ├── seeker_gazebo/       # sim_*.launch.py + bridge.yaml + worlds/
│       ├── seeker_navigation/   # ball_searcher + EKF/SLAM/Nav2 configs + real_*.launch.py
│       ├── seeker_sim/          # fake_mcu_node
│       ├── seeker_display/      # OLED display nodes + HTTP LCD server
│       ├── seeker_media/        # MP4 player (video → OLED + audio → speaker)
│       ├── seeker_test_cmd_vel/ # Minimal cmd_vel driver for manual/auto drive
│       ├── seeker_tts/          # Fish Audio TTS node + tts.launch.py
│       ├── seeker_vision/      # YOLO detection + emotion + camera proxy
│       ├── seeker_voice/       # "Brain" — voice command → SeekObject action client
│       ├── seeker_web/         # Browser-based robot controller (WebSocket + REST on :8080)
│       └── test_package/        # Tiny CI-sanity C++ node
├── mcu_ws/
│   ├── platformio/
│   │   ├── platformio.ini               # Shared base config (board envs, lib_base)
│   │   ├── network_config.ini           # WiFi creds + agent IP (gitignored)
│   │   └── network_config.example.ini   # Template
│   ├── src/                             # 31 per-sketch PlatformIO projects
│   ├── lib/                             # Shared subsystems (see below)
│   ├── libs_external/esp32/             # Pre-vendored micro-ROS PlatformIO lib
│   └── platformio/extra_packages/mcu_msgs/ # Bind-mounted from ros2_ws/src/mcu_msgs
└── docker/
    ├── Dockerfile                       # Multi-stage base → dev/prod
    ├── Dockerfile.init-bootstrap        # Volume chown + libs_external seed
    ├── docker-compose.yml               # ros2 (cpu profile) + ros2-nvidia + ros2-amd + init-bootstrap
    └── .env.example                     # Template for COMPOSE_PROJECT_NAME, BUILD_TARGET, COMPOSE_PROFILES, display/network
```

---

## Docker volumes and bind mounts

| Host path | Container path | Kind | Purpose |
|---|---|---|---|
| `ros2_ws/src/` | `~/ros2_workspaces/src/seeker_ros/` | bind (delegated) | Edit ROS 2 code from the host |
| `ros2_ws/.vscode/` | `~/ros2_workspaces/.vscode/` | bind (delegated) | Shared VS Code settings |
| `mcu_ws/` | `~/mcu_workspaces/seeker_mcu/` | bind (delegated) | Edit firmware from the host |
| `mcu_ws/platformio/network_config.ini` | `~/mcu_workspaces/seeker_mcu/platformio/network_config.ini` | bind (via parent) | Secret-ish WiFi creds (part of the `mcu_ws/` mount) |
| `ros2_ws/src/mcu_msgs/` | `~/mcu_workspaces/seeker_mcu/platformio/extra_packages/mcu_msgs/` | bind (delegated) | Makes `mcu_msgs` visible to micro-ROS codegen |
| `scripts/` | `~/scripts/` | bind | Utility scripts |
| `/dev`, `/sys`, `/tmp/.X11-unix` | same | bind | USB, GPIO, X server |
| (named) `ros2_build` | `~/ros2_workspaces/build` | volume | colcon build artifacts |
| (named) `ros2_install` | `~/ros2_workspaces/install` | volume | colcon install artifacts |
| (named) `ros2_log` | `~/ros2_workspaces/log` | volume | colcon logs |
| (named) `platformio_cache` | `~/.platformio` | volume | PlatformIO tool/lib cache |
| (named) `mcu_lib_external` | `~/mcu_workspaces/seeker_mcu/libs_external` | volume | Seeded micro-ROS PlatformIO library |

The named volumes keep heavy artifacts off the host filesystem (avoiding root-owned files and speeding up IO on macOS/Windows). `init-bootstrap` chowns all of them to UID 1000 (`ubuntu`) and seeds `mcu_lib_external` from `mcu_ws/libs_external` once.

---

## FreeRTOS task pinning (firmware)

`ThreadedSubsystem` (in `mcu_ws/lib/ThreadedSubsystem/`) is the base class for every hardware subsystem. Call `beginThreadedPinned(stackWords, priority, updateDelayMs, core)` to spawn a pinned FreeRTOS task that calls `begin()` once then `update()` on a fixed cadence.

Conventions used across the codebase:

| Core | Subsystem | Cadence | Notes |
|---|---|---|---|
| 0 | `LidarSubsystem` | 1 ms | Time-critical UART drain; must never block |
| 1 | `ESP32WifiSubsystem` | 100 ms | Reconnection state machine |
| 1 | `GyroSubsystem` | 0 ms | Semaphore-blocked on BNO085 interrupt |
| 1 | `BatterySubsystem` | 50 ms | ADC sampling + EMA filter |
| 1 | `MicrorosManager` | 10 ms | Agent ping + executor spin + `publishAll()` |

All subsystems that share a resource (I²C bus, the micro-ROS transport, published data buffers) use `Threads::Mutex` / `Threads::Scope` from `hal_thread.h`.

---

## The `MicroRosBridge` compile-time plugin pattern

A single `MicroRosBridge` instance (see `mcu_ws/lib/MicroRosBridge/`) owns every ROS publisher the robot exposes. Non-ROS-aware subsystems provide thread-safe getters; the bridge reads them and publishes at configured rates.

```
GyroSubsystem ─────┐
BatterySubsystem ──┤
LidarSubsystem ────┼──► MicroRosBridge ──► micro-ROS (WiFi) ──► Agent ──► ROS 2
HeartbeatCounter ──┘
```

Each publisher is **gated by a preprocessor flag** so disabled subsystems cost zero RAM:

| Flag | Publishes |
|---|---|
| `BRIDGE_ENABLE_HEARTBEAT=1` | `/mcu/heartbeat` (`std_msgs/Int32`, 1 Hz) |
| `BRIDGE_ENABLE_GYRO=1` | `/mcu/imu` (`sensor_msgs/Imu`, 50 Hz) |
| `BRIDGE_ENABLE_BATTERY=1` | `/mcu/battery_voltage` (`std_msgs/Float32`, 1 Hz) |
| `BRIDGE_ENABLE_LIDAR=1` | `/mcu/scan` (`sensor_msgs/LaserScan`, ~6 Hz, capped at 20 Hz) |
| `BRIDGE_ENABLE_DEBUG=1` | `/mcu/log` (`std_msgs/String`, event-driven) |
| `BRIDGE_ENABLE_SERVO=1` | Reserved (servo telemetry — currently stubbed) |

> **OLED display** is **not** part of the bridge. `OledSubsystem` runs its own HTTP client that fetches 1024-byte SSD1306 framebuffers from the ROS 2 host at `GET /lcd_out` (port 8390). The host-side server is provided by `seeker_display` (demo sine wave) or `seeker_media` (MP4 playback). No micro-ROS agent is required for the OLED.

When a flag is `0`, the corresponding state struct inside `MicroRosBridge` is a no-op placeholder via `std::conditional_t<..., FooPublisherState, EmptyState>` — the publisher is completely absent from the binary.

**Adding a new publisher to the bridge** (from `CLAUDE.md`):

1. Add `#ifndef BRIDGE_ENABLE_FOO / #define BRIDGE_ENABLE_FOO 0` in `MicroRosBridge.h`.
2. Conditionally include the subsystem header and define `FooPublisherState`.
3. Add `kEnableFoo` to `BridgeConfig` and the corresponding fields to `MicroRosBridgeSetup`.
4. Add `std::conditional_t<..., FooPublisherState, EmptyState> foo_` member.
5. Add `#if BRIDGE_ENABLE_FOO` blocks in the `.cpp`: `onCreate`, `onDestroy`, `publishAll`.
6. Enable via `-DBRIDGE_ENABLE_FOO=1` in the relevant sketch's `platformio.ini`.

---

## `MicrorosManager` reconnection state machine

`MicrorosManager` (in `mcu_ws/lib/Microros/`) owns the XRCE-DDS session and runs a 4-state machine in its `update()`:

```
WAITING_AGENT ──► AGENT_AVAILABLE ──► AGENT_CONNECTED ──► AGENT_DISCONNECTED
      ▲                                                           │
      └───────────────────────────────────────────────────────────┘
```

- `WAITING_AGENT` — ping the agent every second.
- `AGENT_AVAILABLE` — agent replied; create the RCL support, node, and call `onCreate()` on every registered `IMicroRosParticipant`.
- `AGENT_CONNECTED` — spin the executor, call `publishAll()` on every participant.
- `AGENT_DISCONNECTED` — session lost; destroy entities via `onDestroy()` and bounce back to `WAITING_AGENT`.

`IMicroRosParticipant` is the interface any ROS-aware component implements:

```cpp
bool onCreate(MicroRosContext& ctx);  // create publishers/subscribers; return false to abort
void onDestroy();                     // zero-init handles (session already torn down)
void publishAll();                    // called under the transport mutex; must be non-blocking
```

`MicroRosContext` exposes `createPublisherBestEffort()`, `createPublisherReliable()`, and subscription variants. Register participants **before** calling `manager.init()` in your sketch's `setup()`.

---

## How `mcu_msgs` flows between workspaces

`mcu_msgs` lives in `ros2_ws/src/mcu_msgs/` as a standard `ament_cmake` package with `.msg`/`.srv` files. Docker Compose bind-mounts the same directory into the MCU workspace at `~/mcu_workspaces/seeker_mcu/platformio/extra_packages/mcu_msgs/`, which is where the micro-ROS build tooling looks for extra packages when generating the firmware's typesupport.

When you edit a message:

```bash
# 1. Rebuild ROS 2 side
cd ~/ros2_workspaces
colcon build --packages-select mcu_msgs
source install/setup.bash

# 2. Rebuild firmware (clean picks up new extras)
cd ~/mcu_workspaces/seeker_mcu/src/test_bridge_all
pio run --target clean && pio run -e esp32s3sense -t upload
```

Both sides end up with identical wire-format type support, which is what lets a `mcu_msgs/HexapodCmd` flow end-to-end.

---

## Where to go next

- **[ROS2 Packages](ROS2-Packages.md)** — per-package reference for the ROS 2 side.
- **[MCU Firmware](MCU-Firmware.md)** — build/flash/debug workflow for the firmware side.
- **[Simulation](Simulation.md)** — run the whole stack without hardware.
- **[IRL Tests](IRL-Tests.md)** — hardware bring-up playbook.
