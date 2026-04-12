# MCU Firmware

The ESP32 firmware lives in `mcu_ws/`, a multi-project PlatformIO workspace. Every sketch under `mcu_ws/src/` is an independent PlatformIO project that inherits board environments, library dependencies, and build flags from the shared `mcu_ws/platformio/platformio.ini`. The `mcu_ws/` tree is bind-mounted into the dev container at `~/mcu_workspaces/seeker_mcu/`, so editing firmware from the host and building inside the container just works.

See **[Architecture](Architecture.md)** for how this firmware talks to ROS 2 via micro-ROS.

---

## Board environments

All envs are defined in `mcu_ws/platformio/platformio.ini`.

| Env | Board | Transport | Partition | Notes |
|---|---|---|---|---|
| `esp32s3sense` *(default)* | Seeed XIAO ESP32-S3 Sense | WiFi UDP | `min_spiffs.csv` | Primary target. Camera + PSRAM. `BRIDGE_ENABLE_HEARTBEAT/GYRO/BATTERY/LIDAR/DEBUG=1`, `ENABLE_ARDUINO_OTA=1`. |
| `esp32dev` | Generic ESP32-WROOM-32 | WiFi UDP | `min_spiffs.csv` | No camera. Same `BRIDGE_ENABLE_*` set as the S3 Sense (heartbeat/gyro/battery/lidar/debug) plus `ENABLE_ARDUINO_OTA=1`. |
| `esp32s3senseserial` | Seeed XIAO ESP32-S3 Sense | Serial | default | Serial micro-ROS transport (for the serial agent). |
| `esp32devserial` | Generic ESP32-WROOM-32 | Serial | default | Serial micro-ROS transport. |
| `esp32cam` | AI-Thinker ESP32-CAM | WiFi UDP | `huge_app.csv` | PSRAM cache fix enabled. Used by `test_raw_cam`. |
| `esp32dev_ota` | Generic ESP32-WROOM-32 | WiFi (espota) | `min_spiffs.csv` | OTA over-the-air upload; requires existing firmware built with `ENABLE_ARDUINO_OTA=1`. |
| `esp32s3sense_ota` | Seeed XIAO ESP32-S3 Sense | WiFi (espota) | `min_spiffs.csv` | Same for the S3 sense board. |

All WiFi envs inherit the `esp32_microros_wifi` base, which sets `MICRO_ROS_TRANSPORT_ARDUINO_WIFI` and injects network configuration from `network_config.ini` as preprocessor defines: `AGENT_IP`, `AGENT_PORT`, `WIFI_SSID`, `WIFI_PASSWORD`, `STATIC_IP`, `GATEWAY`, `SUBNET`.

Pin definitions live in `mcu_ws/lib/RobotConfig/RobotConfig.h`, gated on `ENV_ESP32S3SENSE` / `ENV_ESP32DEV` / `ENV_ESP32CAM` macros that each env sets in its `build_flags`.

---

## Shared libraries (`mcu_ws/lib/`)

Every subfolder is a PlatformIO library shared across all sketches via `lib_extra_dirs = ../../lib/`. Role at a glance:

| Library | Role |
|---|---|
| `ThreadedSubsystem` | Base class for every hardware subsystem; spawns a pinned FreeRTOS task via `beginThreadedPinned(stack, prio, delayMs, core)`. |
| `RobotConfig` | Header-only pin map, gated on board env macros. |
| `hal_thread` | `Threads::Mutex` / `Threads::Scope` wrapper used by every subsystem to protect shared resources. |
| `GyroSubsystem` | BNO085 IMU via `Adafruit_BNO08x`. Provides orientation (game rotation vector), angular velocity, linear acceleration. |
| `LidarSubsystem` | LD14P 360° LiDAR via UART (230400 baud). 720 points/scan at ~6 Hz. Runs on core 0 at 1 ms cadence. |
| `BatterySubsystem` | Calibrated ADC voltage read with EMA filter. |
| `ServoSubsystem` | Per-channel servo controller with arm/disarm, velocity limits, acceleration profiling, PWM budgeting. |
| `PCA9685` | Hardware driver for the PCA9685 16-channel PWM controller used to drive servos. |
| `GaitController` | Tripod-gait state machine on top of `HexapodKinematics` + `ServoSubsystem`. |
| `HexapodKinematics` | Inverse kinematics for 2-DOF-per-leg hexapod. |
| `CameraSubsystem` | OV2640 MJPEG HTTP server on port 80. |
| `MicSubsystem` | PDM microphone I²S + HTTP PCM server on port 81. |
| `CamMicSubsystem` | Thin wrapper that runs `CameraSubsystem` and `MicSubsystem` side-by-side. |
| `SpeakerSubsystem` | I²S speaker output; long-polls the ROS host for audio. |
| `LedSubsystem` | SK6812 RGB LED chain with patterns (solid, pulse, chase, rainbow…). |
| `OledSubsystem` | SSD1306 128×64 I²C display. Frame + text-overlay model (up to 4 text slots, PROGMEM bitmap frames in `OledFrames.h`), renders CPU-side into a NanoCanvas; only the final `blt()` touches I²C under the shared bus mutex. Also runs an HTTP client task (`lcdFetchTask`) that fetches framebuffers from the ROS 2 host at `GET /lcd_out` (port 8384) — no micro-ROS required for display updates. |
| `ESP32WifiSubsystem` | WiFi connect/reconnect state machine with static IP. |
| `BleDebugSubsystem` | BLE Nordic UART transport for debug output. |
| `HeartbeatParticipant` | Minimal `IMicroRosParticipant` that just publishes a 1 Hz counter on `/mcu/heartbeat`. |
| `Microros` | `MicrorosManager` + `IMicroRosParticipant` interface + `MicroRosContext` helpers. |
| `MicroRosBridge` | Compile-time plugin bridge: conditionally publishes IMU/battery/LiDAR/log/heartbeat based on `BRIDGE_ENABLE_*` flags. |
| `MicroRosDebug` | Debug print helpers for the micro-ROS transport state. |
| `CustomDebug` | Multi-transport debug log (`DEBUG_TRANSPORT_SERIAL`, `DEBUG_TRANSPORT_BLUETOOTH`, `DEBUG_TRANSPORT_MICROROS`) gated by `DEBUG_LEVEL`. |

See **[Architecture → MicroRosBridge](Architecture.md#the-microrosbridge-compile-time-plugin-pattern)** for the plugin pattern.

---

## `BRIDGE_ENABLE_*` flags

Flags default to `0`. Set `-DBRIDGE_ENABLE_FOO=1` in the **sketch's** `platformio.ini` to opt in:

| Flag | Publisher | Topic |
|---|---|---|
| `BRIDGE_ENABLE_HEARTBEAT` | Heartbeat counter | `/mcu/heartbeat` (`std_msgs/Int32`, 1 Hz) |
| `BRIDGE_ENABLE_GYRO` | BNO085 IMU | `/mcu/imu` (`sensor_msgs/Imu`, 50 Hz) |
| `BRIDGE_ENABLE_BATTERY` | Battery voltage | `/mcu/battery_voltage` (`std_msgs/Float32`, 1 Hz) |
| `BRIDGE_ENABLE_LIDAR` | LD14P LiDAR | `/mcu/scan` (`sensor_msgs/LaserScan`, ~6 Hz, 20 Hz cap) |
| `BRIDGE_ENABLE_DEBUG` | Log strings | `/mcu/log` (`std_msgs/String`, event) |
| `BRIDGE_ENABLE_SERVO` | *(reserved — stubbed)* | — |

> **OLED display** is not part of the bridge. `OledSubsystem` runs its own HTTP client task that fetches 1024-byte SSD1306 framebuffers from the ROS 2 host at `GET /lcd_out` (port 8384, served by `seeker_display` or `seeker_media`). No micro-ROS agent is required for the OLED.

Disabled publishers cost zero RAM — the internal state struct becomes `EmptyState` via `std::conditional_t`.

---

## Build / flash / monitor

Every PlatformIO command runs from inside the sketch directory — each sketch is its own project.

```bash
docker compose exec ros2 bash   # enter the container first
cd ~/mcu_workspaces/seeker_mcu/src/<sketch>

pio run                                # build the default env
pio run -e esp32s3sense                # build a specific env
pio run -e esp32s3sense -t upload      # serial flash
pio run -e esp32s3sense_ota -t upload  # OTA upload over WiFi (requires existing OTA firmware)
pio device monitor -b 921600           # open the serial monitor
pio run --target clean                 # wipe the build dir (e.g. after changing mcu_msgs)
```

### Serial monitor baud

The base `[env]` section sets `monitor_speed = 921600`. All sketches print at that rate.

---

## USB passthrough per OS

**Linux:** `/dev` is bind-mounted into the container privileged, so `/dev/ttyUSB0` / `/dev/ttyACM0` work out of the box. If you see `Permission denied`, add your user to the `dialout` group on the host (the container already runs with `privileged: true`).

**Windows:** Docker Desktop can't see host COM ports directly. Use `usbipd-win`:

```powershell
usbipd list                          # identify the BUSID
usbipd bind   --busid <BUS_ID>
usbipd attach --wsl --busid <BUS_ID>
```

The ESP32 then appears as `/dev/ttyUSB0` inside the container. Alternatively, flash with an `*_ota` env and skip `usbipd` entirely.

**macOS:** Docker Desktop containers cannot access USB serial devices on macOS. Flash the ESP32 from the host (e.g. with PlatformIO directly or `esptool.py`), then use WiFi micro-ROS and OTA for everything else.

---

## Running the micro-ROS agent

The WiFi-capable PlatformIO envs set `board_microros_transport = wifi`, so you need the UDP agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# Expected: 'create_session' log line when the ESP32 connects
```

For serial transport envs (`*_serial`), use:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Start the agent **before** flashing — the firmware boots and starts pinging immediately.

---

## Adding a new sketch

1. Create `mcu_ws/src/<new_sketch>/` with a `src/` subdirectory.
2. Copy a nearby sketch's `platformio.ini` as a template. The minimal structure is:

   ```ini
   [platformio]
   default_envs = esp32s3sense
   extra_configs = ../../platformio/platformio.ini

   [env:esp32s3sense]
   extends = env:esp32s3sense   ; inherits from the shared file
   build_flags =
       ${env:esp32s3sense.build_flags}
       -DBRIDGE_ENABLE_HEARTBEAT=1      ; pick whichever publishers you want
   ```

3. Put your code in `src/main.cpp` (or add files and let PlatformIO pick them up).
4. If it's serial-only and doesn't need micro-ROS, override `lib_extra_dirs` to drop `libs_external/esp32`:

   ```ini
   [env:myboard]
   platform = espressif32
   board    = esp32dev
   framework = arduino
   lib_extra_dirs = ../../lib/
   lib_deps = ${common.lib_base}
   ```

5. Follow the naming convention described in **[Contributing → Naming](Contributing.md#naming-conventions)**.

---

## See also

- **[MCU Sketches](MCU-Sketches.md)** — what every sketch under `mcu_ws/src/` actually does.
- **[IRL Tests](IRL-Tests.md)** — hardware bring-up playbook.
- **[Architecture](Architecture.md)** — FreeRTOS task pinning, `MicroRosBridge`, reconnection state machine.
