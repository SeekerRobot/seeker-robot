# mcu_ws

PlatformIO workspace for ESP32 firmware using micro-ROS (Jazzy distro).

## Directory Structure

| Directory | Purpose |
|---|---|
| `platformio/` | Shared PlatformIO config. Contains `platformio.ini` (base board environments, build flags, library deps) and `network_config.ini` (WiFi/agent settings, gitignored). |
| `src/` | Firmware sketches. Each subfolder is a standalone PlatformIO project with its own `platformio.ini` that inherits from `platformio/platformio.ini` via `extra_configs`. |
| `lib/` | Shared libraries (e.g., `hal_thread`) available to all sketches via `lib_extra_dirs`. |
| `libs_external/` | External libraries (e.g., `esp32/micro_ros_platformio`) that don't fit in `lib/`. Seeded into a Docker volume by `init-bootstrap`. |
| `extra_packages/` | Extra ROS 2 packages (including `mcu_msgs`) needed at micro-ROS build time. |

## Setup

Copy the network config template and fill in your local values:

```bash
cp platformio/network_config.example.ini platformio/network_config.ini
```

## Build Commands

Inside the Docker dev container, `cd` into a sketch directory and run:

```bash
cd ~/mcu_workspaces/seeker_mcu/src/main

pio run                          # build default board env
pio run -e esp32dev              # build for a specific board
pio run -e esp32dev -t upload    # flash via serial
```

## Adding a New Sketch

1. Create a folder under `src/` (e.g., `src/bno_test/`).
2. Add a minimal `platformio.ini`:
   ```ini
   [platformio]
   extra_configs = ../../platformio/platformio.ini
   ```
3. Add a `src/main.cpp` inside your sketch folder.
4. Build with `pio run -e <board>` from inside the sketch directory.

## Adding a New Board

Add a new `[env:<name>]` section in `platformio/platformio.ini`:

```ini
[env:my_new_board]
extends = esp32_microros_wifi
platform = espressif32
board = <platformio_board_id>
```

All sketches automatically inherit the new board.

## Notes

- The micro-ROS distro is set to **Jazzy** to match the ROS 2 agent.
- Changes to `mcu_msgs` (in `ros2_ws/src/mcu_msgs/`) require rebuilding both the ROS 2 side (`colcon build --packages-select mcu_msgs`) and the MCU side (`pio run`).
- Per-sketch `.pio/` build directories are gitignored and live within the bind mount.
