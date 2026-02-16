# mcu_ws

PlatformIO workspace for ESP32 firmware using micro-ROS (Jazzy distro).

## Directory Structure

| Directory | Purpose |
|---|---|
| `src/` | Firmware source directories. Each subfolder is a build target selected via `build_src_filter` in `platformio.ini`. |
| `lib/` | Shared libraries available to all targets. |
| `libs_external/` | External libraries (e.g. `esp32/micro_ros_platformio`) that don't fit in `lib/` or `src/`. Keeping micro-ROS here ensures it is built once for all ESP32 targets. |
| `extra_packages/` | Extra ROS 2 packages (including `mcu_msgs`) needed at micro-ROS build time. |
| `test/` | Test sources. |
| `platformio.ini` | Build configuration. Targets extend hardware base configs (`esp32_base`, `esp32_microros_wifi`, etc.); C++17 is enforced. |

## Build Commands

Inside the Docker dev container:

```bash
cd ~/mcu_workspaces/seeker_mcu
pio run                          # build default env (main)
pio run -e <env_name>            # build a specific environment
pio run -t upload                # flash via serial
```

## Flashing from Windows (raw bin)

```bash
pip install esptool
esptool --chip esp32 --port "$PORT" --baud 921600 write_flash -z 0x10000 "$ESP32_BIN"
```

## Notes

- The micro-ROS distro is set to **Jazzy** to match the ROS 2 agent.
- Changes to `mcu_msgs` (in `ros2_ws/src/mcu_msgs/`) require rebuilding both the ROS 2 side (`colcon build --packages-select mcu_msgs`) and the MCU side (`pio run`).