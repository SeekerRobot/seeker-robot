# Seeker Robot Wiki

**Seeker Robot** is a six-legged autonomous robot built on **ROS 2 Jazzy** and an **ESP32-S3** firmware stack talking to the host through **micro-ROS** over WiFi. The entire development environment lives inside a single multi-stage Docker image, so you can go from clone to running simulation (or flashing firmware) with the same five commands on Linux, macOS, or Windows.

The robot maps its environment with a 360° LiDAR and a BNO085 IMU, navigates with Nav2 + SLAM Toolbox, and hunts for objects using YOLO-based camera detection and frontier exploration. A Fish Audio-backed TTS node closes the loop for voice feedback.

---

## Table of Contents

| Page | What it covers |
|---|---|
| **[Setup](Setup.md)** | Host prerequisites (Docker, X server per OS, `usbipd-win`), cloning the repo, filling in `docker/.env` and `network_config.ini`, first build, smoke tests, troubleshooting. |
| **[Architecture](Architecture.md)** | How ROS 2, the micro-ROS agent and the ESP32 firmware fit together. Docker volume / bind-mount layout, FreeRTOS task pinning, the `MicroRosBridge` compile-time plugin pattern, how `mcu_msgs` flows between the two workspaces. |
| **[ROS2 Packages](ROS2-Packages.md)** | Per-package reference for everything under `ros2_ws/src/`: purpose, key nodes, launch files, and how to build each one. |
| **[Simulation](Simulation.md)** | End-to-end recipes for every Gazebo simulation mode (teleop, SLAM-raw, SLAM+EKF, full ball-search autonomy), expected topics/TF, and debugging tips. |
| **[MCU Firmware](MCU-Firmware.md)** | Board environments, transports (WiFi/serial/OTA), shared libraries, generic build/flash/monitor commands, USB passthrough per OS, running the micro-ROS agent. |
| **[MCU Sketches](MCU-Sketches.md)** | Per-sketch reference for every project under `mcu_ws/src/`: purpose, build command, expected output, and debugging tips. |
| **[IRL Tests](IRL-Tests.md)** | Real-hardware bring-up checklist, full integration launches, gait test, navigation test, per-subsystem isolation tests, common debugging commands. |
| **[Contributing](Contributing.md)** | Commit message format, CI checks, naming conventions, how to add a sketch or ROS 2 package, the `BRIDGE_ENABLE_*` checklist, PR checklist. |

---

## Five-command quick start

```bash
# 1. Clone
git clone --recurse-submodules https://github.com/SeekerRobot/seeker-robot.git && cd seeker-robot

# 2. Configure (edit the copies afterwards — WiFi credentials, agent IP, OS display/network block)
cp docker/.env.example docker/.env
cp mcu_ws/platformio/network_config.example.ini mcu_ws/platformio/network_config.ini
# In docker/.env set COMPOSE_PROFILES=cpu (or nvidia/amd for GPU hosts)

# 3. Build + start the dev container
cd docker && docker compose build && docker compose up init-bootstrap && docker compose up -d ros2

# 4. Shell into it and build ROS 2 packages
docker compose exec ros2 bash
# inside the container:
cd ~/ros2_workspaces && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash

# 5. Run something — e.g. manual-drive simulation
ros2 launch seeker_gazebo sim_teleop.launch.py
# in another terminal: ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

See **[Setup](Setup.md)** for the full walkthrough and **[Simulation](Simulation.md)** for every launch mode.

---

## Repository at a glance

```
seeker-robot/
├── ros2_ws/           # ROS 2 Jazzy colcon workspace
│   └── src/
│       ├── mcu_msgs/           # Shared .msg/.srv between ROS 2 and micro-ROS
│       ├── seeker_description/ # URDF + robot_state_publisher
│       ├── seeker_gazebo/      # Gazebo Harmonic simulation launches
│       ├── seeker_display/     # OLED display nodes (HTTP LCD server + demos)
│       ├── seeker_media/       # MP4 player (video → OLED + audio → speaker)
│       ├── seeker_navigation/  # Nav2 + SLAM + EKF + ball_searcher
│       ├── seeker_sim/         # fake_mcu_node (tripod gait sim)
│       ├── seeker_tts/         # Fish Audio TTS bridge
│       ├── seeker_vision/      # YOLO object detection + emotion detection + camera proxy
│       ├── seeker_web/         # Browser-based robot controller (WebSocket + REST)
│       └── test_package/       # Minimal CI sanity package
├── mcu_ws/            # PlatformIO workspace (ESP32 firmware)
│   ├── platformio/             # Shared base platformio.ini + network_config.ini
│   ├── src/                    # Per-sketch PlatformIO projects (31 sketches)
│   ├── lib/                    # Shared C++ libraries (subsystems, bridge, kinematics...)
│   ├── libs_external/          # Pre-vendored micro-ROS PlatformIO library
│   └── platformio/extra_packages/ # mcu_msgs bind-mounted from ros2_ws for micro-ROS builds
├── docker/            # Multi-stage Dockerfile + compose + init-bootstrap
├── doc/               # Hardware docs (PCB design files live here)
└── scripts/           # Utility scripts (bind-mounted into the container at ~/scripts)
```

---

## Key topics (live once the ESP32 is flashed with `test_bridge_all`)

| Topic | Type | Rate | Direction |
|---|---|---|---|
| `/mcu/heartbeat` | `std_msgs/Int32` | 1 Hz | ESP32 → ROS |
| `/mcu/imu` | `sensor_msgs/Imu` | 50 Hz | ESP32 → ROS |
| `/mcu/battery_voltage` | `std_msgs/Float32` | 1 Hz | ESP32 → ROS |
| `/mcu/scan` | `sensor_msgs/LaserScan` | ~6 Hz | ESP32 → ROS |
| `/mcu/log` | `std_msgs/String` | event | ESP32 → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | on demand | ROS → ESP32 |
| `/mcu/hexapod_cmd` | `mcu_msgs/HexapodCmd` | on demand | ROS → ESP32 |

See **[Architecture](Architecture.md)** for the full topic graph and TF tree.
