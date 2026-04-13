# Seeker Robot

A six-legged autonomous robot built on ROS 2 Jazzy and ESP32 firmware. The robot maps its environment using a 360° LiDAR and IMU, navigates with Nav2, and hunts for objects using camera-based detection and an LLM-driven voice command interface.

## AI Disclosure

This project was developed with assistance of AI-powered code generation tools. Contributors and maintainers are clear (to a reasonable extent) about what files contain AI-generated code and how they were reviewed.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ESP32-S3 (Sesame V2 PCB)                           │
│                                                                             │
│  BNO085 IMU ──┐                                                             │
│  LD14P LiDAR ─┤─→ micro-ROS (WiFi UDP) ─────────────────────────────────┐  │
│  PCA9685 Servos┤                                                         │  │
│  Battery ADC ──┘                                                         │  │
│                                                                          │  │
│  Camera ──┐                                                              │  │
│  Mic    ──┴──→ Web Server (WiFi)                                         │  │
│                  /camera  (MJPEG)                                        │  │
│                  /audio   (PCM)                                          │  │
└──────────────────────────────────────────────────────────────────────────┼──┘
                                                                           │
                                    WiFi                                   │
                                                                           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Jazzy (Docker Container)                      │
│                                                                             │
│  micro-ROS Agent ◄──────────────────────────────────────────────────────────┤
│       │                                                                     │
│       │  Publishes          Subscribes                                      │
│       │  /mcu/imu           /cmd_vel         ◄── Nav2 / teleop              │
│       │  /mcu/scan          /mcu/hexapod_cmd ◄── mission planner            │
│       │  /mcu/battery_voltage                                               │
│       │  /mcu/heartbeat                                                     │
│       │                                                                     │
│       ▼                                                                     │
│  robot_localization EKF                                                     │
│    fuses /mcu/imu (roll/pitch from gravity)                                 │
│    → /odom  +  TF: odom→base_footprint (with roll/pitch)                   │
│       │                                                                     │
│       ├──→ SLAM Toolbox ──→ /map  +  TF: map→odom                          │
│       │       uses TF chain to un-tilt LiDAR rays in map frame             │
│       │                                                                     │
│       └──→ robot_state_publisher (URDF fixed joints)                        │
│                TF: base_footprint→base_link→laser / camera / imu           │
│                                                                             │
│  Nav2 (/map + /odom) ──→ /cmd_vel                                           │
│                                                                             │
│  Web Client ──→ Speech-to-text ──→ LLM ──→ JSON commands                   │
│               Camera feed  ──→ Object detection ──→ mission planner        │
│                                   ▲                                         │
│                            ball_searcher                                    │
│                            (frontier exploration)                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

### TF Tree

```
map
 └── odom                  (from SLAM Toolbox: map→odom correction)
      └── base_footprint   (from EKF: roll/pitch from BNO085 game rotation vector)
           └── base_link   (fixed offset, from URDF via robot_state_publisher)
                ├── laser_link   (LiDAR mount position)
                ├── camera_link  (camera mount position)
                └── imu_link     (IMU co-located with LiDAR)
```

**Tilt compensation:** The hexapod body rolls and pitches during walking. The BNO085 game rotation vector measures this with no yaw drift. The EKF bakes roll/pitch into the `odom→base_footprint` transform, so SLAM Toolbox automatically corrects each LiDAR ray when projecting it into the map frame — no custom filter node required.

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| MCU | Seeed XIAO ESP32-S3 | On Sesame V2 PCB |
| IMU | BNO085 | I2C, game rotation vector (drift-free roll/pitch) |
| LiDAR | LDRobot LD14P | UART 230400, 360°, 6 Hz, 720 points/scan |
| Servo driver | PCA9685 | I2C, 12 servos (6 legs × 2 DOF) |
| Power | TPS54427 | Buck regulator on Sesame V2 PCB |
| LEDs | SK6812 SIDE-A | RGB side-emitting |
| Camera | ESP32-S3 onboard | 640×480, MJPEG stream |
| Microphone | ESP32-S3 onboard | PCM audio stream |

PCB design files: `doc/pcb/sesamepcb/`

---

## Repository Layout

```
seeker-robot/
├── ros2_ws/              # ROS 2 colcon workspace
│   └── src/
│       ├── mcu_msgs/           # Shared message definitions (ROS2 ↔ micro-ROS)
│       ├── seeker_description/ # URDF, robot_state_publisher launch
│       ├── seeker_gazebo/      # Gazebo Harmonic simulation
│       ├── seeker_navigation/  # Nav2, SLAM, EKF configs, mission planner
│       ├── seeker_sim/         # Simulated MCU node (fake_mcu_node)
│       ├── seeker_tts/         # Text-to-speech node (Fish Audio API)
│       └── test_package/       # Minimal test package
├── mcu_ws/               # PlatformIO workspace (ESP32 firmware)
│   ├── platformio/             # Shared board/library config (inherited by all sketches)
│   ├── src/                    # Firmware sketches (each is a standalone PlatformIO project)
│   ├── lib/                    # Shared C++ libraries
│   └── extra_packages/         # mcu_msgs for micro-ROS build (bind-mounted from ros2_ws)
└── docker/               # Containerized dev environment
    ├── Dockerfile              # Multi-stage: base → dev/prod
    ├── docker-compose.yml
    └── .env.example
```

---

## ROS 2 Packages

| Package | Purpose |
|---------|---------|
| `mcu_msgs` | Custom message definitions shared between ROS 2 and ESP32 firmware (`HexapodCmd.msg`) |
| `seeker_description` | URDF/Xacro hexapod model, `display.launch.py` for RViz |
| `seeker_gazebo` | Gazebo Harmonic simulation, sensor bridges, simulation launch files |
| `seeker_navigation` | Nav2 + SLAM Toolbox + EKF configs, `ball_searcher` mission planner, real robot launch |
| `seeker_sim` | `fake_mcu_node`: simulates ESP32 gait for testing without hardware |
| `seeker_tts` | Text-to-speech node (Fish Audio API integration) |
| `test_package` | Minimal ROS 2 node for build/workflow verification |

---

## MCU Firmware Sketches

| Sketch | Purpose |
|--------|---------|
| `test_bridge_all` | All sensor publishers enabled (heartbeat, IMU, battery, LiDAR, debug). Use for SLAM/navigation testing. |
| `test_bridge_gait` | Gait + micro-ROS. Subscribes to `/cmd_vel` for walking. No sensor publishing. |
| `test_all` | Integration test for all subsystems together |
| `test_threaded_blink` | ThreadedSubsystem / FreeRTOS task smoke test |
| `test_fast_led_raw` | FastLED SK6812 RGB LED test |
| `test_raw_cam` | Raw camera I2C/PSRAM test (no subsystem abstraction) |
| `test_raw_mic` | PDM microphone raw data test |
| `test_sub_gyro_nondma` | BNO085 IMU in isolation (serial only, no micro-ROS) |
| `test_sub_lidar` | LD14P LiDAR in isolation |
| `test_sub_servo` | PCA9685 servo motion profiling in isolation |
| `test_sub_battery` | Battery ADC calibration check |
| `test_sub_gait` | Gait kinematics smoke test (no ROS, no hardware) |
| `test_sub_cam` | Camera stream test (OV2640 MJPEG) |
| `test_sub_cam_mic` | Camera + microphone integration test |
| `test_sub_heartbeat` | micro-ROS heartbeat publisher test |
| `test_sub_led` | SK6812 LED subsystem test |
| `test_sub_mic` | I2S PDM microphone test |
| `test_sub_speaker` | I2S speaker output test |
| `test_sub_wifi` | WiFi connectivity test |
| `test_sub_ble_debug` | BLE debug output test |
| `main` | Placeholder for full system integration (currently empty) |

---

## Topic Reference

| Topic | Type | Rate | QoS | Direction | Notes |
|-------|------|------|-----|-----------|-------|
| `/mcu/imu` | `sensor_msgs/Imu` | 50 Hz | BEST_EFFORT | ESP32 → ROS | Orientation from game rotation vector, angular vel, linear accel |
| `/mcu/scan` | `sensor_msgs/LaserScan` | ~6 Hz | RELIABLE | ESP32 → ROS | 720 rays, 8 m range, frame: `lidar_link` |
| `/mcu/battery_voltage` | `std_msgs/Float32` | 1 Hz | BEST_EFFORT | ESP32 → ROS | Calibrated voltage in volts |
| `/mcu/heartbeat` | `std_msgs/Int32` | 1 Hz | BEST_EFFORT | ESP32 → ROS | Monotonic counter, confirms ESP32 is alive |
| `/mcu/log` | `std_msgs/String` | event | BEST_EFFORT | ESP32 → ROS | Debug messages from firmware |
| `/cmd_vel` | `geometry_msgs/Twist` | on demand | BEST_EFFORT | ROS → ESP32 | Walking velocity (vx, vy, wz) |
| `/mcu/hexapod_cmd` | `mcu_msgs/HexapodCmd` | on demand | BEST_EFFORT | ROS → ESP32 | Gait mode (STAND/WALK/SIT), body height/pitch/roll |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | — | EKF output | EKF-estimated odometry; orientation / roll-pitch TF from IMU |
| `/map` | `nav_msgs/OccupancyGrid` | ~0.2 Hz | TRANSIENT_LOCAL | SLAM output | 5 cm/cell occupancy grid |

---

## Prerequisites

| Tool                        | Link                                                          | Notes                          |
| --------------------------- | ------------------------------------------------------------- | ------------------------------ |
| **Git**                     | [git-scm.com](https://git-scm.com/)                           | Required for cloning the repo  |
| **Docker Desktop**          | [docker.com](https://www.docker.com/products/docker-desktop/) | Runs the development container |
| **VSCode**                  | [code.visualstudio.com](https://code.visualstudio.com/)       | Recommended editor             |
| **X Server** (for GUI apps) | See [X11 Setup](#x11-server-setup) below                      | Required for Gazebo, RViz, rqt |

## Quick Start

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/SeekerRobot/seeker-robot.git
cd seeker-robot
```

### 2. Configure

```bash
cp docker/.env.example docker/.env
cp mcu_ws/platformio/network_config.example.ini mcu_ws/platformio/network_config.ini
```

Edit `docker/.env`:
- `COMPOSE_PROJECT_NAME` — unique name (e.g. `seeker-robot`)
- `BUILD_TARGET` — `dev` (includes Gazebo/RViz) or `prod` (headless)
- Uncomment the Display/Network block for your OS

Edit `mcu_ws/platformio/network_config.ini` with your WiFi credentials and the IP of the machine running the Docker container (the micro-ROS agent IP).

### 3. X11 Setup (Windows/macOS only)

**Windows:** Install VcXsrv, launch with "Disable access control" checked. Set `DISPLAY_CONFIG=host.docker.internal:0` and `NETWORK_MODE_CONFIG=bridge` in `.env`.

**macOS:** Install XQuartz, enable "Allow connections from network clients", run `xhost +localhost`. Same `.env` settings as Windows.

**Linux:** No extra setup. Set `DISPLAY_CONFIG=${DISPLAY}` and `NETWORK_MODE_CONFIG=host`.

### 4. Start the Container

```bash
cd docker
docker compose build
docker compose up init-bootstrap   # one-time volume init
docker compose up -d ros2
docker compose exec ros2 bash      # enter the container
```

### 5. Build ROS 2 Packages

```bash
# Inside container
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Real Hardware

### Flash the ESP32

```bash
# Inside container — from the sketch directory
cd ~/mcu_workspaces/seeker_mcu/src/test_bridge_all
pio run -e esp32s3sense -t upload
```

On Windows, USB passthrough requires [usbipd-win](https://github.com/dorssel/usbipd-win):
```powershell
usbipd bind --busid <BUS_ID>
usbipd attach --wsl --busid <BUS_ID>
```

### Launch the Real Robot Stack

Start the micro-ROS agent first (separate terminal, keep running):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# Wait for: [create_session] — ESP32 has connected
```

Then pick the appropriate launch:

```bash
# SLAM only (no IMU, simplest — good for initial bringup):
ros2 launch seeker_navigation real_slam_raw.launch.py

# SLAM + IMU tilt compensation (recommended for real deployment):
ros2 launch seeker_navigation real_slam_ekf.launch.py

# Full autonomy (EKF + SLAM + Nav2 + ball searcher):
ros2 launch seeker_navigation real_ball_search.launch.py
```

**Launch sequence (real_slam_ekf / real_ball_search):**
- t=0 s — robot_state_publisher
- t=2 s — EKF (fuses `/mcu/imu` → `odom→base_footprint` with roll/pitch)
- t=4 s — SLAM Toolbox (subscribes to `/mcu/scan`)
- t=10 s — SLAM lifecycle configure + activate
- t=13 s — Nav2 stack (real_ball_search only)
- t=25 s — ball_searcher (real_ball_search only)

### Quick IMU Sanity Check (no launch file)

Verify the IMU is publishing and oriented correctly before running the full stack:

```bash
# Terminal 1 — micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2 — dummy static TF so RViz has a fixed frame
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# Terminal 3 — RViz
rviz2
# Fixed Frame: odom
# Add → Imu → Topic: /mcu/imu
# Tilt the robot and confirm the orientation arrow moves correctly
```

### Verify Tilt Compensation

```bash
# Check topics are arriving from the ESP32
ros2 topic hz /mcu/imu        # expect ~50 Hz
ros2 topic hz /mcu/scan       # expect ~6 Hz

# Check the EKF is publishing roll/pitch in the TF
ros2 run tf2_ros tf2_echo odom base_footprint
# Tilt the robot — roll/pitch values should change in real time

# Check SLAM is building a map
ros2 topic echo /map --once

# Visualize in RViz
rviz2
# Add: TF, LaserScan (/mcu/scan), Map (/map)
# Tilt the robot on a wedge: walls should stay vertical in the map
```

---

## Simulation

All simulation launches are self-contained — just run one command.

```bash
# Manual drive (Gazebo + fake MCU + teleop):
ros2 launch seeker_gazebo sim_teleop.launch.py
# Then in a second terminal: ros2 run teleop_twist_keyboard teleop_twist_keyboard

# SLAM with Gazebo ground-truth odometry (no IMU):
ros2 launch seeker_gazebo sim_slam_raw.launch.py

# SLAM with IMU-fused EKF odometry (tests real hardware pipeline):
ros2 launch seeker_gazebo sim_slam_ekf.launch.py

# Full autonomy demo (EKF + SLAM + Nav2 + ball searcher):
ros2 launch seeker_gazebo sim_ball_search.launch.py
```

**RViz Fixed Frame:**
- `sim_teleop`, `sim_slam_raw`, `sim_slam_ekf` → `odom`
- `sim_ball_search` → `map`

---

## Development

### Build Commands

```bash
# ROS 2
cd ~/ros2_workspaces
colcon build
colcon build --packages-select mcu_msgs   # rebuild single package
colcon test && colcon test-result --verbose

# MCU firmware
cd ~/mcu_workspaces/seeker_mcu/src/<sketch>
pio run                         # build default env (esp32s3sense)
pio run -e esp32dev             # build for specific board
pio run -e esp32s3sense -t upload  # build + flash
```

### After Changing mcu_msgs

Message definitions are shared between both workspaces. Any `.msg` or `.srv` change requires:

```bash
# 1. Rebuild ROS 2 side
colcon build --packages-select mcu_msgs && source install/setup.bash

# 2. Rebuild MCU firmware (extra_packages bind-mount picks up changes)
cd ~/mcu_workspaces/seeker_mcu/src/<sketch>
pio run --target clean && pio run
```

### micro-ROS Agent (manual)

```bash
# WiFi/UDP (matches board_microros_transport = wifi in platformio.ini)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Serial (for test sketches with serial transport)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### Volume Layout (Inside Container)

| Host path | Container path | Type |
|-----------|----------------|------|
| `ros2_ws/src/` | `~/ros2_workspaces/src/seeker_ros/` | Bind mount |
| `mcu_ws/` | `~/mcu_workspaces/seeker_mcu/` | Bind mount |
| `ros2_ws/src/mcu_msgs/` | `~/mcu_workspaces/seeker_mcu/extra_packages/mcu_msgs/` | Bind mount |
| Named volumes | `~/ros2_workspaces/{build,install,log}` | Docker volume |
| Named volume | `~/.platformio` | Docker volume |
| Named volume | `~/mcu_workspaces/seeker_mcu/libs_external` | Docker volume |
