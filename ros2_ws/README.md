# Seeker Robot — ROS 2 Workspace

## System Overview

Seeker is a hexapod robot whose mission is to find objects based on voice commands.
The user says "find the red ball"; the robot searches its environment, navigates to the
object, and reports back. All autonomy logic runs in this ROS 2 workspace. The physical
body is driven by an ESP32 MCU running micro-ROS firmware (in `mcu_ws/`).

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3 Sense |
| Servo driver | PCA9685 (16-channel PWM, I²C) |
| Servos | 12 × MG90S (2 per leg: hip yaw + knee pitch) |
| IMU | BNO085 (rotation vector, gyro, accel) |
| LiDAR | LD14P 360° 2D scanner |
| Camera | OV2640 (ESP32-S3 Sense onboard) |
| Microphone | PDM microphone (ESP32-S3 Sense onboard) |
| Connectivity | WiFi (micro-ROS transport) + BLE (debug) |
| LEDs | SK6812 addressable RGB |

Camera and microphone are **not** published via micro-ROS — they are served by a
lightweight HTTP/WebSocket server running directly on the MCU, separate from the
micro-ROS agent connection.

---

## MCU ↔ ROS 2 Topic Interface

All micro-ROS topics use the `/mcu/` prefix so hardware and simulation topics are
interchangeable from the rest of the stack's perspective.

### MCU publishes → ROS 2

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/mcu/joint_states` | `sensor_msgs/JointState` | 100 Hz | 12 servo angles → TF tree via `robot_state_publisher` |
| `/mcu/imu` | `sensor_msgs/Imu` | 200 Hz | BNO085 rotation vector + gyro + accel |
| `/mcu/scan` | `sensor_msgs/LaserScan` | 10 Hz | LD14P 360° 2D scan |
| `/mcu/battery_voltage` | `std_msgs/Float32` | 1 Hz | ADC-calibrated cell voltage |
| `/mcu/heartbeat` | `std_msgs/Int32` | 1 Hz | Incrementing counter; drop = connection lost |

### ROS 2 publishes → MCU

| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | `geometry_msgs/Twist` | Forward velocity + yaw rate; drives tripod gait |
| `/mcu/hexapod_cmd` | `mcu_msgs/HexapodCmd` | Gait mode (STAND/WALK/SIT) + body height/pitch/roll |

### MCU web server (not ROS 2)

| Endpoint | Protocol | Notes |
|----------|----------|-------|
| `/camera` | HTTP MJPEG | Live camera feed for object recognition |
| `/audio`  | WebSocket  | Raw PCM audio stream for speech-to-text |

---

## Node Graph

```
  Voice input ──► [Speech-to-Text]
                       │ text command
                       ▼
               [LLM / command parser]
                       │ JSON goal
                       ▼
              [Mission planner node]
                       │
         ┌─────────────┴──────────────┐
         │                            │
         ▼                            ▼
   /mcu/hexapod_cmd            /goal_pose (Nav2)
         │                            │
         ▼                            ▼
      [MCU]                     [Nav2 stack]
                                      │ /cmd_vel
              [teleop override] ──────┤
                                      ▼
                              [MCU] → servos walk

[MCU] ──► /mcu/imu  ──────► [robot_localization EKF] ──► /odom
[MCU] ──► /mcu/scan ──────► [slam_toolbox] ──────────────► /map
[MCU] ──► /mcu/joint_states ► [robot_state_publisher] ──► /tf

/odom + /map + /tf ──► [Nav2] ──► /cmd_vel
```

---

## Packages

| Package | Purpose |
|---------|---------|
| `mcu_msgs` | Custom message/service definitions shared between ROS 2 and micro-ROS firmware |
| `seeker_description` | URDF/Xacro robot description; launch files for RViz display |
| `seeker_gazebo` | Gazebo Harmonic simulation world, bridge config, and teleop launch |
| `seeker_sim` | `fake_mcu_node` — Python gait simulator; stand-in for the ESP32 during development |
| `seeker_navigation` | Nav2 configuration, SLAM params, and autonomous behavior nodes *(Reyjay/Slam branch)* |

---

## Docker Setup

### GPU Access (NVIDIA)

Gazebo requires GPU access for hardware-accelerated rendering. Without it, Gazebo falls
back to software rendering on a single CPU core, causing poor real-time performance and
a low real-time factor (RTF).

**One-time host setup:**

1. Install the NVIDIA driver on the host (required first — a reboot is needed after this):
   ```bash
   sudo ubuntu-drivers autoinstall
   sudo reboot
   # After reboot, verify: nvidia-smi
   ```

2. Install the NVIDIA Container Toolkit and configure Docker (no reboot needed):

   Follow the official install guide: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

3. Recreate the container (GPU devices are allocated at container creation time and cannot
   be added to a running container):
   ```bash
   cd docker/
   docker compose down
   docker compose up -d ros2
   ```

**Verify inside the container:**
```bash
nvidia-smi   # should show your GPU
```

The `NVIDIA_VISIBLE_DEVICES=all` and `NVIDIA_DRIVER_CAPABILITIES=all` environment
variables are already set in `docker/docker-compose.yml`, along with the device reservation.
EGL/DRI warnings in the Gazebo launch output should disappear once GPU access is working.

---

## Running the Simulation

```bash
# 1. Build (inside Docker container)
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mcu_msgs seeker_description seeker_gazebo seeker_sim --symlink-install
source install/setup.bash

# 2. Launch Gazebo + fake_mcu_node
ros2 launch seeker_gazebo sim_teleop.launch.py

# 3. Drive (second terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teleop keys:

| Key | Action |
|-----|--------|
| `i` / `,` | Forward / backward |
| `j` / `l` | Rotate left / right |
| `k` | Stop — snaps all legs to neutral stance |
| `t` / `b` | Raise / lower body height (adjusts knee angle while walking or standing) |
| `q` / `z` | Increase / decrease linear speed |
| `w` / `x` | Increase / decrease angular speed |

> **Tip:** Build with `--symlink-install` so changes to `fake_mcu_node.py` (gait tuning
> constants, height limits) take effect immediately on next launch without rebuilding.

See `ros2_ws/src/seeker_gazebo/README.md` for the full simulation runbook.

---

## Running on Hardware

```bash
# Flash MCU
cd ~/mcu_workspaces/seeker_mcu/src/test_bridge_gait
pio run -e esp32s3sense -t upload

# Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Start robot_state_publisher (remapped to /mcu/joint_states)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro $(ros2 pkg prefix --share seeker_description)/urdf/seeker_hexapod.urdf.xacro)" \
  -r joint_states:=/mcu/joint_states

# Drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Architecture Notes (corrections to whiteboard diagram)

1. **`robot_state_publisher`** reads URDF + `/mcu/joint_states` to build `/tf`.
   It does not consume `/odom` or `/map` directly.

2. **Camera and microphone** go through an HTTP/WebSocket server on the MCU,
   not micro-ROS. They feed the mission planning pipeline outside the ROS 2 graph.

3. **LLM pipeline**: Voice → speech-to-text → LLM → JSON → ROS 2 mission planner
   node → either `/goal_pose` for Nav2 or `/mcu/hexapod_cmd` for mode changes.

4. **EKF inputs**: Fuses `/mcu/imu` + odometry source → `/odom`.
   Does not consume `/mcu/joint_states`.

5. **SLAM toolbox**: Consumes `/mcu/scan` + `/odom` → `/map`.
