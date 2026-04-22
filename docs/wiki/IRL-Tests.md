# IRL Tests

This page is the hardware bring-up playbook. It assumes you already have a working dev container (see **[Setup](Setup.md)**), a built ROS 2 workspace, a configured `network_config.ini`, and at least one ESP32-S3 Sense board wired to the Sesame V2 PCB.

---

## Pre-flight checklist

Before every session:

1. **PCB powered from the right rail.** The Sesame V2 expects a battery pack that the TPS54427 buck steps down to 5 V for servos and 3.3 V for logic. Don't feed the ESP32 from the USB-C port when the battery is also connected — that will backfeed the buck.
2. **Servos armed but not locked.** Start with servos detached (`test_sub_servo`'s default). Only `arm` them once you're sure the PCA9685 is programmed with the correct frequency.
3. **LiDAR spinning freely.** LD14P mechanical issues look like firmware bugs. Give the rotor a spin by hand before powering on.
4. **Host and ESP32 on the same subnet.** `agent_ip` in `network_config.ini` must match the host's IP; `static_ip` must be free on that subnet.
5. **micro-ROS agent started in its own terminal.** The ESP32 pings it on boot — if it's not there, you'll wait forever.

```bash
# Terminal 0 — keep this running for the whole session
docker compose exec ros2 bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_workspaces/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# Wait for: [INFO] [...] create_session  (that's the ESP32 connecting)
```

---

## Step-by-step bring-up

These steps are in the right order — don't skip ahead if an earlier one fails.

### 1. Blink + threads

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_threaded_blink
pio run -e esp32s3sense -t upload
pio device monitor -b 921600
```

Expected: onboard LED blinks from a FreeRTOS task. If nothing happens, the board is bad or the wrong env was flashed.

### 2. WiFi connect

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_sub_wifi
pio run -e esp32s3sense -t upload
```

Expected: periodic `WiFi: CONNECTED | SSID: … | IP: … | RSSI: …` on serial. If it never connects, fix `network_config.ini` before going any further.

### 3. micro-ROS heartbeat

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_sub_heartbeat
pio run -e esp32s3sense -t upload
# in Terminal 1 (with the agent running):
ros2 topic echo /mcu/heartbeat
```

Expected: a 1 Hz counter showing up on `/mcu/heartbeat`. This proves the entire transport stack (WiFi → UDP → XRCE-DDS → ROS 2 session) is healthy.

### 4. IMU sanity check

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_sub_gyro_nondma
pio run -e esp32s3senseserial -t upload
pio device monitor -b 921600
```

Expected: BNO085 init success, optionally quaternion output. If `BNO08x not detected`, check I²C pins and pull-ups.

### 5. LiDAR check

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_sub_lidar
pio run -e esp32s3senseserial -t upload
pio device monitor -b 921600
# interactive: `scan`, `stream`, `freq`, `info`
```

Expected: ~720 points per scan, ~6 Hz rotation. `freq 10` bumps it to 10 Hz if your LD14P supports it.

### 6. Servos

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_sub_servo
pio run -e esp32s3senseserial -t upload
pio device monitor -b 921600
# arm; attach 0; angle 0 90   (first leg to 90°)
```

Do this one channel at a time to catch miswired legs. Run through all 12 before moving on.

### 7. Gait standalone

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_sub_gait
pio run -e esp32s3senseserial -t upload
pio device monitor -b 921600
# neutral; start; vel 0.03; status; halt
```

Expected: tripod gait, all legs move. This is the last step before involving ROS 2.

### 8. Full sensor bridge

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_bridge_all
pio run -e esp32s3sense -t upload
# in Terminal 1 (agent must be running):
ros2 topic hz /mcu/heartbeat        # 1 Hz
ros2 topic hz /mcu/imu              # ~50 Hz
ros2 topic hz /mcu/scan             # ~6 Hz
ros2 topic hz /mcu/battery_voltage  # 1 Hz
```

If any topic is missing, check serial for `create_session` and for the `#ifdef BRIDGE_ENABLE_*` guarded init logs.

### 9. Gait over `/cmd_vel`

```bash
cd ~/mcu_workspaces/seeker_mcu/src/test_bridge_gait
pio run -e esp32s3sense -t upload
# in a ROS terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the hexapod with `i/j/l/,` keys. If nothing moves, verify `/cmd_vel` actually publishes: `ros2 topic echo /cmd_vel`.

### 10. Full real-hardware autonomy

Once every step above passes:

```bash
# Option A — test_all (diagnostics + everything on one board):
cd ~/mcu_workspaces/seeker_mcu/src/test_all
pio run -t upload

# Option B — main (production firmware, offloads camera to satellite):
cd ~/mcu_workspaces/seeker_mcu/src/main
pio run -t upload     # defaults to esp32s3sense_offload

# If using a satellite board for camera (Option B):
cd ~/mcu_workspaces/seeker_mcu/src/main_satellite
pio run -t upload     # defaults to esp32cam_satellite

# ROS launch (with agent already running in another terminal):
ros2 launch seeker_navigation real_ball_search.launch.py
```

See the timeline in **[ROS2 Packages → seeker_navigation](ROS2-Packages.md#seeker_navigation)**. It takes ~25 s before `ball_searcher` actually starts issuing navigation goals.

---

## Real-robot launch options

All launches live in `seeker_navigation/launch/`.

| Launch | What it starts | Use for |
|---|---|---|
| `real_slam_raw.launch.py` | robot_state_publisher + **static** `odom → base_footprint` + SLAM Toolbox | First SLAM bring-up without EKF complications |
| `real_slam_ekf.launch.py` | robot_state_publisher + EKF (fuses `/mcu/imu`) + SLAM Toolbox | Best default; matches the real-hardware pipeline |
| `real_ball_search.launch.py` | `real_slam_ekf` + Nav2 + `ball_searcher` | Full autonomy demo |

Typical usage:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888   # Terminal 1
ros2 launch seeker_navigation real_slam_ekf.launch.py        # Terminal 2
rviz2                                                         # Terminal 3 (optional)
# Fixed Frame: map   (odom for real_slam_*)
```

---

## Quick IMU sanity check (no launch file)

Useful when you just want to verify the IMU is publishing and oriented correctly before running any full stack.

```bash
# Terminal 1 — agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2 — dummy static TF so RViz has a fixed frame
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# Terminal 3 — RViz
rviz2
#   Fixed Frame: odom
#   Add → Imu → Topic: /mcu/imu
#   Tilt the robot and confirm the orientation arrow tracks
```

---

## Verifying tilt compensation

With `real_slam_ekf.launch.py` running:

```bash
# Sensors live
ros2 topic hz /mcu/imu                  # ~50 Hz
ros2 topic hz /mcu/scan                 # ~6 Hz

# EKF is publishing roll/pitch in TF
ros2 run tf2_ros tf2_echo odom base_footprint
# Tilt the robot — roll/pitch values should change in real time

# SLAM is building a map
ros2 topic echo /map --once

# Visual check in RViz
rviz2
# Add: TF, LaserScan (/mcu/scan), Map (/map)
# Tilt the robot on a wedge — walls should stay vertical in the map frame
```

---

## Per-subsystem isolation tests

When something misbehaves on the full stack, bisect with the `test_sub_*` sketches. See **[MCU Sketches](MCU-Sketches.md)** for commands.

| Symptom | Isolation sketch |
|---|---|
| No WiFi or wrong subnet | `test_sub_wifi` |
| micro-ROS never connects | `test_sub_heartbeat` |
| IMU missing or wrong orientation | `test_sub_gyro_nondma` |
| LiDAR returning 0 points / wrong frequency | `test_sub_lidar` |
| Battery voltage looks flat | `test_sub_battery` |
| Specific servo leg twitches | `test_sub_servo` |
| Whole gait is wrong | `test_sub_gait`, or `test_sub_movement` for combined servo+gait+IK tuning with NVS persistence |
| Camera feed dead | `test_sub_cam`, then `test_raw_cam` |
| YOLO detection not working | Verify cam proxy: `curl -s http://localhost:8080/stream > /dev/null`; then `ros2 launch seeker_vision mcu_cam.launch.py` |
| Mic silent or crackling | `test_sub_mic`, then `test_raw_mic` |
| TTS playback silent | `test_sub_speaker` + verify `seeker_tts` via `curl` |
| LED chain colour wrong | `test_sub_led`, then `test_fast_led_raw` |
| OLED display dead / wrong frame | `test_sub_oled` (serial-only draw), then `test_bridge_oled` (HTTP LCD path from host :8390), fall back to `test_raw_oled` to rule out `OledSubsystem` |

---

## Common debug commands

```bash
# ROS 2 topic / TF introspection
ros2 topic list
ros2 topic info /mcu/imu
ros2 topic hz /mcu/imu
ros2 topic echo /mcu/imu --once
ros2 run tf2_tools view_frames           # emits PDF in cwd
ros2 run tf2_ros tf2_echo odom base_link
ros2 lifecycle get /slam_toolbox
ros2 node list
ros2 node info /ekf_filter_node

# Firmware serial / upload
pio device list                           # show COM / tty devices
pio device monitor -b 921600 --filter esp32_exception_decoder
pio run -e esp32s3sense -t upload
pio run -e esp32s3sense_ota -t upload     # OTA (WiFi)

# Network
ping <esp32_static_ip>
sudo tcpdump -n -i any udp port 8888     # watch micro-ROS traffic
```

---

## Shutting down

```bash
# Inside the container
# Ctrl-C any running launch files and the micro-ROS agent

# Host, from docker/
docker compose stop ros2
# or to nuke the session entirely:
docker compose down
```

Named volumes (build artifacts, PlatformIO cache) survive `down`. Use `docker compose down -v` only if you explicitly want to wipe them (e.g. to test a clean init-bootstrap pass).
