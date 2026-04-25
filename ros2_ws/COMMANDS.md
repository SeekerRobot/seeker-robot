# Seeker Robot — Simulation & Integration Commands

This guide provides the exact commands used to launch, monitor, and control the autonomous seeker robot in Gazebo.

> **Container name varies by profile.** Check yours first, then export it so
> every command below picks it up:
> ```bash
> docker ps --format "{{.Names}}"        # → copy the seeker-robot-ros2-* name
> export CTR=$CTR         # CPU-only profile
> export CTR=seeker-robot-ros2-nvidia-1  # NVIDIA GPU profile (most common)
> export CTR=seeker-robot-ros2-amd-1     # AMD GPU profile
> ```
> Every `docker exec` below uses `$CTR`. If a command returns no output, you
> probably didn't export it (or set it to a container that isn't running).
>
> **If you're already inside the container** (`docker compose exec ros2-nvidia
> bash` or similar), strip the `docker exec -it $CTR bash -c "..."` wrapper and
> just run the inner command. Source once per shell:
> ```bash
> source /home/ubuntu/ros2_workspaces/install/setup.bash
> ```
> then run `ros2 launch …`, `ros2 topic echo …`, etc. directly.

## Safe mode escape
```echo -n "SEEKER_CLEAR_SM" | nc -u -w1 <esp32-ip> 4210```

## 1. Startup
Launch the full autonomy stack (Gazebo + Nav2 + SLAM + YOLO + Object Seeker).
```bash
docker exec -it -e DISPLAY=:0 $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_gazebo sim_integrated_medium.launch.py"
```

## 2. Triggering a Search
Send an action goal to the robot to find and approach a specific object.

**Find Soccer Ball:**
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \"{class_name: 'sports ball', timeout_sec: 300.0}\" --feedback"
```

**Find Teddy Bear:**
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \"{class_name: 'teddy bear', timeout_sec: 300.0}\" --feedback"
```

## 3. Monitoring & Debugging
Check the robot's internal state machine and search progress.

**Live Feedback (State & Area):**
```bash
docker exec $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic echo /seek_object/_action/feedback"
```

**Follow Logs:**
```bash
docker exec -it $CTR bash -c "tail -f /home/ubuntu/.ros/log/latest/launch.log"
```

---

## 4. Real Hardware — Brain–Body Pipeline

Full IRL stack: micro-ROS agent + EKF + SLAM + Nav2 + YOLO + `object_seeker`
(Action Server) + `command_node` (Gemini Action Client). Whisper is bypassed
via `voice_terminal.py`, which pipes stdin to `/audio_transcription`.

Prereqs: main ESP32 at `192.168.8.50` (mic + hexapod) and satellite ESP32 at
`192.168.8.51` (camera) both online; ROS 2 host at `192.168.8.134`.

**T1 — micro-ROS agent** (wait for `[create_session]` from .50 *and* .51):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
```

**T2 — full autonomy stack** (robot_state_publisher, flipped cam_proxy, EKF,
YOLO vision_node, SLAM, Nav2, `object_seeker` in WANDER):
```bash
docker exec -it -e DISPLAY=:0 $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_navigation real_object_seek.launch.py"
```

**T3 — voice brain** (Gemini intent parser; subscribes to `/audio_transcription`,
sends `SeekObject` goals):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run seeker_voice command_node"
```

**T4 — whisper bypass** (type commands like `hey hatsune find the ball over`):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && python3 /home/ubuntu/scripts/voice_terminal.py"
```

**T4-alt — real whisper pipeline (ESP32 mic)**: replaces T4 when you want
actual speech → text via faster-whisper instead of typed stdin. Pulls audio
from `MicSubsystem`'s HTTP endpoint on the main ESP32 (`192.168.8.50:81/audio`),
runs on-device transcription, publishes to `/audio_transcription` — which
`command_node` (T3) already listens on.
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_voice esp32_mic.launch.py"
```
Override host/port if the mic lives elsewhere:
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_voice esp32_mic.launch.py esp32_ip:=192.168.8.50 esp32_port:=81"
```
Host-mic variant (laptop mic via `sounddevice`, also auto-starts command_node so
skip T3 if using this):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_voice local_mic.launch.py"
```
Choose exactly one of T4 / T4-alt at a time — both publishing to
`/audio_transcription` will interleave and confuse `command_node`.

**T5 — TTS node** (Fish Audio → PCM served on `:8383/audio_out` for ESP32
`SpeakerSubsystem` to fetch; also handles `/audio_play_file` for local WAVs):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_tts tts.launch.py"
```
Requires `FISH_API_KEY` and `FISH_REFERENCE_ID` set in the container env
(already wired in `.env`). Without T5 the ESP32 speaker stays silent even
though `command_node` is publishing to `/audio_tts_input`.

### Manual seek / dance (bypass the voice brain)
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run seeker_navigation find sports_ball --feedback"
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 service call /perform_move mcu_msgs/srv/PerformMove \"{move_name: 'dance', duration_sec: 3.0}\""
```

### cmd_vel clamp overrides (defence-in-depth; MCU already caps)
Defaults: `max_linear_x=0.1` m/s, `max_linear_y=0.05` m/s, `max_angular_z_rad=0.20` (~11.5 deg/s).
Override per-launch by appending `-p` args to the `object_seeker` / `ball_searcher` nodes, or via a params file passed to the launch.

---

## 5. Real Hardware — Lidar-Only Stack (no gyro / no EKF)

Minimal stack for validating SLAM + driving without any IMU involvement. Uses a
static identity `odom → base_footprint` TF and forces SLAM Toolbox into pure
scan-matching mode. No Nav2, no object_seeker, no autonomous behaviour — you
drive manually with `cmd_vel`. Use this to isolate whether SLAM/lidar issues
persist when the gyro/EKF path is removed entirely.

Prereqs: main ESP32 at `192.168.8.50` (hexapod) online; LiDAR spinning. Camera
satellite at `.51` is *not* required.

**T1 — micro-ROS agent** (wait for `[create_session]` from .50):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
```

**T2 — lidar-only SLAM stack** (robot_state_publisher, static `odom` TF,
`scan_tilt_filter`, SLAM Toolbox, RViz — no EKF, no Nav2):
```bash
docker exec -it -e DISPLAY=:0 $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_navigation real_slam_raw.launch.py"
```

**T3 — manual drive** (there's no Nav2, so you publish `/cmd_vel` yourself).
Either keyboard teleop:
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```
Or one-shot publishes from another terminal:
```bash
# rotate in place at ~30 deg/s
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{angular: {z: 0.524}}\""

# forward at 0.1 m/s
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.1}}\""
```
Kill the publish (Ctrl-C) to stop — the new MCU firmware's watchdog will
auto-disable the gait on command silence, but older firmware will keep
walking until an explicit zero cmd arrives.

**T4 (optional) — whisper sanity check**: run the transcription pipeline
standalone to verify the mic → faster-whisper path is working. There's no
`command_node` in this stack, so the resulting `/audio_transcription`
messages just sit on the topic — useful only for validating the audio chain
itself. Pick whichever mic source is hooked up:
```bash
# ESP32 mic (via MicSubsystem HTTP stream on 192.168.8.50:81/audio)
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_voice esp32_mic.launch.py"

# OR laptop mic (sounddevice)
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_voice local_mic.launch.py"
```
Then watch the output from another terminal:
```bash
docker exec $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic echo /audio_transcription"
```
Speak — transcribed strings should appear. No behaviour is triggered;
voice-driven autonomy lives in §4's stack.

### Verification (after T1 + T2)
```bash
# LiDAR reaching the host
docker exec $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic hz /mcu/scan"             # expect ~10 Hz

# Filtered scan (SLAM consumes this)
docker exec $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic hz /mcu/scan_filtered"    # same rate as /mcu/scan

# SLAM producing a map
docker exec $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 topic hz /map"                  # expect ~0.2 Hz (map_update_interval=5s)

# TF chain sanity — should print without errors
docker exec $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run tf2_ros tf2_echo map base_footprint"
```

### When to use §5 vs §4
- `/real_slam_raw` (this section) — SLAM debug path. No EKF means no roll/pitch
  compensation; good for ruling in/out the gyro as the cause of map artefacts.
- `/real_object_seek` (§4) — production autonomy path, requires a correctly
  configured gyro/EKF.

If the map builds cleanly here but replicates walls under §4, the fault is in
the EKF/gyro path (`GyroSubsystem.cpp` reorientation, `ekf_params.yaml`
self-referential `odom0`). If walls replicate in both, the issue is
upstream — URDF `lidar_joint` orientation or scan matcher tuning.

---

## 6. Real Hardware — Full Brain–Body Pipeline WITHOUT Gyro

Same autonomy stack as §4 (agent + SLAM + Nav2 + YOLO + `object_seeker` +
`command_node` + TTS + whisper) but with the EKF replaced by
`dead_reckoning_odom`, which open-loop-integrates `/cmd_vel` into
`odom → base_footprint`. No IMU is consumed anywhere. SLAM's `map → odom`
correction is the only thing keeping the robot localised — expect drift
recovery to be more visible than under §4.

Use when you want the full voice-to-behaviour pipeline but want to prove
the gyro/EKF path is the one responsible for an observed issue.

Prereqs: same as §4 (main ESP32 at `.50`, camera satellite at `.51`,
ROS 2 host at `.134`).

**T1 — micro-ROS agent** (same as §4):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
```

**T2 — full autonomy stack WITHOUT gyro** (dead_reckoning_odom replaces
the EKF; scan_tilt_filter still runs but with roll/pitch=0, so it's a
pass-through):
```bash
docker exec -it -e DISPLAY=:0 $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_navigation real_object_seek_no_gyro.launch.py"
```

**T3 — voice brain**, **T4 / T4-alt — whisper**, **T5 — TTS**: identical to
§4. Use the same commands from that section.

### Manual seek / dance, cmd_vel clamp overrides
Same as §4 — the commands target `object_seeker` / Nav2 / gait which are
unchanged. Only the odometry source has been swapped.

### When to use §6 vs §4
- `real_object_seek` (§4) — production autonomy with IMU-fused EKF. Best
  performance when the gyro is correctly reoriented.
- `real_object_seek_no_gyro` (§6, this section) — autonomy without any IMU
  involvement. Useful as an A/B test against §4: if SLAM map artefacts
  disappear under §6 but come back under §4, the EKF/gyro path is the
  cause. If they persist under §6, the issue is downstream (URDF lidar
  alignment, scan matcher tuning, etc.).
- `real_slam_raw` (§5) — SLAM-only, no Nav2, no autonomy. Fastest to
  bring up for pure scan-matching validation.

---

## 7. Real Hardware — Scripted /cmd_vel Drive ("what-if" replay)

Runs a YAML-defined sequence of `forward` / `backward` / `turn` / `wait`
steps on `/cmd_vel` while SLAM maps the robot's path. Forward/backward
legs can be time-based (`duration`) or odom-closed-loop distance-based
(`inches` / `meters`); turns always close the loop on `/odom` yaw.

**Why a dedicated launch.** Running `scripted_cmd_vel` alongside
`real_object_seek_no_gyro.launch.py` would put two publishers on `/cmd_vel`
(the Nav2 → `velocity_smoother` → `cmd_vel_restrict` chain, plus the
script). Use this launch instead — it reuses the same `robot_state_publisher`
+ `dead_reckoning_odom` + `scan_tilt_filter` + SLAM Toolbox plumbing, but
drops Nav2, `velocity_smoother`, `cmd_vel_restrict`, `object_seeker`, and
the vision stack, so the script is the sole `/cmd_vel` publisher.

Prereqs: main ESP32 at `192.168.8.50` online, LiDAR spinning. Camera
satellite at `.51` is **not** required.

**T1 — micro-ROS agent** (same as §5/§6):
```bash
docker exec -it $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
```

**T2 — scripted drive stack** (dead-reckoning odom + SLAM + RViz +
`scripted_cmd_vel`; script starts at t=15s after SLAM is active):
```bash
docker exec -it -e DISPLAY=:0 $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && ros2 launch seeker_navigation real_scripted_drive.launch.py"
```

Override args individually:
```bash
docker exec -it -e DISPLAY=:0 $CTR bash -c "source /home/ubuntu/ros2_workspaces/install/setup.bash && \
  ros2 launch seeker_navigation real_scripted_drive.launch.py \
    script_path:=/home/ubuntu/ros2_workspaces/install/seeker_navigation/share/seeker_navigation/config/scripted_cmd_vel_example.yaml \
    scale:=1.0 linear_speed:=0.10 angular_speed:=0.20"
```

### Script YAML format
Default script path: `install/seeker_navigation/share/seeker_navigation/config/scripted_cmd_vel_example.yaml`.

```yaml
steps:
  - {type: turn,    angle_deg: 360.0}   # CCW spin (positive = CCW, negative = CW)
  - {type: forward, inches:    26.0}    # odom-closed-loop distance
  - {type: forward, meters:     0.5}    # same, in meters
  - {type: forward, duration:   3.0}    # open-loop time
  - {type: backward, inches:    6.0, speed: 0.08}   # per-step speed override
  - {type: wait,     duration:  1.0}
```

Point `script_path` at any file on disk; no rebuild required as long as
the YAML is outside the install tree. For files under
`ros2_ws/src/seeker_navigation/config/`, do a `colcon build
--packages-select seeker_navigation` to sync them into
`install/.../share/.../config/`.

### Global `scale` param
Multiplies every `duration` / `inches` / `meters` at runtime — set
`scale:=0.5` to replay the same script at half size, or `scale:=2.0` to
double it. Turns are unaffected.

### Mirror it in simulation
To preview the run before committing to the real robot, start Gazebo
(§1) separately and pass the same `script_path` to `scripted_cmd_vel`
inside the sim — it publishes on `/cmd_vel` either way. Don't run §7 and
§1 against the same `/cmd_vel`; use separate ROS domain IDs or swap the
`cmd_vel_topic` param on one side.

---

## 8. Troubleshooting

### Kill everything
Nukes all ROS 2 nodes, Nav2 servers, SLAM, RViz, Gazebo, micro-ROS agent,
whisper/TTS/voice nodes, and launched `docker exec`-spawned scripts. Use
when `Ctrl-C` leaves zombies or nodes restart themselves. `ros2 daemon` may
still respawn — `ros2 daemon stop` if you want it gone too.
```bash
pkill -9 -f "ros2|rviz2|gzserver|gzclient|gz sim|/opt/ros/jazzy/lib|ros2_workspaces/install|micro_ros_agent|voice_terminal|cam_proxy"
ros2 daemon stop
```
What those substrings catch:
- `ros2` — every `ros2 run/launch/topic/action/service` CLI invocation
- `rviz2`, `gzserver|gzclient|gz sim` — RViz + Gazebo
- `/opt/ros/jazzy/lib` — all stock ROS 2 binaries (nav2, slam_toolbox, ekf, robot_state_publisher, static_transform_publisher, teleop_twist, etc.)
- `ros2_workspaces/install` — every workspace-built node (object_seeker, ball_searcher, dead_reckoning_odom, scan_tilt_filter, vision_node, cam_proxy, transcription_node, command_node, tts_node, etc.)
- `micro_ros_agent` — the agent
- `voice_terminal` — the whisper-bypass stdin script
- `cam_proxy` — the MJPEG bridge (ExecuteProcess-spawned)

Inside the container if `docker exec`'s ran out:
```bash
docker exec -it $CTR bash -c "pkill -9 -f 'ros2|rviz2|gzserver|gzclient|/opt/ros/jazzy/lib|ros2_workspaces/install|micro_ros_agent|voice_terminal|cam_proxy' ; ros2 daemon stop"
```

### Verify nothing is left
```bash
pgrep -af "ros2|rviz2|/opt/ros/jazzy/lib|micro_ros_agent" || echo "clean"
```

---
*Note: Ensure the real YOLO weights are downloaded to `install/seeker_vision/share/seeker_vision/model/yolo26n.pt` for object detection to work.*
