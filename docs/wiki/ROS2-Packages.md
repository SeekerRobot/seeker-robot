# ROS 2 Packages

Everything under `ros2_ws/src/` is a standard ROS 2 Jazzy package. The workspace builds with `colcon build` from `~/ros2_workspaces` inside the dev container. Artifacts are written to the named Docker volumes (`ros2_build`, `ros2_install`, `ros2_log`) rather than to the host.

```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build                                          # build everything
colcon build --packages-select <pkg>                  # one package
colcon build --packages-up-to <pkg>                   # pkg + its upstream deps
colcon build --symlink-install --packages-select <pkg> # symlink so Python edits apply without rebuild
source install/setup.bash
```

`--symlink-install` is especially handy for the Python packages (`seeker_display`, `seeker_media`, `seeker_navigation`, `seeker_sim`, `seeker_tts`, `seeker_vision`) because you don't need to rebuild for every edit.

---

## `mcu_msgs`

**Build type:** `ament_cmake` (message codegen only — no nodes)

Shared ROS 2 ↔ micro-ROS interface package. Defines the `.msg`/`.srv` files that both the ROS 2 side and the ESP32 firmware see.

**Messages** (`msg/`):

- `HexapodCmd.msg` — gait mode (`STAND` / `WALK` / `SIT` / `DANCE`) plus body pose (height, pitch, roll). Published to `/mcu/hexapod_cmd` by the mission planner or the vision node (DANCE on target detection).
- `OledFrame.msg` — raw 1024-byte SSD1306 (128×64, page-major, 8 pages) framebuffer for the onboard OLED. Used by `seeker_display` and `seeker_media` to generate frames; the ESP32 receives them via plain HTTP (`GET /lcd_out` on port 8384), not via micro-ROS.
- `ExampleMsg.msg` — scaffolding example.

**Services** (`srv/`):

- `ExampleSrv.srv` — scaffolding example.

Any change here **must** be rebuilt in both workspaces — see [Architecture → mcu_msgs flow](Architecture.md#how-mcu_msgs-flows-between-workspaces).

```bash
colcon build --packages-select mcu_msgs
```

---

## `seeker_display`

**Build type:** `ament_python`

OLED display nodes and the shared HTTP LCD server used by `seeker_media`. The ESP32 `OledSubsystem` connects as an HTTP client to `GET /lcd_out` on port 8384 and reads 1024-byte SSD1306 framebuffers continuously — no micro-ROS agent required.

**Nodes:**

- `oled_sine` (`seeker_display/oled_sine_node.py`) — generates an animated sine wave at 10 Hz and serves it via the LCD HTTP server. Good for verifying the OLED HTTP pipeline end-to-end.

**Modules:**

- `lcd_http_server` (`seeker_display/lcd_http_server.py`) — shared helper: starts a daemon-thread HTTP server on `0.0.0.0:<port>` serving `GET /lcd_out` as a persistent raw byte stream of 1024-byte framebuffers. Used by both `oled_sine_node` and `seeker_media/mp4_player_node`.

**Parameters** (on `oled_sine`):

| Parameter | Default | Notes |
|---|---|---|
| `lcd_serve_port` | `8384` | HTTP port for the LCD stream |

```bash
colcon build --packages-select seeker_display
ros2 run seeker_display oled_sine
```

---

## `seeker_description`

**Build type:** `ament_cmake`

URDF/Xacro model of the hexapod. Consumed by every launch file that needs `robot_state_publisher`, Gazebo spawning, or an RViz robot model.

**Assets:**

- `urdf/seeker_hexapod.urdf.xacro` — 6 legs × 2 DOF (hip yaw + knee pitch) plus fixed frames for LiDAR, camera, and IMU.
- `rviz/seeker.rviz` — starter RViz config for visualising the model.

**Launch files:**

- `launch/display.launch.py` — processes the xacro, starts `robot_state_publisher`, and launches RViz with `seeker.rviz`. Useful smoke test: if the robot shows up in RViz at `Fixed Frame: base_link`, your URDF and X server are both working.

```bash
colcon build --packages-select seeker_description
ros2 launch seeker_description display.launch.py
```

---

## `seeker_gazebo`

**Build type:** `ament_cmake`

All Gazebo Harmonic simulation launches, plus the small helpers that bridge Gazebo topics back to ROS 2. Depends on `seeker_description` (for the URDF) and `seeker_sim` (for the fake MCU gait).

**Worlds:**

- `worlds/slam_test.sdf` — walled test environment for SLAM / Nav2.

**Config:**

- `config/bridge.yaml` — `ros_gz_bridge` topic/service mapping (joint commands, sensor streams).
- `config/slam_toolbox_params.yaml` — SLAM parameters tuned for the sim.

**Scripts:**

- `scripts/gz_odom_bridge.py` — publishes the `odom → base_footprint` TF from Gazebo's ground-truth odometry. Used by the `sim_slam_raw` launch (no EKF).

**RViz configs:**

- `rviz/teleop.rviz` — minimal teleop view.
- `rviz/slam.rviz` — SLAM / mapping view.

**Launch files** (see **[Simulation](Simulation.md)** for what each one actually does):

| Launch | Stack it brings up | RViz fixed frame |
|---|---|---|
| `sim_teleop.launch.py` | Gazebo + robot_state_publisher + fake_mcu_node + odom bridge | `odom` |
| `sim_slam_raw.launch.py` | `sim_teleop` + SLAM Toolbox (ground-truth odom, no EKF) | `odom` |
| `sim_slam_ekf.launch.py` | `sim_teleop` + EKF (IMU tilt) + SLAM Toolbox | `odom` |
| `sim_ball_search.launch.py` | `sim_slam_ekf` + Nav2 + `ball_searcher` | `map` |

```bash
colcon build --packages-select mcu_msgs seeker_description seeker_sim seeker_gazebo
ros2 launch seeker_gazebo sim_teleop.launch.py
```

---

## `seeker_media`

**Build type:** `ament_python`

MP4 media player node. Decodes video to 128×64 SSD1306 framebuffers served over HTTP and audio to 16 kHz PCM streamed to the ESP32 speaker, with A/V sync. Depends on `seeker_display` (uses `lcd_http_server`).

**Nodes:**

- `mp4_player` (`seeker_media/mp4_player_node.py`) — subscribes to `/media/play` (`std_msgs/String`, absolute file path) to trigger playback and `/media/stop` (`std_msgs/Empty`) to abort. Video frames are dithered to 1-bit and served via the LCD HTTP server on port 8384. Audio is extracted via `ffmpeg`, optionally EQ'd with a low-shelf filter, and served on port 8383 (same port as `seeker_tts` — don't run both simultaneously). The video pipeline blocks until the ESP32 audio client connects so the first frame and first audio byte arrive together.

**Parameters:**

| Parameter | Default | Notes |
|---|---|---|
| `serve_port` | `8383` | HTTP port for audio (shared with `seeker_tts`) |
| `lcd_serve_port` | `8384` | HTTP port for OLED LCD stream |
| `threshold` | `127` | Greyscale → 1-bit cutoff (0–255) |
| `audio_lead_ms` | `0` | ms to delay video after audio connects (compensates for I2S DMA latency) |
| `eq_bass_hz` | `300.0` | Low-shelf EQ corner frequency |
| `eq_bass_db` | `0.0` | Low-shelf EQ gain (negative = cut bass) |

**Launch:**

- `launch/media.launch.py` — starts `mp4_player` with `audio_lead_ms=200`.

**Video files** (`video/`):

- `badapple.mp4`, `badapple.mov` — demo media (stored via Git LFS).

```bash
colcon build --packages-select seeker_display seeker_media
ros2 launch seeker_media media.launch.py
# in another terminal:
ros2 topic pub /media/play std_msgs/String "data: '/path/to/video.mp4'" --once
```

---

## `seeker_navigation`

**Build type:** `ament_python`

All autonomy configs and launches plus the `ball_searcher` mission planner. Depends on `seeker_description`.

**Nodes:**

- `ball_searcher` (`seeker_navigation/ball_searcher.py`) — Nav2 client. State machine:
  1. Rotate 360° in place to seed the SLAM map.
  2. Frontier-explore: read `/map`, pick a free/unknown boundary, send `NavigateToPose`.
  3. Fall back to a coverage grid if no frontiers remain.
  4. Cancel exploration and approach as soon as an HSV-thresholded red ball appears in the camera feed.

**Config:**

- `config/ekf_params.yaml` — `robot_localization` parameters fusing `/mcu/imu` into `odom → base_footprint`.
- `config/nav2_params.yaml` — Nav2 tuning for simulation.
- `config/nav2_params_real.yaml` — Nav2 tuning for real hardware.
- `config/slam_toolbox_params_real.yaml` — SLAM Toolbox tuning for real hardware.

**RViz configs:**

- `rviz/nav2.rviz` — map, costmaps, plan paths.
- `rviz/slam.rviz` — SLAM-focused view.

**Launch files:**

| Launch | Purpose |
|---|---|
| `real_slam_raw.launch.py` | SLAM on real hardware **without** IMU fusion (static `odom → base_footprint`) — simplest bring-up |
| `real_slam_ekf.launch.py` | SLAM on real hardware **with** EKF IMU tilt compensation |
| `real_ball_search.launch.py` | Full autonomy: EKF + SLAM + Nav2 + `ball_searcher` |

Launch timing for `real_ball_search.launch.py`:

```
t=0s   robot_state_publisher
t=2s   EKF
t=4s   SLAM Toolbox
t=10s  slam_toolbox lifecycle configure → (2s gap) → activate
t=13s  Nav2 nodes (controller, planner, behavior, bt_navigator, velocity_smoother)
t=15s  Nav2 lifecycle manager
t=25s  ball_searcher
```

```bash
colcon build --packages-select mcu_msgs seeker_description seeker_navigation
ros2 launch seeker_navigation real_slam_ekf.launch.py
```

---

## `seeker_sim`

**Build type:** `ament_python`

Stand-in for the ESP32 MCU when running in Gazebo. Gets pulled in automatically by every `seeker_gazebo` launch.

**Nodes:**

- `fake_mcu_node` (`seeker_sim/fake_mcu_node.py`) — subscribes to `/cmd_vel` (`geometry_msgs/Twist`), runs a joint-space tripod gait (no inverse kinematics — locomotion comes from sweeping the hip). At ~100 Hz it publishes:
  - `/mcu/joint_states` — 12-joint `sensor_msgs/JointState` (hip + knee × 6 legs)
  - Per-joint `std_msgs/Float64` position commands to the 12 `/seeker/joint/leg_*_{hip,knee}/cmd_pos` topics that the `ros_gz_bridge` forwards to Gazebo's `JointPositionController`.

It does **not** publish `/mcu/imu` or `/mcu/scan` — those are bridged from Gazebo sensor plugins in `seeker_gazebo/config/bridge.yaml`. Body height is adjustable at runtime via `linear.z` (the `t` / `b` keys in `teleop_twist_keyboard`), which walks `NEUTRAL_KNEE` within safe limits.

Tuning constants at the top of the file:

| Constant | Default | Effect |
|---|---|---|
| `CYCLE_TIME` | 0.8 s | Full tripod cycle duration |
| `STRIDE_HIP` | 45° | Max hip sweep amplitude per side |
| `NEUTRAL_KNEE` | 30° | Standing knee angle (body height) |
| `LIFT_KNEE` | 88° | Peak knee angle during swing (foot clearance) |
| `VX_MAX` | 0.4 m/s | Speed that maps to full `STRIDE_HIP` |
| `WZ_MAX` | 1.2 rad/s | Yaw rate that maps to half `STRIDE_HIP` turn amplitude |

```bash
colcon build --packages-select seeker_sim
```

You should not normally run this node standalone — use a `seeker_gazebo` launch instead.

---

## `seeker_tts`

**Build type:** `ament_python`

Audio bridge for the ESP32 speaker. Supports two input modes — Fish Audio TTS and local WAV file playback — and re-serves both as a persistent chunked HTTP PCM stream that the ESP32 `SpeakerSubsystem` long-polls.

**Nodes:**

- `tts_node` (`seeker_tts/tts_node.py`) — maintains an internal single-slot queue of PCM audio clips, drained by an HTTP server on `http://<host>:<serve_port>/audio_out`. Subscribes to:
  - `/audio_tts_input` (`std_msgs/String`) — text gets POSTed to `https://api.fish.audio/v1/tts` with `format=pcm` and the returned PCM bytes land on the queue. Has priority over file playback (TTS blocks the executor during the API call).
  - `/audio_play_file` (`std_msgs/String`) — value is a path to a `.wav` file on the host; the file is decoded with `wave` and enqueued as PCM. Drops if the queue is full (TTS has priority).

**Parameters** (declared in the node, overridable via the launch file):

| Parameter | Default | Notes |
|---|---|---|
| `fish_api_key` | `""` | Falls back to the `FISH_API_KEY` env var. |
| `fish_reference_id` | `""` | Falls back to the `FISH_REFERENCE_ID` env var. |
| `fish_model` | `"s2-pro"` | Fish Audio model name. |
| `serve_port` | `8383` | HTTP port for the `/audio_out` endpoint. |
| `sample_rate` | `16000` | Passed to the Fish Audio request. |
| `eq_bass_hz` | `300.0` | Low-shelf EQ corner frequency. |
| `eq_bass_db` | `0.0` | Low-shelf EQ gain (negative = cut bass). |

**Launch:**

- `launch/tts.launch.py` — starts `tts_node` with `sample_rate` and `fish_model` overrides.

**Bundled sounds** (`sounds/`):

- `intro.wav` — startup jingle. Play with: `ros2 topic pub /audio_play_file std_msgs/String "data: '<install_path>/sounds/intro.wav'" --once`

**Environment variables** (set in `docker/.env`):

- `FISH_API_KEY` — Fish Audio API key.
- `FISH_REFERENCE_ID` — voice reference ID.

```bash
colcon build --packages-select seeker_tts
ros2 launch seeker_tts tts.launch.py
```

The ESP32-side consumer is the `test_sub_speaker` sketch — see **[MCU Sketches](MCU-Sketches.md)**.

---

## `seeker_vision`

**Build type:** `ament_python`

Camera-based perception: YOLO object detection, DeepFace emotion recognition, and an MJPEG camera proxy that bridges the ESP32's HTTP camera stream to `localhost` so OpenCV can consume it from inside the container. The Dockerfile's `base` stage pre-installs `ultralytics`, `deepface`, and `tensorflow[and-cuda]`, so the package works out of the box on GPU-equipped hosts.

**Nodes:**

- `vision_node` (`seeker_vision/vision_core.py`) — opens an MJPEG or V4L2 video source, runs YOLOv8-nano (`yolo26n.pt`, bundled) at ~30 fps, and publishes:
  - `/object_found` (`std_msgs/Bool`) — `true` when the target object (default: `"teddy bear"`) is in frame.
  - `/detection_detail` (`std_msgs/Float32MultiArray`) — `[area, center_x, frame_width]` for the target bounding box (zeros when not found).
  - `/mcu/hexapod_cmd` (`mcu_msgs/HexapodCmd`) — sends `MODE_DANCE` when the target is detected.
- `gazebo_vision_node` (`seeker_vision/gazebo_vision_core.py`) — same detection pipeline, but subscribes to `/camera/image` (`sensor_msgs/Image`) instead of opening an HTTP stream. Use this in Gazebo simulation.
- `cam_proxy` (`seeker_vision/cam_proxy.py`) — standalone MJPEG proxy. Fetches the ESP32 camera stream (default `http://192.168.8.50/cam`) and re-serves it at `http://localhost:8080/stream`. Needed because the container may not have a direct route to the ESP32's IP.
- `emotion_node` (`seeker_vision/emotion_node.py`) — Haar-cascade face localisation + DeepFace emotion analysis at ~10 fps. Publishes the dominant emotion to `/emotion_detail` (`std_msgs/String`).

**Parameters** (on `vision_node` / `emotion_node`):

| Parameter | Default | Notes |
|---|---|---|
| `video_source` | `http://localhost:8080/stream` | MJPEG URL or V4L2 device path (e.g. `/dev/video0`) |

**Launch files:**

| Launch | What it starts | Camera source |
|---|---|---|
| `mcu_cam.launch.py` | `cam_proxy` + `vision_node` (2 s delay) | ESP32 camera via proxy |
| `gazebo_cam.launch.py` | `gazebo_vision_node` | Gazebo `/camera/image` topic |
| `local_cam.launch.py` | `vision_node` | Host webcam (`/dev/video0`) |

**Model:**

- `model/yolo26n.pt` — YOLOv8-nano weights (stored via Git LFS). Detects 80 COCO classes; the node filters for a configurable target name.

```bash
colcon build --packages-select mcu_msgs seeker_vision
ros2 launch seeker_vision mcu_cam.launch.py       # real ESP32 camera
ros2 launch seeker_vision gazebo_cam.launch.py     # Gazebo simulation
ros2 launch seeker_vision local_cam.launch.py      # host webcam
```

---

## `test_package`

**Build type:** `ament_cmake`

Minimal C++ sanity package for CI verification. Contains a single `test_node` executable that logs `"Workflow is running..."` once a second and exits, plus a `test_node.launch.py`. Safe to ignore unless you're debugging the build pipeline.

```bash
colcon build --packages-select test_package
ros2 launch test_package test_node.launch.py
```

---

## Inter-package dependency graph

```
seeker_gazebo ──► seeker_description
seeker_gazebo ──► seeker_sim
seeker_media  ──► seeker_display
seeker_navigation ──► seeker_description
seeker_vision ──► mcu_msgs
every ROS node importing HexapodCmd ──► mcu_msgs
```

Everything else is self-contained. If you just want to build the navigation stack without touching Gazebo:

```bash
colcon build --packages-up-to seeker_navigation
```
