# MCU Sketches

Every directory under `mcu_ws/src/` is a standalone PlatformIO project. They follow a naming convention:

- `main` — full integration firmware (default env offloads camera to `main_satellite`)
- `main_add` — incremental modular rebuild of `main` (phases 1–6)
- `main_satellite` — camera/sensor offload board for dual-board architecture
- `build_microros` — placeholder sketch used only to pre-build the micro-ROS library
- `test_all` — **full** integration: micro-ROS bridge + camera + mic + speaker + OLED, all concurrently
- `test_bridge_*` — exercises the full micro-ROS stack via WiFi (sensor publishers, gait subscription, or OLED HTTP display)
- `test_sub_*` — single subsystem in isolation (mostly serial-only, no micro-ROS)
- `test_raw_*` — low-level hardware tests with no subsystem abstraction
- `test_threaded_blink` — `ThreadedSubsystem` / FreeRTOS task smoke test
- `test_fast_led_raw` — `FastLED` blink without the `LedSubsystem` abstraction

All commands below run inside the dev container from the sketch directory, e.g.:

```bash
docker compose exec ros2 bash
cd ~/mcu_workspaces/seeker_mcu/src/<sketch>
pio run -e <env> -t upload
pio device monitor -b 921600
```

Monitor baud is always `921600` (set globally in the base `platformio.ini`).

---

## Integration sketches

### `test_all`

- **Board env:** `esp32s3sense_psram` (pioarduino 54.03.21)
- **Transport:** WiFi (micro-ROS) + HTTP (camera :80, mic :81, speaker from :8383, OLED from :8390)
- **Bridge flags:** `HEARTBEAT, GYRO, BATTERY, LIDAR, DEBUG`
- **Purpose:** The "everything on" integration test. Runs the full `MicroRosBridge` plus a concurrent MJPEG camera server, PDM audio HTTP server, I2S speaker (fetches PCM from host :8383), and OLED display (fetches framebuffers from host :8390) on a single ESP32-S3 Sense. Also includes a PBUF watchdog (warns at <8 KB DMA or <20 KB internal free), stack high-water mark tracking (logged every 10 s), and per-loop heap telemetry. Gait subscribes to `/cmd_vel` via `GaitRosParticipant` with velocity caps loaded from NVS.
- **Prereq:** micro-ROS agent running (`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`).
- **Build/flash:** `pio run -t upload` (default env is `esp32s3sense_psram`)
- **Serial output:** WiFi connection status, agent state transitions, IP + RSSI, heap snapshots every 1 s, task stack high-water marks every 10 s.
- **Verify on ROS side:**
  ```bash
  ros2 topic hz /mcu/heartbeat        # 1 Hz
  ros2 topic hz /mcu/imu              # ~50 Hz
  ros2 topic hz /mcu/scan             # ~6 Hz
  ros2 topic hz /mcu/battery_voltage  # 1 Hz
  curl http://<esp_ip>/cam            # MJPEG
  curl http://<esp_ip>:81/audio       # raw PCM
  ```
- **OLED (no micro_ros_agent needed):**
  ```bash
  ros2 run seeker_display oled_sine   # animated sine wave demo
  # or: ros2 launch seeker_media media.launch.py
  ```
- **Speaker / TTS:**
  ```bash
  ros2 launch seeker_tts tts.launch.py
  ros2 topic pub /audio_tts_input std_msgs/String "data: 'hello'" --once
  ros2 topic pub /audio_play_file std_msgs/String "data: '/path/to/sound.wav'" --once
  ```
- **Debug tips:** (1) If topics appear but the camera 404s, PSRAM failed to init — check serial. (2) If the whole thing boots but micro-ROS never connects, double-check `agent_ip` in `network_config.ini`. (3) RSSI < −75 dBm reliably breaks micro-ROS — move closer to the AP. (4) `min_spiffs.csv` partition can run tight — watch `RAM: %` after build. (5) Use `pio device monitor --filter esp32_exception_decoder` to get backtraces on panic.

### `test_bridge_all`

- **Board env:** `esp32s3sense`
- **Transport:** WiFi (micro-ROS) + HTTP (speaker from :8383, OLED from :8390)
- **Bridge flags:** `HEARTBEAT, GYRO, BATTERY, LIDAR, DEBUG`
- **Purpose:** Like `test_all` but **without** the camera and mic HTTP servers. micro-ROS sensor bridge + I2S speaker + OLED HTTP display — ideal for SLAM, Nav2, or anything that doesn't need the camera feed.
- **Prereq:** micro-ROS agent running.
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Verify on ROS side:**
  ```bash
  ros2 topic hz /mcu/{heartbeat,imu,battery_voltage,scan}
  ```
- **OLED (no micro_ros_agent needed):**
  ```bash
  ros2 run seeker_display oled_sine   # animated sine wave demo
  # or: ros2 launch seeker_media media.launch.py
  ```
- **Speaker / TTS:**
  ```bash
  ros2 launch seeker_tts tts.launch.py
  ros2 topic pub /audio_tts_input std_msgs/String "data: 'hello'" --once
  ```
- **Debug tips:** Same as `test_all` minus the camera/mic issues. If `/mcu/scan` publishes 0 points, the LD14P is probably unpowered or the UART pins in `RobotConfig.h` don't match your wiring. If the OLED never updates, make sure the host is serving on port 8390 (`ros2 run seeker_display oled_sine`) and that the ESP32 can reach `AGENT_IP:8390`.

### `test_bridge_gait`

- **Board env:** `esp32s3sense`
- **Transport:** WiFi
- **Bridge flags:** none (uses a dedicated `GaitRosParticipant`)
- **Purpose:** Gait + locomotion only. Subscribes to `/cmd_vel` (`geometry_msgs/Twist`) and drives the hexapod via `GaitController` + `ServoSubsystem`. No sensor publishing. Use this for teleop or Nav2 output tests on hardware without the extra bandwidth of the sensor bridge.
- **Prereq:** micro-ROS agent running, servos powered.
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Verify on ROS side:** `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05}}"` — robot should walk.
- **Debug tips:** (1) If servos twitch but don't sync, the PCA9685 frequency is off — check `test_sub_servo freq`. (2) If the robot falls over, the neutral gait pose may need retuning — use `test_sub_gait` first. (3) No `/cmd_vel` response means the subscription was never created — watch the serial log for the `create_session` line. (4) Always power cycle servos before re-flashing; a stuck PWM can kill the MCU's power rail. (5) You can send commands with `teleop_twist_keyboard` for interactive driving.

### `test_bridge_oled`

- **Board env:** `esp32s3sense`
- **Transport:** WiFi (HTTP only — **no micro-ROS agent required**)
- **Bridge flags:** none
- **Purpose:** Isolated WiFi + OLED HTTP streaming test. The ESP32 connects to WiFi, then opens a persistent HTTP connection to the host at `AGENT_IP:8390/lcd_out` and reads 1024-byte SSD1306 framebuffers continuously. The `OledSubsystem` renders them at ~10 Hz. Text overlays show the boot banner and WiFi connection state. No `MicroRosBridge` is involved — the OLED data flows over plain HTTP, independent of DDS.
- **Prereq:** WiFi configured in `network_config.ini`; SSD1306 128×64 wired on the shared I²C bus (`Config::sda`/`Config::scl`, default address `0x3C`); a host-side LCD server running.
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Verify on host (pick one):**
  ```bash
  ros2 run seeker_display oled_sine    # animated sine wave
  ros2 launch seeker_media media.launch.py  # MP4 video playback
  ```
- **Debug tips:** (1) Display blank → confirm `AGENT_IP` in `network_config.ini` and that the host LCD server is running on port 8390. (2) Display shows boot frame but never updates → WiFi may not be connected yet; check serial output. (3) Updates cap at 10 Hz — this is by design. (4) If text overlays don't show, remember the HTTP framebuffer push overrides them until the next overlay update tick.

---

## Subsystem tests (`test_sub_*`)

### `test_sub_heartbeat`

- **Board env:** `esp32s3sense` / **Transport:** WiFi
- **Purpose:** Minimal micro-ROS smoke test — registers a `HeartbeatParticipant` that publishes a monotonic counter at 1 Hz on `/mcu/heartbeat`. Use this to validate WiFi + agent connectivity end-to-end before layering anything else on.
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Verify:** `ros2 topic echo /mcu/heartbeat`
- **Debug tips:** (1) Check serial for WiFi connect success, (2) check agent logs for `create_session`, (3) make sure `agent_ip` in `network_config.ini` is your host's IP on the same subnet.

### `test_sub_wifi`

- **Board env:** `esp32s3sense` / **Transport:** WiFi (no micro-ROS)
- **Purpose:** Bare-bones WiFi connectivity test via `ESP32WifiSubsystem`. Prints SSID, IP, RSSI every second. Use it when a fresh board refuses to connect and you need to rule out the higher-level micro-ROS stack.
- **Serial output:** `WiFi: CONNECTED | SSID: <name> | IP: <addr> | RSSI: -<n> dBm`
- **Debug tips:** (1) If it never connects, check `wifi_ssid`/`wifi_password` in `network_config.ini`. (2) Static IP collisions will show repeated disconnects — pick a free address. (3) 5 GHz APs won't work — ESP32 is 2.4 GHz only.

### `test_sub_gyro_nondma`

- **Board env:** serial-only (no WiFi / no micro-ROS)
- **Purpose:** Absolute-minimal alive test for `GyroSubsystem`. Just verifies the BNO085 initialises over I²C.
- **Build/flash:** plug in via USB, `pio run -e <serial env> -t upload`
- **Debug tips:** (1) `BNO08x not detected` → check I²C pull-ups and pin assignments in `RobotConfig.h`. (2) Power-glitch crashes usually mean the board's 3v3 rail can't source the BNO085's peak current — bigger bulk cap.

### `test_sub_lidar`

- **Board env:** serial-only
- **Purpose:** Stream LD14P scan summaries at 1 Hz over serial and expose commands (`scan`, `stream`, `freq`, `info`, `help`) for diagnostics.
- **Serial output:** `scan: N points, min=<mm> max=<mm> freq=<Hz>`
- **Debug tips:** (1) No points → check UART baud (230400) and that TX/RX aren't swapped. (2) Jittery frequency → the LD14P's power draw is spiky; use a dedicated 5 V rail. (3) `freq` command lets you check the motor rpm without streaming every point.

### `test_sub_battery`

- **Board env:** serial-only
- **Purpose:** Interactive serial test for `BatterySubsystem`. Commands: `read`, `stream`, `cal`, `info`, `help`. Streams voltage and raw ADC at 1 Hz.
- **Debug tips:** (1) Use `cal` to capture a known reference voltage; the subsystem uses a 2-point linear calibration. (2) ADC non-linearity near the rails is large on ESP32 — keep the divider output between 0.5 V and 2.5 V for best accuracy.

### `test_sub_servo`

- **Board env:** serial-only
- **Purpose:** Interactive `ServoSubsystem` + PCA9685 tester. Commands include `attach`, `angle`, `vel`, `accel`, `invert`, `arm`, `disarm`, `budget`, `freq`. All 16 channels start detached so you can bring them online one at a time.
- **Debug tips:** (1) Always `arm` before commanding angles. (2) `budget` caps the simultaneous current draw; raise it only if your 5 V supply can handle it. (3) If the PCA9685 doesn't respond, I²C address is wrong (default `0x40`) or pull-ups are missing. (4) Use `freq` to tweak the PWM rate — standard hobby servos want 50 Hz, digital servos can handle 200 Hz+.

### `test_sub_gait`

- **Board env:** serial-only
- **Purpose:** Integration of `GaitController` + `HexapodKinematics` + `ServoSubsystem` driven by serial commands (`start`, `stop`, `halt`, `vel`, `status`, `neutral`). Useful for tuning the gait offline before connecting `/cmd_vel`.
- **Debug tips:** (1) If legs don't reach the commanded pose, check servo limits in `RobotConfig.h`. (2) `neutral` drives the legs to the zero stance — start every session from there. (3) If the robot stumbles on `start`, acceleration is too aggressive — lower the `vel` or tweak `GaitController` constants.

### `test_sub_led`

- **Board env:** serial-only
- **Purpose:** Interactive test for `LedSubsystem` driving 5 SK6812 LEDs. Commands include `clear`, `solid`, `set`, `brightness`, `pulse`, `chase`, `rainbow_pulse`, `chase_rainbow`, `rainbow_spread`.
- **Debug tips:** (1) SK6812 is SIDE-A orientation — power must match your PCB. (2) If colours look wrong, the colour order is probably `GRB` vs `RGB` — adjust in `LedSubsystem`. (3) Drawing lots of LEDs at full white at 3.3 V logic can be flaky — add a level shifter.

### `test_sub_cam`

- **Board env:** `esp32s3sense` / **Transport:** WiFi (no micro-ROS)
- **Purpose:** Stand-alone `CameraSubsystem` test. Connects to WiFi via DHCP, initialises the OV2640, and serves MJPEG at `http://<IP>/stream`. Serial commands: `status`, `ip`, `help`.
- **Debug tips:** (1) `psram init failed` → pick a smaller frame size or enable PSRAM in the build. (2) Camera init timeout usually means the ribbon cable isn't seated. (3) `curl http://<ip>/stream > test.mjpg` is a quick way to grab a test clip.

### `test_sub_cam_mic`

- **Board env:** `esp32s3sense` / **Transport:** WiFi
- **Purpose:** Runs `CameraSubsystem` and `MicSubsystem` side-by-side, each on its own httpd instance (camera :80, mic :81). Useful to verify both pipelines can coexist before flashing `test_all`.
- **Debug tips:** (1) If the mic server drops samples, the I²S DMA buffer is too small — increase it. (2) Two httpd instances compete for stack — you may need to bump `CONFIG_HTTPD_STACK_SIZE`.

### `test_sub_mic`

- **Board env:** `esp32s3sense` / **Transport:** WiFi
- **Purpose:** `MicSubsystem`-only HTTP PCM streaming at `http://<IP>:81/audio`. 16 kHz 16-bit mono PDM.
- **Debug tips:** (1) `ffplay -f s16le -ar 16000 -ac 1 http://<ip>:81/audio` to play back in real-time. (2) If output is silence, check the PDM clock pin; mics are sensitive to CLK polarity. (3) If it crackles, increase the I²S DMA buffer count.

### `test_sub_oled`

- **Board env:** serial-only (no WiFi / no micro-ROS)
- **Purpose:** Interactive serial harness for `OledSubsystem` driving an SSD1306 128×64 I²C display. The subsystem keeps a background frame (looked up by key in `OledFrames.h`, e.g. `blank`/`boot`/`status`) plus up to four text overlays (6×8 font, page-aligned `y`). All draw calls go into a RAM canvas and only the final `blt()` touches I²C, guarded by the shared bus mutex.
- **Commands:** `frame <key>`, `text <slot> <x> <y> <msg…>`, `cleartext [slot]`, `clear`, `contrast <0-255>`, `invert`, `normal`, `info`, `help`.
- **Build/flash:** `pio run -e <serial env> -t upload`, then `pio device monitor -b 921600`.
- **Debug tips:** (1) If `init` fails, check `oled_addr` in `RobotConfig.h` (default `0x3C`) and the I²C pull-ups. (2) `y` should be page-aligned (multiples of 8) for the 6×8 font — off-page y values clip weirdly. (3) Overlay slots are 0..3; slot 4+ is rejected. (4) Share the same I²C mutex with any other I²C subsystem (gyro, PCA9685) when combining sketches.

### `test_sub_speaker`

- **Board env:** `esp32s3sense` / **Transport:** WiFi
- **Purpose:** Complement to `seeker_tts`: long-polls `http://<AGENT_IP>:8383/audio_out` for PCM audio and plays it over I²S via `SpeakerSubsystem`.
- **Prereq:** `ros2 launch seeker_tts tts.launch.py` running on the host. For Fish Audio TTS set `FISH_API_KEY` and publish text to `/audio_tts_input` (`ros2 topic pub /audio_tts_input std_msgs/String "data: 'hello'" --once`). For local WAV playback, publish a file path on `/audio_play_file` (`ros2 topic pub /audio_play_file std_msgs/String "data: '/path/to/sound.wav'" --once`). Both share the same `/audio_out` HTTP stream.
- **Debug tips:** (1) If playback stutters, the network RTT is too high — move closer to the AP. (2) If it's silent, check I²S pins in `RobotConfig.h`. (3) Use `curl -v http://<host>:8383/audio_out` from another machine to test the endpoint independently — it should stay open and deliver a chunked stream whenever new text is published. (4) 404 from the endpoint means the path is wrong — the `tts_node` only serves `/audio_out`. (5) Mute first! Speaker volume defaults to max.

### `test_sub_movement`

- **Board env:** `esp32s3sense` / `esp32dev` / **Transport:** Serial + BLE (no micro-ROS agent required)
- **Purpose:** Comprehensive hexapod movement console that combines servo control, inverse kinematics, and tripod gait into a single dual-transport REPL (USB Serial + BLE Nordic UART). A command from either transport produces a response on **both**. Superset of `test_sub_servo` and `test_sub_gait` — carries over the full servo command set (`attach`, `detach`, `angle`, `vel`, `accel`, `invert`, `minpwm`/`maxpwm`, `arm`/`disarm`, `budget`, `freq`, `neutral`, `hips`, `knees`, `flat`, `standing`) and adds movement commands (`walk`, `stop`, `idle`, `forward`, `back`, `strafe`, `turn`, `move`), body height control (`height`), velocity caps (`max_velocities`, `max_hvel`), and gait tuning (`gait_step`, `gait_cycle`, `gait_scale`, `gait_status`). All tunings — servo calibration, gait parameters, body height, velocity caps — can be saved to NVS via `save` and auto-load on boot. Shares the NVS namespace `srvtest` with `test_sub_servo`, so servo calibration carries over between the two sketches. Also includes battery voltage readout and a heartbeat blink task.
- **Prereq:** Servos wired via PCA9685, battery connected (optional — status will omit voltage if init fails).
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Serial output:** Banner showing transport status (Serial + BLE), config source (NVS or defaults), gait tuning, and body height. Then an interactive command prompt.
- **Typical workflow:**
  ```
  > attachall           (attach M1–M12)
  > arm                 (enable OE)
  > standing            (kinematics neutral pose)
  > height 50           (body 50 mm above ground)
  > walk                (start tripod gait)
  > forward 0.05        (walk forward at 50 mm/s)
  > turn 30             (yaw 30 deg/s)
  > stop                (snap to neutral)
  > save                (persist all tunings to NVS)
  ```
- **Commands:** Type `help` for the full list. Key additions over `test_sub_servo`:
  - `walk` / `stop` / `idle` — start, snap-stop, or graceful-stop the gait
  - `forward <m/s>` / `back <m/s>` / `strafe <m/s>` / `turn <deg/s>` — single-axis velocity
  - `move <vx> <vy> <wz>` — full velocity command (m/s, m/s, rad/s)
  - `height <mm>` — set body height (IDLE only)
  - `max_velocities <vx> <vy> <wz>` — per-axis velocity caps
  - `max_hvel <m/s>` — combined horizontal velocity cap
  - `gait_step <mm>` / `gait_cycle <s>` / `gait_scale <x>` — tune gait parameters
  - `gait_status` — print current gait state and tuning
  - `save` / `clearprefs` — NVS persistence
- **Debug tips:** (1) If servos don't respond, check `arm` and `attachall` first. (2) `height` only works in IDLE — issue `stop` before changing it. (3) Velocity caps are applied silently; `status` shows the current caps. (4) `clearprefs` resets to HexapodConfig defaults on next boot. (5) BLE transport advertises as `"SeekerMovement"` — use nRF Connect or Bluefruit Connect to send commands wirelessly. (6) The debug level is reduced to 2 (INFO) to avoid flooding from battery samples.

### `test_sub_ble_debug`

- **Board env:** no micro-ROS; BLE only
- **Purpose:** Verifies the `BleDebugSubsystem` Nordic UART transport. Runs a simple counter and prints over BLE instead of UART.
- **Debug tips:** (1) Use `nRF Connect` or `Bluefruit Connect` on a phone to see the output. (2) If the device doesn't advertise, the NimBLE stack wasn't initialised — check build flags and serial log. (3) You can pair simultaneously with serial for debugging double-transport issues.

---

## Raw hardware tests (`test_raw_*`)

### `test_raw_cam`

- **Board env:** `esp32cam` (AI-Thinker ESP32-CAM)
- **Purpose:** Minimal raw MJPEG web server with no `CameraSubsystem` abstraction. Auto-selects frame size based on PSRAM availability.
- **Debug tips:** (1) Use `huge_app.csv` partition (set automatically by the env). (2) Camera init failures → double-check the AI-Thinker pin map in `RobotConfig.h` under `ENV_ESP32CAM`. (3) Good first check if `test_sub_cam` fails on the S3 sense.

### `test_raw_mic`

- **Board env:** `esp32s3sense` / **Transport:** WiFi
- **Purpose:** Raw PDM I²S stream over HTTP on port 80 without `MicSubsystem`. 16 kHz, 16-bit PCM.
- **Debug tips:** Same as `test_sub_mic`, but useful to rule out a subsystem bug vs a hardware bug.

### `test_raw_oled`

- **Board env:** bare (no WiFi / no micro-ROS)
- **Purpose:** Runs the upstream `lexus2k/ssd1306` demo sketch (`ssd1306_demo.cpp` + `sova.*`) directly against the SSD1306 128×64, with no `OledSubsystem` abstraction. Good sanity check when `test_sub_oled` misbehaves and you need to rule out everything above the bare driver.
- **Debug tips:** (1) Uses the ssd1306 library's own I²C init — don't share a bus with other subsystems without disabling them first. (2) If the display flickers, lower the I²C clock. (3) If the bitmap renders inverted, the column remap is wrong for your particular panel revision.

### `test_fast_led_raw`

- **Board env:** bare (no WiFi, no micro-ROS)
- **Purpose:** Raw FastLED test — blinks 5 SK6812 LEDs sequentially with a red→blue pattern on pin `D2`. No `LedSubsystem` abstraction.
- **Debug tips:** (1) Good first check when `test_sub_led` colours look wrong — bypasses the subsystem entirely. (2) If the chain is dark, the level shifter is likely missing.

---

## Smoke tests & placeholder

### `test_threaded_blink`

- **Board env:** bare
- **Purpose:** Minimal `ThreadedSubsystem` smoke test. Spawns a `BlinkSubsystem` FreeRTOS task and verifies the board runs pinned threads. Use this on any brand-new ESP32 to confirm FreeRTOS threading and board pin assignments are sane before layering anything on.

### `build_microros`

- **Board env:** inherits all from `platformio.ini`
- **Purpose:** Placeholder sketch used only to pre-build the micro-ROS PlatformIO library without compiling any real application code. Contains an empty `setup()`/`loop()`. Run `pio run` from this directory to cache the micro-ROS library build so subsequent sketch builds are faster.

### `test_bridge_media`

- **Board env:** inherits all from `platformio.ini`
- **Purpose:** Placeholder for a media bridge test (speaker + OLED over HTTP). Currently has only a `platformio.ini` — **no source files yet**. When implemented, it will test the speaker + OLED pipeline in isolation (without the sensor bridge).

### `main`

- **Board envs:** `esp32s3sense_main` (camera + mic + speaker on this board), `esp32s3sense_offload` (default — cam/mic offloaded to `main_satellite`), `esp32dev` (no camera/mic/speaker). OTA variants: `esp32s3sense_main_ota`, `esp32s3sense_offload_ota`, `esp32dev_ota`.
- **Transport:** WiFi (micro-ROS) + HTTP (camera :80, mic :81, speaker from :8383, OLED from :8390)
- **Bridge flags (offload):** `HEARTBEAT, GYRO, BATTERY, LIDAR` (DEBUG disabled to reduce PBUF pressure)
- **Purpose:** Full production firmware. All subsystems threaded and pinned; micro-ROS bridge publishes heartbeat (as `mcu/heartbeat1`), IMU, battery, lidar; `GaitRosParticipant` subscribes to `/cmd_vel`. Includes `StatusLedController` (battery/WiFi/micro-ROS/gait/audio state LED animations), `RobotPersistence` (NVS servo/gait/body-height calibration), and a safe-mode watchdog (5 boot threshold, clears after 10 s stable run, UDP port 4210).
- **Build/flash:** `pio run -t upload` (defaults to `esp32s3sense_offload`) or `pio run -e esp32s3sense_main -t upload` for the all-in-one variant.
- **Serial output:** WiFi + micro-ROS state, IP + RSSI every 2 s.
- **LED boot sequence:** Rainbow pulse → red chase (WiFi wait) → yellow pulse (micro-ROS wait) → cyan slow pulse (idle). Red fast pulse = low battery.
- **Safe mode override:** `echo -n "SEEKER_CLEAR_SM" | nc -u -w1 <MCU_IP> 4210`
- **Debug tips:** (1) Camera header (`sensor.h`) must be included before BNO driver to avoid `sensor_t` typedef collision. (2) I2C mutex is shared by gyro + servo + OLED. (3) In offload mode, camera and mic are disabled entirely — use `main_satellite` for those streams.

### `main_add`

- **Board envs:** `esp32s3sense`, `esp32dev`. OTA variants: `esp32s3sense_ota`, `esp32dev_ota`.
- **Transport:** WiFi (micro-ROS) + HTTP (camera :80, mic :81, speaker from :8383, OLED from :8390)
- **Bridge flags:** `HEARTBEAT, GYRO, BATTERY, LIDAR, DEBUG`
- **Purpose:** Incremental modular rebuild of `main` — phases 1–6 bring-up. All subsystems enabled; no `StatusLedController` (uses a simpler loop-side LED FSM instead). Useful for staged hardware bring-up where you want verbose debug output and all features on by default.
- **Build/flash:** `pio run -t upload` (default: `esp32s3sense`)
- **Serial output:** WiFi + micro-ROS state, IP + RSSI every 2 s.
- **Camera/mic/speaker:** Enabled on ESP32-S3 only (guarded by `#ifdef`). Camera init happens before WiFi to claim contiguous DMA buffer while heap is intact. Mic has acoustic feedback guard (`setMuteSource` tied to `SpeakerSubsystem::playing`).
- **Safe mode:** 5 boot threshold, 10 s stable run, UDP port 8889, any packet clears counter.
- **NVS namespace:** `"srvtest"` (shared with `test_sub_servo`, `test_sub_movement`).

### `main_satellite`

- **Board envs:** `esp32cam_satellite` (default, AI-Thinker ESP32-CAM), `esp32s3sense_satellite` (Seeed XIAO ESP32-S3 Sense). OTA variants available.
- **Transport:** WiFi (HTTP). micro-ROS optional (enabled on S3, disabled on ESP32-CAM).
- **Bridge flags (S3 only):** `HEARTBEAT` (publishes as `mcu/heartbeat2`)
- **Purpose:** Camera/sensor offload board for the dual-board architecture. The main board (`main` with `esp32s3sense_offload`) handles gait, sensors, and micro-ROS; this board runs the MJPEG camera server, freeing PBUF/SRAM pressure on the main board.
- **Build/flash:** `pio run -t upload` (default: `esp32cam_satellite`) or `pio run -e esp32s3sense_satellite -t upload`
- **HTTP endpoints:** Camera MJPEG at `:80/cam`. Mic PCM at `:81/audio` (S3 only, not yet on ESP32-CAM).
- **Safe mode:** 5 boot threshold, 30 s stable run (longer than main), UDP port 4211. Override: `echo -n "SEEKER_CLEAR_SM" | nc -u -w1 <SAT_IP> 4211`
- **Network config:** Requires `satellite_ip` in `network_config.ini`. Shares WiFi creds with main.
- **Why ESP32-CAM micro-ROS is disabled:** The XRCE-DDS executor on Core 1 contends with the MJPEG send path on the 4 MB non-S3 ESP32, throttling camera to ~3 fps.
- **Debug tips:** (1) Camera init must happen before WiFi (contiguous DMA buffer). (2) On ESP32-CAM, default I2C pins (21/22) collide with camera Y5/PCLK — gyro setup carries explicit SDA/SCL. (3) Loop logs WiFi state + micro-ROS state ("DISABLED" when `ENABLE_MICROROS=0`).

### `test_raw_bno`

- **Board env:** `esp32s3sense_bare` (no WiFi, no micro-ROS)
- **Purpose:** BNO085 IMU raw hardware isolation test. Bare-metal I2C test without any subsystem abstraction — verifies the BNO085 sensor responds and produces data.
- **Build/flash:** `pio run -t upload`
- **Debug tips:** Same as `test_sub_gyro_nondma` but bypasses the `GyroSubsystem` abstraction entirely.

---

## Debugging reference — commands that save time

```bash
# Serial monitor with backtrace decoder
pio device monitor -b 921600 --filter esp32_exception_decoder

# List the current PlatformIO env chain
pio run --verbose --target envdump

# Erase flash (fixes "stuck" boards)
pio run -e esp32s3sense -t erase

# OTA upload over WiFi (needs device already running ENABLE_ARDUINO_OTA=1)
pio run -e esp32s3sense_ota -t upload
```
