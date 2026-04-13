# MCU Sketches

Every directory under `mcu_ws/src/` is a standalone PlatformIO project. They follow a naming convention:

- `main` — placeholder for full system integration (empty `setup()`/`loop()`)
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

- **Board env:** `esp32s3sense`
- **Transport:** WiFi (micro-ROS) + HTTP (camera :80, mic :81, speaker from :8383, OLED from :8384)
- **Bridge flags:** `HEARTBEAT, GYRO, BATTERY, LIDAR, DEBUG`
- **Purpose:** The "everything on" integration test. Runs the full `MicroRosBridge` plus a concurrent MJPEG camera server, PDM audio HTTP server, I2S speaker (fetches PCM from host :8383), and OLED display (fetches framebuffers from host :8384) on a single ESP32-S3 Sense. This is the firmware you flash when you want the real robot to look like the Gazebo simulation — all sensor topics live plus camera/mic streams pullable from ROS.
- **Prereq:** micro-ROS agent running (`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`).
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Serial output:** WiFi connection status, agent state transitions, IP + RSSI, periodic participant heartbeat.
- **Verify on ROS side:**
  ```bash
  ros2 topic hz /mcu/heartbeat        # 1 Hz
  ros2 topic hz /mcu/imu              # ~50 Hz
  ros2 topic hz /mcu/scan             # ~6 Hz
  ros2 topic hz /mcu/battery_voltage  # 1 Hz
  curl http://<esp_ip>/stream         # MJPEG
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
- **Transport:** WiFi (micro-ROS) + HTTP (speaker from :8383, OLED from :8384)
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
- **Debug tips:** Same as `test_all` minus the camera/mic issues. If `/mcu/scan` publishes 0 points, the LD14P is probably unpowered or the UART pins in `RobotConfig.h` don't match your wiring. If the OLED never updates, make sure the host is serving on port 8384 (`ros2 run seeker_display oled_sine`) and that the ESP32 can reach `AGENT_IP:8384`.

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
- **Purpose:** Isolated WiFi + OLED HTTP streaming test. The ESP32 connects to WiFi, then opens a persistent HTTP connection to the host at `AGENT_IP:8384/lcd_out` and reads 1024-byte SSD1306 framebuffers continuously. The `OledSubsystem` renders them at ~10 Hz. Text overlays show the boot banner and WiFi connection state. No `MicroRosBridge` is involved — the OLED data flows over plain HTTP, independent of DDS.
- **Prereq:** WiFi configured in `network_config.ini`; SSD1306 128×64 wired on the shared I²C bus (`Config::sda`/`Config::scl`, default address `0x3C`); a host-side LCD server running.
- **Build/flash:** `pio run -e esp32s3sense -t upload`
- **Verify on host (pick one):**
  ```bash
  ros2 run seeker_display oled_sine    # animated sine wave
  ros2 launch seeker_media media.launch.py  # MP4 video playback
  ```
- **Debug tips:** (1) Display blank → confirm `AGENT_IP` in `network_config.ini` and that the host LCD server is running on port 8384. (2) Display shows boot frame but never updates → WiFi may not be connected yet; check serial output. (3) Updates cap at 10 Hz — this is by design. (4) If text overlays don't show, remember the HTTP framebuffer push overrides them until the next overlay update tick.

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

- **Board env:** `esp32s3sense`
- **Purpose:** Placeholder for the "final" integration firmware. Currently an empty `setup()`/`loop()` — don't flash.

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
