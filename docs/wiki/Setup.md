# Setup

This page walks you from a fresh machine to a running dev container with ROS 2 Jazzy, Gazebo Harmonic, and PlatformIO all inside Docker. Everything works identically on Linux, macOS, and Windows — only the X server and USB passthrough details differ.

---

## 1. Host prerequisites

| Tool | Linux | macOS | Windows |
|---|---|---|---|
| Git (+ Git LFS) | `apt install git git-lfs` | `brew install git git-lfs` | Git for Windows |
| Docker + Compose v2 | Docker Engine + `docker-compose-plugin` | Docker Desktop | Docker Desktop (WSL2 backend) |
| X server | native (already running) | [XQuartz](https://www.xquartz.org/) | [VcXsrv](https://sourceforge.net/projects/vcxsrv/) |
| USB passthrough | native | n/a (no serial flashing from macOS containers) | [usbipd-win](https://github.com/dorssel/usbipd-win) |
| GPU acceleration (optional) | `nvidia-container-toolkit` for Gazebo GPU | n/a | NVIDIA Container Toolkit via WSL2 |

You should also have a working WiFi network the ESP32 and the host can both see. Everything inside the container defaults to **WiFi / UDP micro-ROS** over port `8888`.

### X server, per OS

**Linux:** nothing to do — the container will bind `/tmp/.X11-unix` and pick up `$DISPLAY` automatically. In `docker/.env` set:

```
DISPLAY_CONFIG=${DISPLAY}
NETWORK_MODE_CONFIG=host
```

**macOS (XQuartz):**

1. Install XQuartz and launch it.
2. Preferences → Security → check *Allow connections from network clients*.
3. Run `xhost +localhost` once per login.
4. In `docker/.env`:

```
DISPLAY_CONFIG=host.docker.internal:0
NETWORK_MODE_CONFIG=bridge
```

**Windows (VcXsrv on Docker Desktop):**

1. Install VcXsrv and launch *XLaunch* with **Disable access control** checked.
2. Allow VcXsrv through the Windows firewall on private networks.
3. In `docker/.env`:

```
DISPLAY_CONFIG=host.docker.internal:0
NETWORK_MODE_CONFIG=bridge
```

### `usbipd-win` for Windows serial flashing

WSL2 and Docker Desktop cannot see host COM ports directly. To flash the ESP32 over USB serial from inside the container on Windows:

```powershell
usbipd list                         # find the ESP32 BUSID
usbipd bind   --busid <BUS_ID>
usbipd attach --wsl --busid <BUS_ID>
```

The device then appears as `/dev/ttyUSB0` (or similar) inside the container. If you only flash over WiFi (using the `*_ota` PlatformIO envs), you can skip `usbipd` entirely.

---

## 2. Clone the repo

```bash
git clone --recurse-submodules https://github.com/SeekerRobot/seeker-robot.git
cd seeker-robot
```

`--recurse-submodules` is important if the PCB / hardware submodules live under `doc/`.

---

## 3. Configure env files

Both files below are gitignored — your local values never ship.

```bash
cp docker/.env.example docker/.env
cp mcu_ws/platformio/network_config.example.ini mcu_ws/platformio/network_config.ini
```

### `docker/.env`

| Key | What to set |
|---|---|
| `COMPOSE_PROJECT_NAME` | Unique per worktree so containers/volumes don't collide (e.g. `seeker-robot-main`, `seeker-robot-feature-x`). |
| `BUILD_TARGET` | `dev` includes Gazebo/RViz/Nav2/SLAM Toolbox tooling. `prod` is headless, runtime-only. Pick `dev` unless you're deploying to a fleet. |
| `DISPLAY_CONFIG` / `NETWORK_MODE_CONFIG` | Uncomment and fill in the block for your OS (see §1). |
| `FISH_API_KEY`, `FISH_REFERENCE_ID` | Optional — only needed if you plan to run the `seeker_tts` node. |

### `mcu_ws/platformio/network_config.ini`

All values are consumed as C preprocessor defines by the WiFi micro-ROS base env in `mcu_ws/platformio/platformio.ini`. They are brace-initialised on purpose (`{192,168,8,134}` not `"192.168.8.134"`) because they feed `IPAddress(...)`.

```ini
[network]
wifi_ssid      = YourSSID
wifi_password  = YourPassword
agent_ip       = { 192, 168, 8, 134 }   ; machine running the micro-ROS agent
agent_port     = 8888
static_ip      = { 192, 168, 8, 50 }    ; assigned to the ESP32 (required)
gateway        = { 192, 168, 8, 1 }
subnet         = { 255, 255, 255, 0 }
ota_upload_port = 192.168.8.50           ; plain string for espota
```

> **A static IP for the ESP32 is required for micro-ROS to work correctly.** You can either reserve one in your router DHCP or configure the firmware to claim one.

---

## 4. First build and launch

All `docker compose` commands below are run from the `docker/` directory. If you prefer running from the repo root, add `-f docker/docker-compose.yml`.

```bash
cd docker

# Build images (forcing a clean init-bootstrap build the first time avoids
# stale chown state)
docker compose build --no-cache init-bootstrap
docker compose build

# Seed named volumes (chown + copy libs_external into mcu_lib_external)
docker compose up init-bootstrap

# Start the main dev container in the background
docker compose up -d ros2

# Open a shell inside it
docker compose exec ros2 bash
```

### GPU-accelerated variants

```bash
docker compose --profile nvidia up -d ros2-nvidia   # NVIDIA (requires nvidia-container-toolkit)
docker compose --profile amd    up -d ros2-amd      # AMD / Intel DRI passthrough
```

These services `extends` the base `ros2` service and only add driver/device exposure.

### First ROS 2 build (inside the container)

```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

This builds `mcu_msgs`, `seeker_description`, `seeker_gazebo`, `seeker_navigation`, `seeker_sim`, `seeker_tts`, and `test_package`. Build artifacts go into named Docker volumes (`ros2_build`, `ros2_install`, `ros2_log`) so they survive container restarts without cluttering the host.

`source install/setup.bash` must be rerun in every new shell — or add it to `~/.bashrc` alongside `source /opt/ros/jazzy/setup.bash` (the Dockerfile adds only the ROS 2 base source automatically).

---

## 5. Smoke checks

Run these inside the container after the first build to make sure everything is wired up.

```bash
# ROS 2 finds the packages
ros2 pkg list | grep seeker
# Expected: seeker_description seeker_gazebo seeker_navigation seeker_sim seeker_tts ...

# micro-ROS agent is installed
ros2 run micro_ros_agent micro_ros_agent --help

# PlatformIO is on PATH
pio --version

# Gazebo Harmonic is available (dev target only)
gz sim --version

# The URDF loads cleanly
ros2 launch seeker_description display.launch.py
```

Close the display launch with `Ctrl-C`. If RViz opened and showed the hexapod model, your X server is wired up too.

Try the simplest end-to-end test next:

```bash
ros2 launch seeker_gazebo sim_teleop.launch.py
# in another shell: ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

See **[Simulation](Simulation.md)** for all the launch modes.

---

## 6. Troubleshooting

### Docker / container

| Symptom | Fix |
|---|---|
| `init-bootstrap` errors about permissions | Rerun `docker compose build --no-cache init-bootstrap && docker compose up init-bootstrap`. It idempotently chowns every named volume to UID 1000 (`ubuntu`). |
| `ros2` container exits immediately after `up` | Check `docker compose logs ros2`. The most common cause is a failing bind mount — make sure `mcu_ws/platformio/network_config.ini` actually exists (it's mounted read-only, Docker errors out if it doesn't). |
| *stale volumes from an old worktree* | Set a unique `COMPOSE_PROJECT_NAME` per worktree in `docker/.env`, or `docker compose down -v` to nuke the old volumes. |

### X server / display

| Symptom | Fix |
|---|---|
| `Could not connect to display` / Gazebo window doesn't open | Linux: `xhost +local:docker`. macOS: XQuartz → Preferences → Security → *Allow connections from network clients*, then `xhost +localhost`. Windows: launch VcXsrv with **Disable access control** checked. |
| Gazebo opens but renders a black screen | GPU-accel missing. On NVIDIA hosts install `nvidia-container-toolkit` and use `docker compose --profile nvidia up -d ros2-nvidia`. On AMD/Intel use `--profile amd`. |
| RViz crashes with an OpenGL / GLX error | Usually the same issue as above — switch to a GPU profile service. |

### Networking / micro-ROS

| Symptom | Fix |
|---|---|
| ESP32 boots but agent never logs `create_session` | 1) On Windows/macOS switch `NETWORK_MODE_CONFIG` to `bridge` and set `DISPLAY_CONFIG=host.docker.internal:0`, then forward UDP/8888. 2) Confirm `agent_ip` in `network_config.ini` matches the IP of the machine actually running `ros2 run micro_ros_agent …`. 3) Double-check the static IP is free and inside the subnet. |
| `ros2 topic hz /mcu/imu` reports `No new messages` but Gazebo is working | Known Docker DDS multicast isolation issue. Run the command in the *same shell* that launched Gazebo, or rebuild with the `seeker_gazebo` README workaround. |
| Firmware builds fine but the ESP32 never joins WiFi | You likely filled in the wrong brace-style IPs in `network_config.ini`. They must be `{a,b,c,d}` integer literals, not strings. |

### PlatformIO

| Symptom | Fix |
|---|---|
| `error: extra_packages/mcu_msgs: not found` | The `mcu_msgs` bind mount is missing. Make sure the container was started via docker-compose (which mounts it automatically) and that `ros2_ws/src/mcu_msgs/` exists on the host. |
| Serial upload fails on Linux with `Permission denied` on `/dev/ttyUSB0` | Re-enter the container (`docker compose exec ros2 bash`) — the container runs privileged and with `/dev` bind-mounted, so the permission comes from the host. On the host, add your user to the `dialout` group. |
| Serial upload impossible on Windows | Run `usbipd attach --wsl --busid <BUS_ID>` as shown in §1, or flash over WiFi with the `*_ota` envs. |

See **[MCU Firmware](MCU-Firmware.md)** for more PlatformIO details and **[IRL Tests](IRL-Tests.md)** for hardware bring-up procedures.
