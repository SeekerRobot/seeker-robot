# Contributing

Anything that lands on `main` has to pass CI and follow the conventions below. This page collects everything you need to make a clean PR.

---

## Commit messages — Conventional Commits

The repo enforces [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) via `@commitlint/config-conventional`. The GitHub Actions workflow `.github/workflows/commitlint.yml` runs `npx commitlint --from <merge-base> --to HEAD --verbose` on every pull request to `main`, so non-conforming commits will fail CI.

Use one of the standard prefixes:

| Prefix | When to use |
|---|---|
| `feat:` | A new feature (new sketch, new ROS 2 node, new capability). |
| `fix:` | A bug fix. |
| `docs:` | Documentation-only changes (including this wiki — commit in the main repo if updating `CLAUDE.md` / `README.md`). |
| `chore:` | Tooling, CI, build, deps — anything that doesn't touch runtime code. |
| `refactor:` | Code restructuring with no behaviour change. |
| `style:` | Whitespace / clang-format / cosmetic — no semantic change. |
| `test:` | Adding or updating tests. |
| `perf:` | Performance improvement. |
| `ci:` | GitHub Actions / CI pipeline changes. |
| `build:` | Build-system or dependency changes (Dockerfile, platformio.ini, CMake, package.xml). |

Scopes are optional but encouraged for multi-component repos:

```
feat(mcu): add SpeakerSubsystem long-poll HTTP client
fix(seeker_navigation): wait for SLAM lifecycle before Nav2 bringup
docs(wiki): clarify usbipd-win steps on Windows
```

Breaking changes: append `!` and add a `BREAKING CHANGE:` footer.

```
feat(mcu_msgs)!: rename HexapodCmd.gait_mode to gait_state

BREAKING CHANGE: /mcu/hexapod_cmd field renamed; rebuild both workspaces.
```

---

## CI checks

All workflows live under `.github/workflows/`.

### `commitlint.yml`

- Trigger: `pull_request` → `main`.
- Node 20 + `@commitlint/cli @commitlint/config-conventional`.
- Runs `npx commitlint --from <merge-base> --to HEAD --verbose`.

### `clang-format.yml`

- Trigger: `pull_request` → `main`.
- Runs `clang-format -i --style=file` over every `*.c` / `*.cpp` / `*.h` / `*.hpp` under `mcu_ws/` and `ros2_ws/`, **excluding** anything under `libs_external/`.
- **Auto-commits the fix-ups** back to the PR branch. So if your PR fails clang-format, just pull the auto-style commit — don't fight the bot.

To run it locally before pushing:

```bash
find mcu_ws ros2_ws \
  -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \
  | grep -v 'libs_external/' \
  | xargs -r clang-format -i --style=file
```

---

## Naming conventions

### Sketches (`mcu_ws/src/<name>/`)

| Pattern | Meaning |
|---|---|
| `test_sub_*` | Single subsystem in isolation. Usually serial-only (no micro-ROS), although a few (`test_sub_cam`, `test_sub_cam_mic`, `test_sub_mic`, `test_sub_speaker`, `test_sub_wifi`, `test_sub_heartbeat`) do need WiFi because the subsystem itself does. |
| `test_bridge_*` | Exercises the full micro-ROS stack over WiFi. `test_bridge_all` publishes all enabled bridge topics; `test_bridge_gait` subscribes to `/cmd_vel`. |
| `test_raw_*` | Low-level hardware tests with no subsystem abstraction (e.g. `test_raw_cam`, `test_raw_mic`). |
| `test_all` | Full integration: all publishers **plus** camera + mic HTTP servers running on one board. |
| `test_threaded_blink` | `ThreadedSubsystem` / FreeRTOS smoke test. |
| `test_fast_led_raw` | FastLED blink bypassing `LedSubsystem`. |
| `main` | Placeholder for the final integration firmware (currently empty). |

Serial-only sketches must **exclude** `libs_external/esp32` from `lib_extra_dirs` in their own `platformio.ini` to avoid pulling in micro-ROS.

### ROS 2 packages (`ros2_ws/src/<name>/`)

- `seeker_*` — all runtime packages for the robot. One package = one concern (description, gazebo sim, navigation, sim stub, tts).
- `mcu_msgs` — the only package **not** prefixed `seeker_`, because it's a pure interface package shared with the firmware.
- `test_package` — CI-sanity only.

### Shared libraries (`mcu_ws/lib/<name>/`)

- `XxxSubsystem` — hardware abstractions that derive from `ThreadedSubsystem` (`GyroSubsystem`, `LidarSubsystem`, …).
- `XxxParticipant` — classes that implement `IMicroRosParticipant` (`HeartbeatParticipant`, …).
- `MicroRosBridge` — the one big compile-time-plugin bridge that owns every publisher.
- `RobotConfig` — header-only pin map gated on board env.
- Libraries named without a pattern are generic C++ utilities (`HexapodKinematics`, `CustomDebug`, `hal_thread`).

---

## Adding a new MCU sketch

1. Create `mcu_ws/src/<sketch>/platformio.ini` and `mcu_ws/src/<sketch>/src/main.cpp`.
2. Inherit from the shared base so network config and libraries are injected:

   ```ini
   [platformio]
   default_envs  = esp32s3sense
   extra_configs = ../../platformio/platformio.ini

   [env:esp32s3sense]
   extends = env:esp32s3sense     ; pulled from the shared base
   build_flags =
       ${env:esp32s3sense.build_flags}
       -DBRIDGE_ENABLE_HEARTBEAT=1
       -DBRIDGE_ENABLE_GYRO=1
   ```

3. If it's a serial-only `test_sub_*`, drop micro-ROS:

   ```ini
   [env:serial_only]
   platform  = espressif32
   board     = esp32dev
   framework = arduino
   lib_extra_dirs = ../../lib/
   lib_deps       = ${common.lib_base}
   build_flags    = ${common.build_flags}
   build_unflags  = ${common.build_unflags}
   ```

4. Build: `pio run -e <env>` from the new sketch directory.
5. Add a section to **[MCU Sketches](MCU-Sketches.md)** describing what it does.

---

## Adding a new ROS 2 package

1. `ros2 pkg create --build-type ament_{cmake,python} <pkg_name>` inside `ros2_ws/src/`.
2. Declare dependencies in `package.xml` (both `<depend>` and `<exec_depend>` for Python nodes).
3. For Python packages, make sure `setup.py` registers your entry points:

   ```python
   entry_points={
       'console_scripts': [
           'my_node = my_pkg.my_node:main',
       ],
   },
   ```

4. Put launch files under `launch/`, config under `config/`, and install both from `setup.py` (Python) or `CMakeLists.txt` (C++).
5. Build: `colcon build --packages-select <pkg_name>`.
6. Add a section to **[ROS2 Packages](ROS2-Packages.md)**.

---

## Adding a publisher to `MicroRosBridge`

Checklist for the compile-time plugin pattern (see also **[Architecture](Architecture.md#the-microrosbridge-compile-time-plugin-pattern)**):

- [ ] `#ifndef BRIDGE_ENABLE_FOO / #define BRIDGE_ENABLE_FOO 0` added in `MicroRosBridge.h`.
- [ ] Subsystem header `#include`d under `#if BRIDGE_ENABLE_FOO`.
- [ ] `FooPublisherState` struct defined (holds `rcl_publisher_t pub`, the message buffer, and an `elapsedMillis elapsed` timer).
- [ ] `kEnableFoo = BRIDGE_ENABLE_FOO` added to `BridgeConfig`.
- [ ] `FooSubsystem* foo = nullptr` + `uint32_t foo_interval_ms` added to `MicroRosBridgeSetup`.
- [ ] `std::conditional_t<BridgeConfig::kEnableFoo, FooPublisherState, EmptyState> foo_` member added.
- [ ] `#if BRIDGE_ENABLE_FOO` block in `MicroRosBridge::onCreate()` — initialises the message buffer and creates the publisher.
- [ ] `#if BRIDGE_ENABLE_FOO` block in `MicroRosBridge::onDestroy()` — zero-inits the publisher handle and frees the message buffer.
- [ ] `#if BRIDGE_ENABLE_FOO` block in `MicroRosBridge::publishAll()` — reads the subsystem getter (thread-safe!), fills the message, calls `rcl_publish()`. Non-blocking only.
- [ ] Sketch's `platformio.ini` opts in with `-DBRIDGE_ENABLE_FOO=1` in `build_flags`.
- [ ] New topic added to **[Architecture](Architecture.md)** and **[MCU Firmware](MCU-Firmware.md)** references.

---

## `mcu_msgs` change checklist

When you edit anything under `ros2_ws/src/mcu_msgs/`:

- [ ] Rebuild ROS 2 side: `colcon build --packages-select mcu_msgs && source install/setup.bash`
- [ ] Clean and rebuild **every** firmware sketch that uses the changed message: `pio run --target clean && pio run -e <env>`
- [ ] Update any consumer code on both sides that references removed/renamed fields.
- [ ] Breaking changes → `feat!:` or `fix!:` + a `BREAKING CHANGE:` footer in the commit message.

---

## Pull request checklist

- [ ] Commit messages follow Conventional Commits (`commitlint` will check).
- [ ] `clang-format` is clean, or you've pulled the auto-style commit.
- [ ] ROS 2 workspace builds: `colcon build && colcon test && colcon test-result --verbose`.
- [ ] All firmware sketches you touched build: `pio run` from each `mcu_ws/src/<sketch>/`.
- [ ] You updated `CLAUDE.md` / `README.md` / this wiki if you changed user-facing behaviour, added a sketch, added a package, or changed any documented command.
- [ ] You ran the relevant `test_sub_*` sketches before flashing a big integration build.
- [ ] No secrets, `.env` files, or personal WiFi creds are included.
- [ ] PR description explains **why**, not just **what**.
