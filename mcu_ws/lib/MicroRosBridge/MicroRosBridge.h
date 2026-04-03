/**
 * @file MicroRosBridge.h
 * @author Aldem Pido
 * @date 3/31/2026
 * @brief "God bridge" IMicroRosParticipant that owns all ROS
 * publishers/subscribers for hardware subsystems that are not ROS-aware
 * themselves (gyro, battery, servo, …). Non-involved subsystems simply expose a
 * getter; the bridge reads and publishes at the configured rate.
 *
 * ## Compile-time feature flags
 * Each subsystem publisher is gated by a preprocessor flag that is surfaced as
 * a `constexpr bool` in BridgeConfig. Disabled publishers cost zero RAM — the
 * corresponding state struct is replaced by an empty placeholder at compile
 * time.
 *
 * Override defaults in `platformio.ini` build_flags, e.g.:
 *   build_flags = -D BRIDGE_ENABLE_GYRO=1
 *
 * ## Adding a new subsystem publisher
 *   1. Add `#ifndef BRIDGE_ENABLE_FOO / #define BRIDGE_ENABLE_FOO 0` below.
 *   2. Conditionally include the subsystem header and define
 * `FooPublisherState`.
 *   3. Add fields to `MicroRosBridgeSetup` (forward-declared pointer is
 * enough).
 *   4. Add `std::conditional_t<…>` member `foo_` to `MicroRosBridge`.
 *   5. Implement the `#if BRIDGE_ENABLE_FOO` blocks in MicroRosBridge.cpp.
 */
#pragma once

#include <elapsedMillis.h>
#include <microros_manager_robot.h>

#include <type_traits>

// ---- Feature flags (defaults 0; override via platformio build_flags)
// ----------
#ifndef BRIDGE_ENABLE_HEARTBEAT
#define BRIDGE_ENABLE_HEARTBEAT 0
#endif
#ifndef BRIDGE_ENABLE_GYRO
#define BRIDGE_ENABLE_GYRO 0
#endif
#ifndef BRIDGE_ENABLE_BATTERY
#define BRIDGE_ENABLE_BATTERY 0
#endif
#ifndef BRIDGE_ENABLE_SERVO
#define BRIDGE_ENABLE_SERVO 0
#endif
#ifndef BRIDGE_ENABLE_LIDAR
#define BRIDGE_ENABLE_LIDAR 0
#endif
#ifndef BRIDGE_ENABLE_DEBUG
#define BRIDGE_ENABLE_DEBUG 0
#endif

// Compile-time sanity check — warns if every feature is disabled so a silent
// no-op bridge isn't mistaken for a working one.
#if !BRIDGE_ENABLE_HEARTBEAT && !BRIDGE_ENABLE_GYRO && \
    !BRIDGE_ENABLE_BATTERY && !BRIDGE_ENABLE_SERVO && \
    !BRIDGE_ENABLE_LIDAR && !BRIDGE_ENABLE_DEBUG
#pragma message( \
    "MicroRosBridge: all features disabled — no publishers will be created. Set -DBRIDGE_ENABLE_*=1 in build_flags.")
#endif

// ---- Conditional includes for enabled features
// --------------------------------
#if BRIDGE_ENABLE_HEARTBEAT
#include <std_msgs/msg/int32.h>
#endif
#if BRIDGE_ENABLE_GYRO
#include <GyroSubsystem.h>
#include <sensor_msgs/msg/imu.h>
#endif
#if BRIDGE_ENABLE_BATTERY
#include <BatterySubsystem.h>
#include <std_msgs/msg/float32.h>
#endif
// #if BRIDGE_ENABLE_SERVO
// #include <ServoSubsystem.h>
// #include <sensor_msgs/msg/joint_state.h>
// #endif
#if BRIDGE_ENABLE_LIDAR
#include <LidarSubsystem.h>
#include <sensor_msgs/msg/laser_scan.h>
#endif
#if BRIDGE_ENABLE_DEBUG
#include <std_msgs/msg/string.h>
#endif

namespace Subsystem {

// Forward-declare subsystem types so the setup struct can hold pointers to them
// regardless of which features are enabled.
#if !BRIDGE_ENABLE_GYRO
class GyroSubsystem;
#endif
#if !BRIDGE_ENABLE_BATTERY
class BatterySubsystem;
#endif
// class ServoSubsystem;
#if !BRIDGE_ENABLE_LIDAR
class LidarSubsystem;
#endif

// ---- Compile-time feature switches
// -------------------------------------------

struct BridgeConfig {
  static constexpr bool kEnableHeartbeat = BRIDGE_ENABLE_HEARTBEAT;
  static constexpr bool kEnableGyro = BRIDGE_ENABLE_GYRO;
  static constexpr bool kEnableBattery = BRIDGE_ENABLE_BATTERY;
  static constexpr bool kEnableServo = BRIDGE_ENABLE_SERVO;
  static constexpr bool kEnableLidar = BRIDGE_ENABLE_LIDAR;
  static constexpr bool kEnableDebug = BRIDGE_ENABLE_DEBUG;
};

// ---- Zero-cost placeholder for disabled features
// -----------------------------

struct EmptyState {};

// ---- Per-feature publisher state structs
// -------------------------------------

#if BRIDGE_ENABLE_HEARTBEAT
struct HeartbeatPublisherState {
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  std_msgs__msg__Int32 msg{};
  elapsedMillis elapsed{};
};
#else
using HeartbeatPublisherState = EmptyState;
#endif

#if BRIDGE_ENABLE_GYRO
struct GyroPublisherState {
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  sensor_msgs__msg__Imu msg{};
  elapsedMillis elapsed{};
};
#else
using GyroPublisherState = EmptyState;
#endif

#if BRIDGE_ENABLE_BATTERY
struct BatteryPublisherState {
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  std_msgs__msg__Float32 msg{};
  elapsedMillis elapsed{};
};
#else
using BatteryPublisherState = EmptyState;
#endif

#if BRIDGE_ENABLE_SERVO
struct ServoPublisherState {
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  // TODO: sensor_msgs__msg__JointState msg{};
  elapsedMillis elapsed{};
};
#else
using ServoPublisherState = EmptyState;
#endif

#if BRIDGE_ENABLE_LIDAR
struct LidarPublisherState {
  static constexpr uint16_t kCapacity = kLidarMaxPoints;  // 720
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  sensor_msgs__msg__LaserScan msg{};
  float ranges_buf[kCapacity];
  float intensities_buf[kCapacity];
  elapsedMillis elapsed{};
  uint32_t last_scan_count = 0;
};
#else
using LidarPublisherState = EmptyState;
#endif

#if BRIDGE_ENABLE_DEBUG
// Debug log publisher: Debug::printf output → std_msgs/String → mcu/log.
// Messages are enqueued from any task via MicroRosDebug::enqueue() and
// drained in publishAll() on the manager task.
struct DebugPublisherState {
  static constexpr uint16_t kMsgLen = 240;
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  std_msgs__msg__String msg{};
  char data_buf[kMsgLen + 1];  // backing buffer wired to msg.data.data
};
#else
using DebugPublisherState = EmptyState;
#endif

// ---- Setup struct
// ------------------------------------------------------------

struct MicroRosBridgeSetup {
  // Heartbeat — used only when BridgeConfig::kEnableHeartbeat is true.
  const char* heartbeat_topic = "mcu/heartbeat";
  uint32_t heartbeat_interval_ms = 1000;

  // Gyro IMU publisher — used only when BridgeConfig::kEnableGyro is true.
  GyroSubsystem* gyro = nullptr;
  const char* imu_topic = "mcu/imu";
  uint32_t imu_interval_ms = 20;  ///< ms (50 Hz default)

  // Battery — used only when BridgeConfig::kEnableBattery is true.
  BatterySubsystem* battery = nullptr;
  const char* battery_topic = "mcu/battery_voltage";
  uint32_t battery_interval_ms = 1000;  ///< ms (1 Hz default)

  // Servo — used only when BridgeConfig::kEnableServo is true.
  // ServoSubsystem* servo             = nullptr;
  // const char*     servo_topic       = "mcu/joint_states";
  // uint32_t        servo_interval_ms = 20;

  // Lidar — used only when BridgeConfig::kEnableLidar is true.
  LidarSubsystem* lidar = nullptr;
  const char* scan_topic = "mcu/scan";
  uint32_t scan_interval_ms = 50;  ///< Min ms between publishes (~20 Hz cap).

  // Debug log — used only when BridgeConfig::kEnableDebug is true.
  const char* log_topic = "mcu/log";
};

// ---- Bridge
// ------------------------------------------------------------------

class MicroRosBridge : public IMicroRosParticipant {
 public:
  explicit MicroRosBridge(const MicroRosBridgeSetup& setup);

  bool onCreate(MicroRosContext& ctx) override;
  void onDestroy() override;
  void publishAll() override;

 private:
  const MicroRosBridgeSetup setup_;
  bool initialized_ = false;

  std::conditional_t<BridgeConfig::kEnableHeartbeat, HeartbeatPublisherState,
                     EmptyState>
      heartbeat_;
  std::conditional_t<BridgeConfig::kEnableGyro, GyroPublisherState, EmptyState>
      gyro_;
  std::conditional_t<BridgeConfig::kEnableBattery, BatteryPublisherState,
                     EmptyState>
      battery_;
  std::conditional_t<BridgeConfig::kEnableServo, ServoPublisherState,
                     EmptyState>
      servo_;
  std::conditional_t<BridgeConfig::kEnableLidar, LidarPublisherState,
                     EmptyState>
      lidar_;
  std::conditional_t<BridgeConfig::kEnableDebug, DebugPublisherState,
                     EmptyState>
      debug_;
};

}  // namespace Subsystem
