/**
 * @file MicroRosBridge.cpp
 * @author Aldem Pido
 * @date 3/31/2026
 */
#include "MicroRosBridge.h"

#include <CustomDebug.h>
#include <MicroRosDebug.h>
#include <string.h>

namespace Subsystem {

MicroRosBridge::MicroRosBridge(const MicroRosBridgeSetup& setup)
    : setup_(setup) {}

bool MicroRosBridge::onCreate(MicroRosContext& ctx) {
  bool ok = true;

#if BRIDGE_ENABLE_HEARTBEAT
  {
    heartbeat_.msg.data = 0;
    rcl_ret_t rc = ctx.createPublisherBestEffort(
        &heartbeat_.pub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        setup_.heartbeat_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR,
                    "[Bridge] Heartbeat publisher failed (%d)", (int)rc);
      ok = false;
    } else {
      Debug::printf(Debug::Level::INFO, "[Bridge] Heartbeat publisher -> %s",
                    setup_.heartbeat_topic);
    }
  }
#endif  // BRIDGE_ENABLE_HEARTBEAT

#if BRIDGE_ENABLE_GYRO
  if (!setup_.gyro) {
    Debug::printf(Debug::Level::ERROR,
                  "[Bridge] BRIDGE_ENABLE_GYRO=1 but gyro pointer is null");
    ok = false;
  } else {
    // __init allocates the frame_id string buffer; plain {} leaves data=nullptr
    // which micro-CDR dereferences during serialisation → crash.
    sensor_msgs__msg__Imu__init(&gyro_.msg);
    gyro_.msg.orientation_covariance[0] = -1.0;
    gyro_.msg.angular_velocity_covariance[0] = -1.0;
    gyro_.msg.linear_acceleration_covariance[0] = -1.0;

    rcl_ret_t rc = ctx.createPublisherBestEffort(
        &gyro_.pub, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        setup_.imu_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR, "[Bridge] IMU publisher failed (%d)",
                    (int)rc);
      ok = false;
    } else {
      Debug::printf(Debug::Level::INFO, "[Bridge] IMU publisher -> %s",
                    setup_.imu_topic);
    }
  }
#endif  // BRIDGE_ENABLE_GYRO

#if BRIDGE_ENABLE_BATTERY
  if (!setup_.battery) {
    Debug::printf(
        Debug::Level::ERROR,
        "[Bridge] BRIDGE_ENABLE_BATTERY=1 but battery pointer is null");
    ok = false;
  } else {
    battery_.msg.data = 0.0f;
    rcl_ret_t rc = ctx.createPublisherBestEffort(
        &battery_.pub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        setup_.battery_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR,
                    "[Bridge] Battery publisher failed (%d)", (int)rc);
      ok = false;
    } else {
      Debug::printf(Debug::Level::INFO, "[Bridge] Battery publisher -> %s",
                    setup_.battery_topic);
    }
  }
#endif  // BRIDGE_ENABLE_BATTERY

#if BRIDGE_ENABLE_SERVO
  Debug::printf(Debug::Level::WARN,
                "[Bridge] BRIDGE_ENABLE_SERVO=1 but not yet implemented");
#endif

#if BRIDGE_ENABLE_LIDAR
  if (!setup_.lidar) {
    Debug::printf(Debug::Level::ERROR,
                  "[Bridge] BRIDGE_ENABLE_LIDAR=1 but lidar pointer is null");
    ok = false;
  } else {
    // __init allocates the header.frame_id string sequence; must be called
    // before any publish or micro-CDR will dereference a nullptr.
    sensor_msgs__msg__LaserScan__init(&lidar_.msg);

    // Override the sequence data pointers with our pre-allocated buffers.
    // __init() sets data=nullptr; leaving it would cause micro-CDR to
    // dereference nullptr during serialisation → crash.
    lidar_.msg.ranges.data = lidar_.ranges_buf;
    lidar_.msg.ranges.size = 0;
    lidar_.msg.ranges.capacity = LidarPublisherState::kCapacity;
    lidar_.msg.intensities.data = lidar_.intensities_buf;
    lidar_.msg.intensities.size = 0;
    lidar_.msg.intensities.capacity = LidarPublisherState::kCapacity;

    // Physical limits for the LD14P (metres).
    lidar_.msg.range_min = 0.02f;
    lidar_.msg.range_max = 12.0f;

    rcl_ret_t rc = ctx.createPublisherBestEffort(
        &lidar_.pub,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        setup_.scan_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR,
                    "[Bridge] LaserScan publisher failed (%d)", (int)rc);
      ok = false;
    } else {
      Debug::printf(Debug::Level::INFO, "[Bridge] LaserScan publisher -> %s",
                    setup_.scan_topic);
    }
  }
#endif  // BRIDGE_ENABLE_LIDAR

#if BRIDGE_ENABLE_DEBUG
  {
    // __init allocates the rosidl String header; must be called before publish.
    std_msgs__msg__String__init(&debug_.msg);
    // Wire pre-allocated buffer so __fini() never frees it.
    debug_.msg.data.data = debug_.data_buf;
    debug_.msg.data.size = 0;
    debug_.msg.data.capacity = DebugPublisherState::kMsgLen + 1;

    rcl_ret_t rc = ctx.createPublisherBestEffort(
        &debug_.pub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        setup_.log_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR,
                    "[Bridge] Debug log publisher failed (%d)", (int)rc);
      ok = false;
    } else {
      // Open the queue after the publisher is ready.
      MicroRosDebug::open();
      Debug::printf(Debug::Level::INFO, "[Bridge] Debug log publisher -> %s",
                    setup_.log_topic);
    }
  }
#endif  // BRIDGE_ENABLE_DEBUG

  initialized_ = ok;
  return ok;
}

void MicroRosBridge::onDestroy() {
#if BRIDGE_ENABLE_HEARTBEAT
  heartbeat_.pub = rcl_get_zero_initialized_publisher();
#endif
#if BRIDGE_ENABLE_GYRO
  gyro_.pub = rcl_get_zero_initialized_publisher();
  sensor_msgs__msg__Imu__fini(&gyro_.msg);
#endif
#if BRIDGE_ENABLE_BATTERY
  battery_.pub = rcl_get_zero_initialized_publisher();
  battery_.msg = {};
#endif
#if BRIDGE_ENABLE_SERVO
  servo_.pub = rcl_get_zero_initialized_publisher();
#endif
#if BRIDGE_ENABLE_LIDAR
  // Null backing pointers before __fini() to prevent free() of our struct buffer.
  lidar_.msg.ranges.data = nullptr;
  lidar_.msg.ranges.size = 0;
  lidar_.msg.ranges.capacity = 0;
  lidar_.msg.intensities.data = nullptr;
  lidar_.msg.intensities.size = 0;
  lidar_.msg.intensities.capacity = 0;
  sensor_msgs__msg__LaserScan__fini(&lidar_.msg);
  lidar_.pub = rcl_get_zero_initialized_publisher();
#endif
#if BRIDGE_ENABLE_DEBUG
  MicroRosDebug::close();
  // Null backing pointer before __fini() to prevent free() of our struct buffer.
  debug_.msg.data.data = nullptr;
  debug_.msg.data.size = 0;
  debug_.msg.data.capacity = 0;
  std_msgs__msg__String__fini(&debug_.msg);
  debug_.pub = rcl_get_zero_initialized_publisher();
#endif
  initialized_ = false;
  Debug::printf(Debug::Level::INFO, "[Bridge] onDestroy");
}

void MicroRosBridge::publishAll() {
  if (!initialized_) return;

#if BRIDGE_ENABLE_HEARTBEAT
  if (heartbeat_.elapsed >= setup_.heartbeat_interval_ms) {
    heartbeat_.elapsed = 0;
    rcl_ret_t rc = rcl_publish(&heartbeat_.pub, &heartbeat_.msg, nullptr);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::WARN,
                    "[Bridge] Heartbeat publish failed (%d)", (int)rc);
    }
    heartbeat_.msg.data++;
  }
#endif  // BRIDGE_ENABLE_HEARTBEAT

#if BRIDGE_ENABLE_GYRO
  if (setup_.gyro && gyro_.elapsed >= setup_.imu_interval_ms) {
    gyro_.elapsed = 0;

    ImuData d = setup_.gyro->getImuData();

    gyro_.msg.orientation.x = d.gameRotationVector.i;
    gyro_.msg.orientation.y = d.gameRotationVector.j;
    gyro_.msg.orientation.z = d.gameRotationVector.k;
    gyro_.msg.orientation.w = d.gameRotationVector.real;

    gyro_.msg.angular_velocity.x = d.gyroscope.x;
    gyro_.msg.angular_velocity.y = d.gyroscope.y;
    gyro_.msg.angular_velocity.z = d.gyroscope.z;

    gyro_.msg.linear_acceleration.x = d.linearAcceleration.x;
    gyro_.msg.linear_acceleration.y = d.linearAcceleration.y;
    gyro_.msg.linear_acceleration.z = d.linearAcceleration.z;

    rcl_ret_t rc = rcl_publish(&gyro_.pub, &gyro_.msg, nullptr);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::WARN, "[Bridge] IMU publish failed (%d)",
                    (int)rc);
    }
  }
#endif  // BRIDGE_ENABLE_GYRO

#if BRIDGE_ENABLE_BATTERY
  if (setup_.battery && battery_.elapsed >= setup_.battery_interval_ms) {
    battery_.elapsed = 0;
    battery_.msg.data = setup_.battery->getVoltage();
    rcl_ret_t rc = rcl_publish(&battery_.pub, &battery_.msg, nullptr);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::WARN, "[Bridge] Battery publish failed (%d)",
                    (int)rc);
    }
  }
#endif  // BRIDGE_ENABLE_BATTERY

#if BRIDGE_ENABLE_SERVO
  // TODO
#endif

#if BRIDGE_ENABLE_LIDAR
  if (setup_.lidar && lidar_.elapsed >= setup_.scan_interval_ms) {
    LidarScanData scan = setup_.lidar->getScanData();
    if (scan.valid && scan.scan_count != lidar_.last_scan_count &&
        scan.count > 0) {
      lidar_.last_scan_count = scan.scan_count;
      lidar_.elapsed = 0;

      // Compute angle bounds from actual delivered angles. The LD14P applies
      // geometric correction so points are not perfectly uniform.
      static constexpr float kDeg2Rad = 3.14159265358979f / 180.0f;
      float a_min = scan.angles_deg[0], a_max = scan.angles_deg[0];
      for (uint16_t i = 1; i < scan.count; i++) {
        if (scan.angles_deg[i] < a_min) a_min = scan.angles_deg[i];
        if (scan.angles_deg[i] > a_max) a_max = scan.angles_deg[i];
      }
      lidar_.msg.angle_min = a_min * kDeg2Rad;
      lidar_.msg.angle_max = a_max * kDeg2Rad;
      lidar_.msg.angle_increment =
          (scan.count > 1)
              ? ((a_max - a_min) * kDeg2Rad / (scan.count - 1))
              : 0.0f;

      float freq = setup_.lidar->getCurrentScanFreqHz();
      if (freq > 0.0f) {
        lidar_.msg.scan_time = 1.0f / freq;
        lidar_.msg.time_increment = lidar_.msg.scan_time / scan.count;
      }

      uint16_t n = (scan.count < LidarPublisherState::kCapacity)
                       ? scan.count
                       : LidarPublisherState::kCapacity;
      for (uint16_t i = 0; i < n; i++) {
        lidar_.ranges_buf[i] = scan.distances_mm[i] * 0.001f;  // mm → m
        lidar_.intensities_buf[i] = scan.qualities[i];
      }
      lidar_.msg.ranges.size = n;
      lidar_.msg.intensities.size = n;

      rcl_ret_t rc = rcl_publish(&lidar_.pub, &lidar_.msg, nullptr);
      if (rc != RCL_RET_OK) {
        Debug::printf(Debug::Level::WARN,
                      "[Bridge] LaserScan publish failed (%d)", (int)rc);
      }
    }
  }
#endif  // BRIDGE_ENABLE_LIDAR

#if BRIDGE_ENABLE_DEBUG
  // Drain one queued debug message per publishAll() call.
  // publish failures are not logged here — that would recurse into the queue.
  {
    char text[DebugPublisherState::kMsgLen + 1];
    if (MicroRosDebug::dequeue(text, sizeof(text))) {
      size_t len = strnlen(text, DebugPublisherState::kMsgLen);
      memcpy(debug_.data_buf, text, len + 1);
      debug_.msg.data.size = len;
      rcl_publish(&debug_.pub, &debug_.msg, nullptr);
    }
  }
#endif  // BRIDGE_ENABLE_DEBUG
}

}  // namespace Subsystem
