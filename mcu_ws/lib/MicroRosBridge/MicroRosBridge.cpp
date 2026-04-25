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
    Debug::printf(
        Debug::Level::WARN,
        "[Bridge] BRIDGE_ENABLE_GYRO=1 but gyro pointer is null — skipping");
  } else {
    // __init allocates the frame_id string buffer; plain {} leaves data=nullptr
    // which micro-CDR dereferences during serialisation → crash.
    sensor_msgs__msg__Imu__init(&gyro_.msg);
    // Wire frame_id to our pre-allocated buffer (same pattern as LiDAR).
    gyro_.msg.header.frame_id.data = gyro_.frame_id_buf;
    gyro_.msg.header.frame_id.size = 9;  // strlen("base_link")
    gyro_.msg.header.frame_id.capacity = sizeof(gyro_.frame_id_buf);
    // Diagonal covariance matrices (row-major 3x3, indices 0/4/8 are xx/yy/zz).
    // BNO085 game rotation vector: ~1° RMS → 0.0003 rad² per axis.
    gyro_.msg.orientation_covariance[0] = 0.0003;
    gyro_.msg.orientation_covariance[4] = 0.0003;
    gyro_.msg.orientation_covariance[8] = 0.0003;
    // Gyroscope: ~0.01 rad/s RMS → 0.0001 (rad/s)² per axis.
    gyro_.msg.angular_velocity_covariance[0] = 0.0001;
    gyro_.msg.angular_velocity_covariance[4] = 0.0001;
    gyro_.msg.angular_velocity_covariance[8] = 0.0001;
    // Linear acceleration: ~0.1 m/s² RMS → 0.01 (m/s²)² per axis.
    gyro_.msg.linear_acceleration_covariance[0] = 0.01;
    gyro_.msg.linear_acceleration_covariance[4] = 0.01;
    gyro_.msg.linear_acceleration_covariance[8] = 0.01;

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
    Debug::printf(Debug::Level::WARN,
                  "[Bridge] BRIDGE_ENABLE_BATTERY=1 but battery pointer is "
                  "null — skipping");
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
    Debug::printf(
        Debug::Level::WARN,
        "[Bridge] BRIDGE_ENABLE_LIDAR=1 but lidar pointer is null — skipping");
  } else {
    // Without __init(), header.frame_id.data is nullptr → micro-CDR crash on
    // publish.
    mcu_msgs__msg__CompactScan__init(&lidar_.msg);
    // Wire frame_id to our pre-allocated buffer (__init allocates 1 byte; we
    // override before any publish so __fini never sees our pointer).
    lidar_.msg.header.frame_id.data = lidar_.frame_id_buf;
    lidar_.msg.header.frame_id.size = 10;  // strlen("lidar_link")
    lidar_.msg.header.frame_id.capacity = sizeof(lidar_.frame_id_buf);
    // Wire pre-allocated buffer so __fini() never frees it.
    lidar_.msg.ranges_mm.data = lidar_.ranges_buf;
    lidar_.msg.ranges_mm.size = 0;
    lidar_.msg.ranges_mm.capacity = kLidarMaxChunkSize;

    // Physical limits for the LD14P (metres).
    lidar_.msg.range_min = 0.02f;
    lidar_.msg.range_max = 12.0f;
    lidar_.msg.chunk_count = kLidarChunkCount;

    rcl_ret_t rc = ctx.createPublisherBestEffort(
        &lidar_.pub, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, CompactScan),
        setup_.scan_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR,
                    "[Bridge] CompactScan publisher failed (%d)", (int)rc);
      ok = false;
    } else {
      Debug::printf(Debug::Level::INFO, "[Bridge] CompactScan publisher -> %s",
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
    debug_.msg.data.capacity = MicroRosDebug::kMsgLen + 1;

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
  // Null backing pointer before __fini() to prevent free() of our struct
  // buffer.
  gyro_.msg.header.frame_id.data = nullptr;
  gyro_.msg.header.frame_id.size = 0;
  gyro_.msg.header.frame_id.capacity = 0;
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
  // Null backing pointers before __fini() to prevent free() of our struct
  // buffer.
  lidar_.msg.header.frame_id.data = nullptr;
  lidar_.msg.header.frame_id.size = 0;
  lidar_.msg.header.frame_id.capacity = 0;
  lidar_.msg.ranges_mm.data = nullptr;
  lidar_.msg.ranges_mm.size = 0;
  lidar_.msg.ranges_mm.capacity = 0;
  mcu_msgs__msg__CompactScan__fini(&lidar_.msg);
  lidar_.pub = rcl_get_zero_initialized_publisher();
#endif
#if BRIDGE_ENABLE_DEBUG
  MicroRosDebug::close();
  // Null backing pointer before __fini() to prevent free() of our struct
  // buffer.
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

    int64_t now_ns = rmw_uros_epoch_nanos();
    gyro_.msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
    gyro_.msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

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
    static LidarScanData scan;
    setup_.lidar->getScanData(scan);
    if (scan.valid && scan.scan_count != lidar_.last_scan_count &&
        scan.count > 0) {
      lidar_.last_scan_count = scan.scan_count;
      lidar_.elapsed = 0;

      // Split the full rotation into kLidarChunkCount CompactScan messages so
      // each serialises to ~820 B (single IP frame) and the host reassembles
      // by scan_id. Ranges go out as uint16 mm (0 == no return), halving the
      // payload vs float32 metres.
      static constexpr float kDeg2Rad = 3.14159265358979f / 180.0f;
      static constexpr float kTwoPi = 6.28318530717958647f;
      const uint16_t n =
          (scan.count < kLidarMaxPoints) ? scan.count : kLidarMaxPoints;

      const float freq = setup_.lidar->getCurrentScanFreqHz();
      const float scan_time = (freq > 0.0f) ? (1.0f / freq) : 0.0f;
      // LD14P covers a full 2π rotation per scan. Deriving angle_increment
      // from (angles[n-1] - angles[0])/(n-1) breaks when the driver's angle
      // stream wraps across 0°/360° mid-scan — computed increment collapses
      // toward zero and every ray projects to the same bearing (looks like a
      // straight line in RViz). Uniform 2π/n is wrap-safe.
      const float full_inc_rad = (n > 1) ? (kTwoPi / (float)n) : 0.0f;
      // Absolute start bearing of ranges[0] in the lidar frame. Per-chunk
      // angle_min is derived from this + start * inc so mid-scan wraps don't
      // leak into chunk metadata.
      const float scan_angle_min_rad =
          (n > 0) ? (scan.angles_deg[0] * kDeg2Rad) : 0.0f;

      // Shared across chunks so the host can correlate.
      const int64_t now_ns = rmw_uros_epoch_nanos();
      lidar_.msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
      lidar_.msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
      lidar_.msg.scan_id = (uint16_t)scan.scan_count;
      lidar_.msg.chunk_count = kLidarChunkCount;
      lidar_.msg.scan_time = scan_time;
      lidar_.msg.time_increment = (n > 0) ? (scan_time / n) : 0.0f;
      lidar_.msg.angle_increment = full_inc_rad;

      for (uint8_t chunk = 0; chunk < kLidarChunkCount; ++chunk) {
        const uint16_t start =
            (uint16_t)((uint32_t)chunk * n / kLidarChunkCount);
        const uint16_t end =
            (uint16_t)((uint32_t)(chunk + 1) * n / kLidarChunkCount);
        const uint16_t chunk_n = (end > start) ? (end - start) : 0;
        if (chunk_n == 0) continue;

        for (uint16_t i = start, j = 0; i < end; ++i, ++j) {
          // LD14P reports 0 mm for no-return; pass through as the sentinel.
          float d = scan.distances_mm[i];
          lidar_.ranges_buf[j] = (d > 65535.0f || d < 0.0f)
                                     ? (uint16_t)0
                                     : (uint16_t)(d + 0.5f);
        }

        lidar_.msg.chunk_index = chunk;
        lidar_.msg.angle_min =
            scan_angle_min_rad + (float)start * full_inc_rad;
        lidar_.msg.angle_max =
            scan_angle_min_rad + (float)(end - 1) * full_inc_rad;
        lidar_.msg.ranges_mm.size = chunk_n;

        rcl_ret_t rc = rcl_publish(&lidar_.pub, &lidar_.msg, nullptr);
        if (rc != RCL_RET_OK) {
          Debug::printf(Debug::Level::WARN,
                        "[Bridge] CompactScan publish failed chunk=%u (%d)",
                        (unsigned)chunk, (int)rc);
        }
      }
    }
  }
#endif  // BRIDGE_ENABLE_LIDAR

#if BRIDGE_ENABLE_DEBUG
  // publish failures are not logged here — that would recurse into the queue.
  if (MicroRosDebug::dequeue(debug_.data_buf, sizeof(debug_.data_buf))) {
    debug_.msg.data.size =
        strnlen(debug_.data_buf, sizeof(debug_.data_buf) - 1);
    rcl_publish(&debug_.pub, &debug_.msg, nullptr);
  }
#endif  // BRIDGE_ENABLE_DEBUG
}

}  // namespace Subsystem
