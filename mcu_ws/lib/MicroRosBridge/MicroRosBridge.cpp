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

#if BRIDGE_ENABLE_OLED
MicroRosBridge* MicroRosBridge::s_instance_ = nullptr;
#endif

MicroRosBridge::MicroRosBridge(const MicroRosBridgeSetup& setup)
    : setup_(setup) {
#if BRIDGE_ENABLE_OLED
  s_instance_ = this;
#endif
}

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
    Debug::printf(Debug::Level::WARN,
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
                  "[Bridge] BRIDGE_ENABLE_BATTERY=1 but battery pointer is null — skipping");
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
    Debug::printf(Debug::Level::WARN,
                  "[Bridge] BRIDGE_ENABLE_LIDAR=1 but lidar pointer is null — skipping");
  } else {
    // Without __init(), header.frame_id.data is nullptr → micro-CDR crash on
    // publish.
    sensor_msgs__msg__LaserScan__init(&lidar_.msg);
    // Wire frame_id to our pre-allocated buffer (__init allocates 1 byte; we
    // override before any publish so __fini never sees our pointer).
    lidar_.msg.header.frame_id.data = lidar_.frame_id_buf;
    lidar_.msg.header.frame_id.size = 10;  // strlen("lidar_link")
    lidar_.msg.header.frame_id.capacity = sizeof(lidar_.frame_id_buf);
    // Wire pre-allocated buffers so __fini() never frees them.
    lidar_.msg.ranges.data = lidar_.ranges_buf;
    lidar_.msg.ranges.size = 0;
    lidar_.msg.ranges.capacity = kLidarMaxPoints;
    lidar_.msg.intensities.data = lidar_.intensities_buf;
    lidar_.msg.intensities.size = 0;
    lidar_.msg.intensities.capacity = kLidarMaxPoints;

    // Physical limits for the LD14P (metres).
    lidar_.msg.range_min = 0.02f;
    lidar_.msg.range_max = 12.0f;

    rcl_ret_t rc = ctx.createPublisherReliable(
        &lidar_.pub, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
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

#if BRIDGE_ENABLE_OLED
  if (!setup_.oled) {
    Debug::printf(Debug::Level::WARN,
                  "[Bridge] BRIDGE_ENABLE_OLED=1 but oled pointer is null — skipping");
  } else {
    // Create FreeRTOS queue for bridge -> display frame handoff.
    // Depth kQueueDepth items of OledFrameItem (1024 bytes each).
    // Total RAM: ~10 KB. Callback drops new frames on full.
    if (!oled_.queue) {
      oled_.queue =
          xQueueCreate(OledSubscriberState::kQueueDepth, sizeof(OledFrameItem));
      if (!oled_.queue) {
        Debug::printf(Debug::Level::ERROR, "[Bridge] OLED queue alloc failed");
        ok = false;
      }
    }

    // __init allocates rosidl sequence metadata; wire our pre-allocated rx_buf
    // so micro-CDR deserializes framebuffer bytes in place without malloc.
    mcu_msgs__msg__OledFrame__init(&oled_.msg);
    oled_.msg.framebuffer.data = oled_.rx_buf;
    oled_.msg.framebuffer.size = 0;
    oled_.msg.framebuffer.capacity = sizeof(oled_.rx_buf);

    rcl_ret_t rc = ctx.createSubscriptionBestEffort(
        &oled_.sub, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, OledFrame),
        setup_.lcd_topic);
    if (rc != RCL_RET_OK) {
      Debug::printf(Debug::Level::ERROR,
                    "[Bridge] OLED createSubscription failed (%d)", (int)rc);
      ok = false;
    } else {
      rc = ctx.addSubscription(&oled_.sub, &oled_.msg,
                               &MicroRosBridge::oledFrameCb);
      if (rc != RCL_RET_OK) {
        Debug::printf(Debug::Level::ERROR,
                      "[Bridge] OLED addSubscription failed (%d)", (int)rc);
        ok = false;
      } else {
        Debug::printf(Debug::Level::INFO,
                      "[Bridge] OLED subscriber <- %s (10 Hz cap, depth=%u)",
                      setup_.lcd_topic,
                      (unsigned)OledSubscriberState::kQueueDepth);
      }
    }
  }
#endif  // BRIDGE_ENABLE_OLED

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
  // Null backing pointer before __fini() to prevent free() of our struct
  // buffer.
  debug_.msg.data.data = nullptr;
  debug_.msg.data.size = 0;
  debug_.msg.data.capacity = 0;
  std_msgs__msg__String__fini(&debug_.msg);
  debug_.pub = rcl_get_zero_initialized_publisher();
#endif
#if BRIDGE_ENABLE_OLED
  if (setup_.oled) {
    // Null backing pointer before __fini() so rosidl doesn't free our rx_buf.
    oled_.msg.framebuffer.data = nullptr;
    oled_.msg.framebuffer.size = 0;
    oled_.msg.framebuffer.capacity = 0;
    mcu_msgs__msg__OledFrame__fini(&oled_.msg);
    oled_.sub = rcl_get_zero_initialized_subscription();
    // Keep the queue alive across reconnects — only freed on program exit.
    // This preserves any in-flight frames and avoids repeated alloc/free.
  }
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

      // LD14P applies geometric correction so angles are not perfectly uniform;
      // compute actual bounds in a single pass while converting scan data.
      // Stride-3 downsample: 720 pts → 240 pts (~1980 B), fits in the
      // 2048-byte micro-ROS stream buffer (MTU=512 × STREAM_HISTORY=4).
      static constexpr float kDeg2Rad = 3.14159265358979f / 180.0f;
      static constexpr uint16_t kStride = 3;
      uint16_t n =
          (scan.count < kLidarMaxPoints) ? scan.count : kLidarMaxPoints;
      uint16_t n_out = 0;
      float a_min = scan.angles_deg[0], a_max = scan.angles_deg[0];
      for (uint16_t i = 0; i < n; i += kStride, n_out++) {
        if (scan.angles_deg[i] < a_min) a_min = scan.angles_deg[i];
        if (scan.angles_deg[i] > a_max) a_max = scan.angles_deg[i];
        lidar_.ranges_buf[n_out] = scan.distances_mm[i] * 0.001f;  // mm → m
        lidar_.intensities_buf[n_out] = scan.qualities[i];
      }
      lidar_.msg.angle_min = a_min * kDeg2Rad;
      lidar_.msg.angle_max = a_max * kDeg2Rad;
      lidar_.msg.angle_increment =
          (n_out > 1) ? ((a_max - a_min) * kDeg2Rad / (n_out - 1)) : 0.0f;

      float freq = setup_.lidar->getCurrentScanFreqHz();
      if (freq > 0.0f) {
        lidar_.msg.scan_time = 1.0f / freq;
        lidar_.msg.time_increment = lidar_.msg.scan_time / n_out;
      }

      lidar_.msg.ranges.size = n_out;
      lidar_.msg.intensities.size = n_out;

      int64_t now_ns = rmw_uros_epoch_nanos();
      lidar_.msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
      lidar_.msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

      rcl_ret_t rc = rcl_publish(&lidar_.pub, &lidar_.msg, nullptr);
      if (rc != RCL_RET_OK) {
        Debug::printf(Debug::Level::WARN,
                      "[Bridge] LaserScan publish failed (%d)", (int)rc);
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

#if BRIDGE_ENABLE_OLED
  // Drain one frame per kMinIntervalMs (hard 10 Hz cap). Extra frames sit in
  // the queue until the next tick; if the queue fills, the callback drops new
  // frames. The OledSubsystem's own 10 Hz update loop + dirty flag provides a
  // second rate-limiting layer.
  if (setup_.oled && oled_.queue &&
      oled_.elapsed >= OledSubscriberState::kMinIntervalMs) {
    static OledFrameItem item;  // static: reused across calls, 1 KB off-stack
    if (xQueueReceive(oled_.queue, &item, 0) == pdTRUE) {
      oled_.elapsed = 0;
      setup_.oled->setFramebuffer(item.data);
    }
  }
#endif  // BRIDGE_ENABLE_OLED
}

#if BRIDGE_ENABLE_OLED
// Subscriber callback — runs in the micro-ROS executor (manager) task.
// Must be non-blocking: copies the frame into a FreeRTOS queue and returns.
// Drops the frame if the queue is full (no wait).
void MicroRosBridge::oledFrameCb(const void* msg_in) {
  Debug::printf(Debug::Level::DEBUG, "[Bridge] oledFrameCb fired");
  if (!s_instance_ || !s_instance_->initialized_) {
    Debug::printf(Debug::Level::WARN, "[Bridge] oledFrameCb: not initialized");
    return;
  }
  if (!s_instance_->oled_.queue) {
    Debug::printf(Debug::Level::WARN, "[Bridge] oledFrameCb: no queue");
    return;
  }

  const auto* msg = static_cast<const mcu_msgs__msg__OledFrame*>(msg_in);
  if (!msg || !msg->framebuffer.data) {
    Debug::printf(Debug::Level::WARN, "[Bridge] oledFrameCb: null msg");
    return;
  }

  Debug::printf(Debug::Level::DEBUG, "[Bridge] OLED frame size=%u",
                (unsigned)msg->framebuffer.size);

  // Only accept exactly-sized frames. Drop anything else to avoid partial
  // display corruption.
  if (msg->framebuffer.size != sizeof(OledFrameItem::data)) {
    Debug::printf(Debug::Level::WARN,
                  "[Bridge] OLED frame wrong size (%u != %u), dropping",
                  (unsigned)msg->framebuffer.size,
                  (unsigned)sizeof(OledFrameItem::data));
    return;
  }

  OledFrameItem item;
  memcpy(item.data, msg->framebuffer.data, sizeof(item.data));
  // xQueueSend with 0 ticks timeout — non-blocking, drops on full.
  if (xQueueSend(s_instance_->oled_.queue, &item, 0) != pdTRUE) {
    // Queue full — frame dropped. Not logged to avoid log spam.
  }
}
#endif  // BRIDGE_ENABLE_OLED

}  // namespace Subsystem
