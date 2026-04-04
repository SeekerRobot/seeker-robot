/**
 * @file MicroRosBridge.cpp
 * @author Aldem Pido
 * @date 3/31/2026
 */
#include "MicroRosBridge.h"

#include <CustomDebug.h>

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
}

}  // namespace Subsystem
