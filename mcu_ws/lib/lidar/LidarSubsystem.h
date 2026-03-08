/**
 * @file LidarSubsystem.h
 * @brief LD14P LiDAR subsystem — reads UART scan data and publishes
 *        sensor_msgs/msg/LaserScan via micro-ROS.
 */
#pragma once

#include <BaseSubsystem.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "ld14p_protocol.h"
#include "microros_manager_robot.h"

namespace Subsystem {

/// Maximum points that can be accumulated for a single 360-degree revolution.
static constexpr uint16_t MAX_SCAN_POINTS = 900;

/// Default UART pins for the LD14P (ESP32 GPIO numbers).
static constexpr int LIDAR_RX_PIN = 7;  // D7
static constexpr int LIDAR_TX_PIN = 6;  // D6

class LidarSubsystemSetup : public Classes::BaseSetup {
 public:
  LidarSubsystemSetup(const char* _id, int rx_pin = LIDAR_RX_PIN,
                      int tx_pin = LIDAR_TX_PIN)
      : Classes::BaseSetup(_id), rx_pin_(rx_pin), tx_pin_(tx_pin) {}
  int rx_pin_;
  int tx_pin_;
};

class LidarSubsystem : public IMicroRosParticipant,
                       public Classes::BaseSubsystem {
 public:
  explicit LidarSubsystem(const LidarSubsystemSetup& setup);

  // BaseSubsystem lifecycle
  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  void setManager(MicrorosManager* manager) { manager_ = manager; }

  // IMicroRosParticipant
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

#ifdef USE_FREERTOS
  static void taskFunction(void* pvParams);
  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     BaseType_t core = 1);
#endif

 private:
  /// Feed one byte into the packet parser; returns true when a full valid
  /// packet has been assembled.
  bool parseByte(uint8_t byte);

  /// Process a validated LD14P packet — extract points into the scan buffer.
  void processPacket(const LD14P::Packet& pkt);

  /// Publish the accumulated 360-degree scan as a LaserScan message.
  void publishScan();

  /// Allocate the ranges/intensities arrays in the LaserScan message.
  void allocateScanArrays(uint32_t num_points);

  const LidarSubsystemSetup setup_;
  MicrorosManager* manager_ = nullptr;
  rcl_publisher_t pub_{};
  sensor_msgs__msg__LaserScan scan_msg_{};
  rcl_node_t* node_ = nullptr;

  // Packet parser state
  enum ParseState : uint8_t { WAIT_HEADER, WAIT_VER_LEN, COLLECT_DATA };
  ParseState parse_state_ = WAIT_HEADER;
  uint8_t pkt_buf_[LD14P::PACKET_SIZE]{};
  uint8_t pkt_idx_ = 0;

  // Scan accumulator — one full revolution
  float ranges_[MAX_SCAN_POINTS]{};
  float intensities_[MAX_SCAN_POINTS]{};
  uint16_t scan_count_ = 0;      ///< Points accumulated so far
  float prev_angle_deg_ = 0.0f;  ///< Previous point angle for wrap detection

  // Timing
  uint64_t last_publish_ms_ = 0;
};

}  // namespace Subsystem
