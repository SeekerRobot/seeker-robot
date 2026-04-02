/**
 * @file LidarSubsystem.cpp
 * @brief LD14P LiDAR subsystem implementation — UART parsing, scan assembly,
 *        and LaserScan publishing via micro-ROS.
 */
#include "LidarSubsystem.h"

#include <Arduino.h>
#include <uxr/client/util/time.h>

#include <cmath>
#include <cstring>

namespace Subsystem {

// Use Serial2 for LiDAR UART (Serial0 is debug, Serial1 may conflict)
static HardwareSerial& LidarSerial = Serial2;

// ---------------------------------------------------------------------------
// Construction / lifecycle
// ---------------------------------------------------------------------------

LidarSubsystem::LidarSubsystem(const LidarSubsystemSetup& setup)
    : Classes::BaseSubsystem(setup), setup_(setup) {}

bool LidarSubsystem::init() {
  parse_state_ = WAIT_HEADER;
  pkt_idx_ = 0;
  scan_count_ = 0;
  prev_angle_deg_ = 0.0f;
  return true;
}

void LidarSubsystem::begin() {
  LidarSerial.begin(LD14P::BAUD_RATE, SERIAL_8N1, setup_.rx_pin_,
                    setup_.tx_pin_);
}

void LidarSubsystem::update() {
  // Read all available bytes from the LiDAR UART
  while (LidarSerial.available()) {
    uint8_t byte = LidarSerial.read();
    if (parseByte(byte)) {
      // A valid packet was received — process it
      const auto& pkt = *reinterpret_cast<const LD14P::Packet*>(pkt_buf_);
      processPacket(pkt);
    }
  }
}

void LidarSubsystem::pause() {}

void LidarSubsystem::reset() { pause(); }

const char* LidarSubsystem::getInfo() {
  static const char info[] = "LidarSubsystem";
  return info;
}

// ---------------------------------------------------------------------------
// Packet parser
// ---------------------------------------------------------------------------

bool LidarSubsystem::parseByte(uint8_t byte) {
  switch (parse_state_) {
    case WAIT_HEADER:
      if (byte == LD14P::PKG_HEADER) {
        pkt_idx_ = 0;
        pkt_buf_[pkt_idx_++] = byte;
        parse_state_ = WAIT_VER_LEN;
      }
      break;

    case WAIT_VER_LEN:
      if (byte == LD14P::PKG_VER_LEN) {
        pkt_buf_[pkt_idx_++] = byte;
        parse_state_ = COLLECT_DATA;
      } else {
        parse_state_ = WAIT_HEADER;
        pkt_idx_ = 0;
      }
      break;

    case COLLECT_DATA:
      pkt_buf_[pkt_idx_++] = byte;
      if (pkt_idx_ >= LD14P::PACKET_SIZE) {
        parse_state_ = WAIT_HEADER;
        pkt_idx_ = 0;
        return LD14P::ValidatePacket(pkt_buf_);
      }
      break;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Scan assembly
// ---------------------------------------------------------------------------

void LidarSubsystem::processPacket(const LD14P::Packet& pkt) {
  float start_deg = static_cast<float>(pkt.start_angle) / 100.0f;
  float end_deg = static_cast<float>(pkt.end_angle) / 100.0f;

  // Compute angular step between the 12 points in this packet
  float diff = end_deg - start_deg;
  if (diff < 0.0f) diff += 360.0f;
  float step = diff / static_cast<float>(LD14P::POINT_PER_PACK - 1);

  for (uint8_t i = 0; i < LD14P::POINT_PER_PACK; ++i) {
    float angle_deg = start_deg + i * step;
    if (angle_deg >= 360.0f) angle_deg -= 360.0f;

    // Detect 360-degree wrap: previous angle was high, current angle is low
    if (scan_count_ > 0 && prev_angle_deg_ > 340.0f && angle_deg < 20.0f) {
      publishScan();
      scan_count_ = 0;
    }

    if (scan_count_ < MAX_SCAN_POINTS) {
      float dist_m = static_cast<float>(pkt.point[i].distance) / 1000.0f;

      // Mark out-of-range readings as 0 (will be set to inf in publish)
      ranges_[scan_count_] = dist_m;
      intensities_[scan_count_] = static_cast<float>(pkt.point[i].intensity);
      ++scan_count_;
    }

    prev_angle_deg_ = angle_deg;
  }
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

void LidarSubsystem::allocateScanArrays(uint32_t num_points) {
  // Reuse existing allocation if big enough; otherwise reallocate
  if (scan_msg_.ranges.capacity < num_points) {
    if (scan_msg_.ranges.data) free(scan_msg_.ranges.data);
    scan_msg_.ranges.data =
        static_cast<float*>(malloc(num_points * sizeof(float)));
    if (!scan_msg_.ranges.data) return;
    scan_msg_.ranges.capacity = num_points;
  }
  scan_msg_.ranges.size = num_points;

  if (scan_msg_.intensities.capacity < num_points) {
    if (scan_msg_.intensities.data) free(scan_msg_.intensities.data);
    scan_msg_.intensities.data =
        static_cast<float*>(malloc(num_points * sizeof(float)));
    if (!scan_msg_.intensities.data) return;
    scan_msg_.intensities.capacity = num_points;
  }
  scan_msg_.intensities.size = num_points;
}

void LidarSubsystem::publishScan() {
  if (!pub_.impl || !manager_ || !manager_->isConnected()) return;
  if (scan_count_ == 0) return;

  allocateScanArrays(scan_count_);
  if (!scan_msg_.ranges.data || !scan_msg_.intensities.data) return;

  // Fill header
  scan_msg_.header.frame_id.data = const_cast<char*>("lidar_link");
  scan_msg_.header.frame_id.size = strlen("lidar_link");
  scan_msg_.header.frame_id.capacity = scan_msg_.header.frame_id.size + 1;

  int64_t now_ns = static_cast<int64_t>(uxr_millis()) * 1000000LL;
  scan_msg_.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
  scan_msg_.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);

  // LaserScan fields — full 360-degree scan
  scan_msg_.angle_min = 0.0f;
  scan_msg_.angle_max = 2.0f * M_PI;
  scan_msg_.angle_increment = (2.0f * M_PI) / static_cast<float>(scan_count_);
  scan_msg_.time_increment = 0.0f;  // not computed per-point
  scan_msg_.scan_time = 0.1f;       // ~10 Hz nominal
  scan_msg_.range_min = LD14P::RANGE_MIN_M;
  scan_msg_.range_max = LD14P::RANGE_MAX_M;

  // Copy accumulated data into the message arrays
  for (uint16_t i = 0; i < scan_count_; ++i) {
    float r = ranges_[i];
    // Out-of-range readings → infinity per REP-117
    if (r < LD14P::RANGE_MIN_M || r > LD14P::RANGE_MAX_M || r == 0.0f) {
      scan_msg_.ranges.data[i] = INFINITY;
    } else {
      scan_msg_.ranges.data[i] = r;
    }
    scan_msg_.intensities.data[i] = intensities_[i];
  }

  {
    std::lock_guard<std::mutex> guard(manager_->getMutex());
    rcl_publish(&pub_, &scan_msg_, NULL);
  }
}

// ---------------------------------------------------------------------------
// IMicroRosParticipant
// ---------------------------------------------------------------------------

bool LidarSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* /*executor*/) {
  node_ = node;
  if (rclc_publisher_init_best_effort(
          &pub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
          "lidar/scan") != RCL_RET_OK) {
    return false;
  }
  return true;
}

void LidarSubsystem::onDestroy() {
  if (pub_.impl) {
    rcl_publisher_fini(&pub_, node_);
  }
  if (scan_msg_.ranges.data) {
    free(scan_msg_.ranges.data);
    scan_msg_.ranges.data = nullptr;
    scan_msg_.ranges.size = 0;
    scan_msg_.ranges.capacity = 0;
  }
  if (scan_msg_.intensities.data) {
    free(scan_msg_.intensities.data);
    scan_msg_.intensities.data = nullptr;
    scan_msg_.intensities.size = 0;
    scan_msg_.intensities.capacity = 0;
  }
}

// ---------------------------------------------------------------------------
// FreeRTOS threading
// ---------------------------------------------------------------------------

#ifdef USE_FREERTOS
void LidarSubsystem::taskFunction(void* pvParams) {
  auto* self = static_cast<LidarSubsystem*>(pvParams);
  self->begin();
  while (true) {
    self->update();
    vTaskDelay(pdMS_TO_TICKS(5));  // 200 Hz poll — fast enough for 230400 baud
  }
}

void LidarSubsystem::beginThreaded(uint32_t stackSize, UBaseType_t priority,
                                   BaseType_t core) {
  xTaskCreatePinnedToCore(taskFunction, getInfo(), stackSize, this, priority,
                          nullptr, core);
}
#endif

}  // namespace Subsystem
