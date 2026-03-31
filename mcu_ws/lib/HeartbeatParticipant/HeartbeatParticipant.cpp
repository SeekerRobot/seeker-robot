/**
 * @file HeartbeatParticipant.cpp
 * @author Aldem Pido
 */
#include "HeartbeatParticipant.h"

#include <CustomDebug.h>

namespace Subsystem {

HeartbeatParticipant::HeartbeatParticipant(
    const HeartbeatParticipantSetup& setup)
    : setup_(setup),
      publisher_(rcl_get_zero_initialized_publisher()),
      msg_() {}

bool HeartbeatParticipant::onCreate(MicroRosContext& ctx) {
  rcl_ret_t rc = ctx.createPublisherBestEffort(
      &publisher_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      setup_.topic_name);
  if (rc != RCL_RET_OK) {
    Debug::printf(Debug::Level::ERROR,
                  "[Heartbeat] createPublisher failed (%d)", (int)rc);
    return false;
  }
  msg_.data = 0;
  initialized_ = true;
  Debug::printf(Debug::Level::INFO, "[Heartbeat] onCreate OK -> %s",
                setup_.topic_name);
  return true;
}

void HeartbeatParticipant::onDestroy() {
  publisher_ = rcl_get_zero_initialized_publisher();
  initialized_ = false;
  Debug::printf(Debug::Level::INFO, "[Heartbeat] onDestroy");
}

void HeartbeatParticipant::publishAll() {
  if (!initialized_) return;
  if (elapsed_since_publish_ < setup_.publish_interval_ms) return;
  elapsed_since_publish_ = 0;
  rcl_ret_t rc = rcl_publish(&publisher_, &msg_, nullptr);
  if (rc != RCL_RET_OK) {
    Debug::printf(Debug::Level::WARN, "[Heartbeat] publish failed (%d)", (int)rc);
  }
  msg_.data++;
}

}  // namespace Subsystem
