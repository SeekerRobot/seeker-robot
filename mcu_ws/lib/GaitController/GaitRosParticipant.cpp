/**
 * @file GaitRosParticipant.cpp
 * @date 4/3/2026
 * @brief GaitRosParticipant implementation.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include "GaitRosParticipant.h"

#include <CustomDebug.h>

#include <cmath>

namespace Gait {

GaitRosParticipant* GaitRosParticipant::s_instance_ = nullptr;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

GaitRosParticipant::GaitRosParticipant(const GaitRosParticipantSetup& setup)
    : setup_(setup), sub_(rcl_get_zero_initialized_subscription()) {
  s_instance_ = this;
}

// ---------------------------------------------------------------------------
// IMicroRosParticipant overrides
// ---------------------------------------------------------------------------

bool GaitRosParticipant::onCreate(Subsystem::MicroRosContext& ctx) {
  if (!setup_.gait) {
    Debug::printf(Debug::Level::ERROR,
                  "[GaitRos] onCreate failed — gait pointer is null");
    return false;
  }

  rcl_ret_t rc = ctx.createSubscriptionBestEffort(
      &sub_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      setup_.cmd_vel_topic);

  if (rc != RCL_RET_OK) {
    Debug::printf(Debug::Level::ERROR,
                  "[GaitRos] createSubscription failed (%d)", (int)rc);
    return false;
  }

  rc = ctx.addSubscription(&sub_, &twist_msg_, twistCb);
  if (rc != RCL_RET_OK) {
    Debug::printf(Debug::Level::ERROR,
                  "[GaitRos] addSubscription failed (%d)", (int)rc);
    return false;
  }

  initialized_ = true;
  Debug::printf(Debug::Level::INFO, "[GaitRos] onCreate OK -> %s",
                setup_.cmd_vel_topic);
  return true;
}

void GaitRosParticipant::onDestroy() {
  sub_         = rcl_get_zero_initialized_subscription();
  initialized_ = false;
  Debug::printf(Debug::Level::INFO, "[GaitRos] onDestroy");
}

// ---------------------------------------------------------------------------
// Static callback
// ---------------------------------------------------------------------------

void GaitRosParticipant::twistCb(const void* msg_in) {
  if (!s_instance_ || !s_instance_->initialized_) return;
  if (!s_instance_->setup_.gait) return;

  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  float vx = static_cast<float>(msg->linear.x);
  float vy = static_cast<float>(msg->linear.y);
  float wz = static_cast<float>(msg->angular.z);

  s_instance_->setup_.gait->setVelocity(vx, vy, wz);

  float magnitude = fabsf(vx) + fabsf(vy) + fabsf(wz);
  if (magnitude > s_instance_->setup_.vel_dead_band) {
    s_instance_->setup_.gait->enable();
  } else {
    s_instance_->setup_.gait->disable();
  }
}

}  // namespace Gait
