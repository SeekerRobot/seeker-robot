#include "microros_manager_robot.h"

#include <Arduino.h>
#include <CustomDebug.h>
#include <uxr/client/util/time.h>

namespace Subsystem {

MicrorosManager* MicrorosManager::s_instance_ = nullptr;

bool MicrorosManager::create_entities() {
  Debug::printf(Debug::Level::INFO, "[uROS] Creating entities...");
  allocator_ = rcl_get_default_allocator();
  support_ = {};
  node_ = rcl_get_zero_initialized_node();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
    Debug::printf(Debug::Level::ERROR, "[uROS] FAIL: rclc_support_init");
    return false;
  }
  support_initialized_ = true;
  if (rclc_node_init_default(&node_, setup_.node_name, "", &support_) !=
      RCL_RET_OK) {
    Debug::printf(Debug::Level::ERROR, "[uROS] FAIL: rclc_node_init_default");
    return false;
  }
  executor_ = rclc_executor_get_zero_initialized_executor();
  // Reserve enough executor handles for all subscription/service callbacks.
  // Publishers do not consume handles — only subscriptions, services, and
  // timers do.  16 handles supports the current set of subsystems with room
  // for growth.
  if (rclc_executor_init(&executor_, &support_.context, 24, &allocator_) !=
      RCL_RET_OK) {
    Debug::printf(Debug::Level::ERROR, "[uROS] FAIL: rclc_executor_init");
    return false;
  }
  Debug::printf(Debug::Level::INFO, "[uROS] Node + executor created");
  MicroRosContext context(&node_, &executor_);
  // Let registered participants create their pubs/subs
  for (size_t i = 0; i < participants_count_; ++i) {
    if (participants_[i] && !participants_[i]->onCreate(context)) {
      last_failed_participant_ = (int)i;
      Debug::printf(Debug::Level::ERROR,
                    "[uROS] FAIL: participant[%d] onCreate failed", (int)i);
      return false;
    }
    Debug::printf(Debug::Level::INFO, "[uROS] participant[%d] onCreate OK",
                  (int)i);
  }
  last_failed_participant_ = -1;

  Debug::printf(Debug::Level::INFO,
                "[uROS] All %d participants created successfully",
                (int)participants_count_);
  return true;
}

void MicrorosManager::destroy_entities() {
  Debug::printf(Debug::Level::INFO, "[uROS] Destroying entities...");
  if (support_initialized_) {
    rmw_context_t* rmw_context =
        rcl_context_get_rmw_context(&support_.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    // Do NOT call rcl_node_fini — it queues a DELETE_PARTICIPANT XRCE message
    // that leaks into the next session's serial stream on reconnect, causing
    // "unknown reference" errors on the agent.  rclc_support_fini closes the
    // session; the agent tears down all entities when the session ends.
    rclc_executor_fini(&executor_);
    rclc_support_fini(&support_);
    support_initialized_ = false;
  }
  node_ = rcl_get_zero_initialized_node();
  executor_ = rclc_executor_get_zero_initialized_executor();
  support_ = {};
  // Notify participants to clean up
  for (size_t i = 0; i < participants_count_; ++i) {
    if (participants_[i]) participants_[i]->onDestroy();
  }

  // Flush serial to clear any corrupt XRCE-DDS data before reconnect
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
  Serial.flush();
  while (Serial.available()) Serial.read();
  vTaskDelay(pdMS_TO_TICKS(100));
#endif
}

// No manager-owned timer; executor spin is driven from update()

bool MicrorosManager::init() {
  state_ = WAITING_AGENT;
  return true;
}

void MicrorosManager::begin() {
  s_instance_ = this;
  Debug::printf(Debug::Level::INFO, "[uROS] Setting up transport...");
  set_microros_transports();
  Debug::printf(Debug::Level::INFO,
                "[uROS] Transport ready — waiting for agent");
}

void MicrorosManager::update() {
  Threads::Scope guard(mutex_);

  State prev_state = state_;
  switch (state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        bool reachable = (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
        Debug::printf(Debug::Level::DEBUG, "[uROS] ping agent: %s",
                      reachable ? "OK" : "FAIL");
        state_ = reachable ? AGENT_AVAILABLE : WAITING_AGENT;
      });
      break;
    case AGENT_AVAILABLE:
      state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state_ == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      // IMPORTANT: Spin executor FIRST to drain any pending data (service
      // requests, subscriptions) from the UDP socket buffer.  If we ping
      // first, rmw_uros_ping_agent reads from the same socket and can
      // consume/discard non-ping XRCE-DDS packets (like service requests),
      // causing the ping to fail and tearing down the entire connection.
      rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
      // Publish all participant data under the same manager mutex hold.
      for (size_t i = 0; i < participants_count_; ++i) {
        if (participants_[i]) participants_[i]->publishAll();
      }
      // Keep-alive ping every 5s (after executor has drained the socket).
      EXECUTE_EVERY_N_MS(5000,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                                      ? AGENT_CONNECTED
                                      : AGENT_DISCONNECTED;);
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state_ = WAITING_AGENT;
      break;
    default:
      break;
  }
  if (state_ != prev_state) {
    static const char* const state_names[] = {
        "WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED",
        "AGENT_DISCONNECTED"};
    Debug::printf(Debug::Level::INFO, "[uROS] %s -> %s",
                  state_names[prev_state], state_names[state_]);
    if (state_cb_) {
      if (state_ == AGENT_CONNECTED)
        state_cb_(true);
      else if (state_ == AGENT_DISCONNECTED)
        state_cb_(false);
    }
  }
}

void MicrorosManager::pause() {
  Threads::Scope guard(mutex_);
  if (state_ == AGENT_CONNECTED) {
    destroy_entities();
  }
  state_ = WAITING_AGENT;
}

void MicrorosManager::reset() { pause(); }

const char* MicrorosManager::getInfo() {
  static const char info[] = "MicrorosManager";
  return info;
}

void MicrorosManager::registerParticipant(IMicroRosParticipant* participant) {
  if (participants_count_ <
      (sizeof(participants_) / sizeof(participants_[0]))) {
    participants_[participants_count_++] = participant;
  }
}

bool MicrorosManager::isConnected() const {
  Threads::Scope guard(mutex_);
  return state_ == AGENT_CONNECTED;
}

const char* MicrorosManager::getStateStr() const {
  Threads::Scope guard(mutex_);
  switch (state_) {
    case WAITING_AGENT:
      return "WAITING_AGENT";
    case AGENT_AVAILABLE:
      return "AGENT_AVAILABLE";
    case AGENT_CONNECTED:
      return "AGENT_CONNECTED";
    case AGENT_DISCONNECTED:
      return "AGENT_DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace Subsystem
