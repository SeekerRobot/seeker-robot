#include "microros_manager_robot.h"

#include <Arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <uxr/client/util/time.h>

namespace Subsystem {

MicrorosManager* MicrorosManager::s_instance_ = nullptr;

bool MicrorosManager::create_entities() {
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK)
    return false;
  if (rclc_node_init_default(&node_, "robot_manager", "", &support_) !=
      RCL_RET_OK)
    return false;
  executor_ = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) !=
      RCL_RET_OK)
    return false;
  // Let registered participants create their pubs/subs
  for (size_t i = 0; i < participants_count_; ++i) {
    if (participants_[i] && !participants_[i]->onCreate(&node_, &executor_)) {
      return false;
    }
  }
  return true;
}

void MicrorosManager::destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rclc_executor_fini(&executor_);
  rcl_ret_t _ret_node = rcl_node_fini(&node_);
  (void)_ret_node;
  rclc_support_fini(&support_);
  // Notify participants to clean up
  for (size_t i = 0; i < participants_count_; ++i) {
    if (participants_[i]) participants_[i]->onDestroy();
  }
}

// No manager-owned timer; executor spin is driven from update()

bool MicrorosManager::init() {
  state_ = WAITING_AGENT;
  return true;
}

void MicrorosManager::begin() {
  s_instance_ = this;
  set_microros_transports();
}

void MicrorosManager::update() {
  switch (state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                      ? AGENT_AVAILABLE
                                      : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state_ == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                      ? AGENT_CONNECTED
                                      : AGENT_DISCONNECTED;);
      if (state_ == AGENT_CONNECTED) {
        std::lock_guard<std::mutex> guard(mutex_);
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state_ = WAITING_AGENT;
      break;
    default:
      break;
  }
}

void MicrorosManager::pause() {
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

#ifdef USE_FREERTOS
void MicrorosManager::taskFunction(void* pvParams) {
  auto* self = static_cast<MicrorosManager*>(pvParams);
  self->begin();
  while (true) {
    self->update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void MicrorosManager::beginThreaded(uint32_t stackSize, UBaseType_t priority) {
  xTaskCreate(taskFunction, getInfo(), stackSize, this, priority, nullptr);
}
#endif

std::mutex& MicrorosManager::getMutex() { return mutex_; }

bool MicrorosManager::isConnected() const { return state_ == AGENT_CONNECTED; }

}  // namespace Subsystem
