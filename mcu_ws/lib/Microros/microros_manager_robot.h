/**
 * @file microros_manager_robot.h
 * @brief Defines the universal microros manager class
 * @author Aldem Pido
 * @date 12/12/2025
 * Edited 3/31/25
 */

#pragma once
#include <ThreadedSubsystem.h>
#include <hal_thread.h>
// microros includes
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <stdio.h>

#include "microros_setup.h"

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do {                                 \
    static volatile int64_t init = -1; \
    if (init == -1) {                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS) {    \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

namespace Subsystem {

class MicroRosContext {
 public:
  MicroRosContext(rcl_node_t* node, rclc_executor_t* executor)
      : node_(node), executor_(executor) {}

  rcl_ret_t createPublisherBestEffort(
      rcl_publisher_t* publisher,
      const rosidl_message_type_support_t* type_support,
      const char* topic_name) const {
    return rclc_publisher_init_best_effort(publisher, node_, type_support,
                                           topic_name);
  }

  rcl_ret_t createPublisherReliable(
      rcl_publisher_t* publisher,
      const rosidl_message_type_support_t* type_support,
      const char* topic_name) const {
    return rclc_publisher_init_default(publisher, node_, type_support,
                                       topic_name);
  }

  rcl_ret_t createSubscriptionBestEffort(
      rcl_subscription_t* subscription,
      const rosidl_message_type_support_t* type_support,
      const char* topic_name) const {
    return rclc_subscription_init_best_effort(subscription, node_, type_support,
                                              topic_name);
  }

  rcl_ret_t createSubscriptionReliable(
      rcl_subscription_t* subscription,
      const rosidl_message_type_support_t* type_support,
      const char* topic_name) const {
    return rclc_subscription_init_default(subscription, node_, type_support,
                                          topic_name);
  }

  rcl_ret_t addSubscription(
      rcl_subscription_t* subscription, void* message,
      rclc_subscription_callback_t callback,
      rclc_executor_handle_invocation_t invocation = ON_NEW_DATA) const {
    return rclc_executor_add_subscription(executor_, subscription, message,
                                          callback, invocation);
  }

 private:
  rcl_node_t* node_;
  rclc_executor_t* executor_;
};

/// Interface for subsystems that want to own their ROS pubs/subs
class IMicroRosParticipant {
 public:
  virtual ~IMicroRosParticipant() {}

  /// @brief
  /// @param ctx
  /// @return
  virtual bool onCreate(MicroRosContext& ctx) = 0;

  /// @brief Called when manager tears down entities; participant should clean
  /// up any resources
  virtual void onDestroy() = 0;

  /// @brief Called by the manager thread while manager transport lock is held.
  /// Default no-op for non-publishing participants.
  virtual void publishAll() {}
};

class MicrorosManagerSetup : public Classes::BaseSetup {
 public:
  const char* node_name;

  MicrorosManagerSetup(const char* _id,
                       const char* _node_name = "robot_manager")
      : Classes::BaseSetup(_id), node_name(_node_name){};
};

class MicrorosManager : public Subsystem::ThreadedSubsystem {
 public:
  ~MicrorosManager() override = default;
  MicrorosManager(const MicrorosManagerSetup& setup)
      : Subsystem::ThreadedSubsystem(setup), setup_(setup){};

  bool init() override;
  void update() override;
  void begin() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  // Register a participant; it will be created/destroyed with the manager
  void registerParticipant(IMicroRosParticipant* participant);

  // Query agent connection state
  bool isConnected() const;
  const char* getStateStr() const;

  // Set a callback invoked on connection state changes.
  // cb(true) = just connected, cb(false) = just disconnected.
  using StateCallback = void (*)(bool connected);
  void setStateCallback(StateCallback cb) { state_cb_ = cb; }

  // Returns index of participant that failed onCreate, or -1 if all succeeded.
  int lastFailedParticipant() const { return last_failed_participant_; }

 private:
  const MicrorosManagerSetup setup_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;
  rcl_allocator_t allocator_;

  // Registered participants (increase capacity as subsystems are added)
  static constexpr size_t MAX_PARTICIPANTS = 20;
  IMicroRosParticipant* participants_[MAX_PARTICIPANTS] = {nullptr};
  size_t participants_count_ = 0;

  enum State {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state_;

  static MicrorosManager* s_instance_;
  mutable Threads::Mutex mutex_;
  bool support_initialized_ = false;
  int last_failed_participant_ = -1;
  StateCallback state_cb_ = nullptr;
  bool create_entities();
  void destroy_entities();
};

}  // namespace Subsystem
