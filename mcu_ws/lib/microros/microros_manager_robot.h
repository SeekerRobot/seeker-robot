/**
 * @file microros_manager.h
 * @brief Defines the universal microros manager class
 * @author Aldem Pido
 * @date 12/12/2025
 */

#ifndef MICROROS_MANAGER_H
#define MICROROS_MANAGER_H
#include <BaseSubsystem.h>
// microros includes
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

#include <mutex>

#ifdef USE_FREERTOS
#include "arduino_freertos.h"
#endif

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

// Interface for subsystems that want to own their ROS pubs/subs
class IMicroRosParticipant {
 public:
  virtual ~IMicroRosParticipant() {}
  // Called when manager creates entities; participant should create its
  // pubs/subs here
  virtual bool onCreate(rcl_node_t* node, rclc_executor_t* executor) = 0;
  // Called when manager tears down entities; participant should clean up any
  // resources
  virtual void onDestroy() = 0;
};

class MicrorosManagerSetup : public Classes::BaseSetup {
 public:
  MicrorosManagerSetup(const char* _id) : Classes::BaseSetup(_id){};
};
class MicrorosManager : public Classes::BaseSubsystem {
 public:
  ~MicrorosManager() override = default;
  MicrorosManager(const MicrorosManagerSetup& setup)
      : BaseSubsystem(setup), setup_(setup){};

  bool init() override;
  void update() override;
  void begin() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;
  // Register a participant; it will be created/destroyed with the manager
  void registerParticipant(IMicroRosParticipant* participant);

#ifdef USE_FREERTOS
  // FreeRTOS task entry point — pass `this` as pvParams
  static void taskFunction(void* pvParams);

  // Create and start the micro-ROS FreeRTOS task
  void beginThreaded(uint32_t stackSize, UBaseType_t priority);
#endif

  // Mutex for thread-safe access to the executor (use with std::lock_guard)
  std::mutex& getMutex();

  // Query agent connection state
  bool isConnected() const;

 private:
  const MicrorosManagerSetup setup_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;
  rcl_allocator_t allocator_;
  // No publishers owned by the manager; subsystems own their pubs/subs
  // Cached pose/state removed as TF/state publishing is delegated
  // Registered participants
  IMicroRosParticipant* participants_[8] = {nullptr};
  size_t participants_count_ = 0;

  enum State {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state_;

  static MicrorosManager* s_instance_;
  std::mutex mutex_;
  bool create_entities();
  void destroy_entities();

 public:
  // External setters for pose and state
  void setPose(float, float, float, float, float, float) {}  // no-op
  void setState(const char*) {}                              // no-op
};

}  // namespace Subsystem

#endif
