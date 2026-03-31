/**
 * @file HeartbeatParticipant.h
 * @author Aldem Pido
 * @brief IMicroRosParticipant that publishes an incrementing counter heartbeat.
 *
 * Follows the participant contract:
 *  - publishAll() is called by MicrorosManager under the transport lock;
 *    this class never acquires the transport mutex itself.
 *  - All data is written exclusively from publishAll() (manager thread), so no
 *    participant-side data mutex is needed.
 *  - onDestroy() only resets local state — the RCL session is already closed
 *    by the manager before this is called.
 */
#pragma once

#include <elapsedMillis.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/int32.h>

namespace Subsystem {

struct HeartbeatParticipantSetup {
  const char* topic_name;
  uint32_t publish_interval_ms;

  explicit HeartbeatParticipantSetup(const char* topic = "mcu/heartbeat",
                                     uint32_t interval_ms = 1000)
      : topic_name(topic), publish_interval_ms(interval_ms) {}
};

class HeartbeatParticipant : public IMicroRosParticipant {
 public:
  explicit HeartbeatParticipant(const HeartbeatParticipantSetup& setup);

  /// @brief Creates the std_msgs/Int32 publisher. Resets the counter and marks
  ///        the participant as initialized on success.
  bool onCreate(MicroRosContext& ctx) override;

  /// @brief Zeros the publisher handle and clears initialized_. Does NOT call
  ///        rcl_publisher_fini — the RCL session is already torn down by the
  ///        manager before this is invoked.
  void onDestroy() override;

  /// @brief Publishes the current counter if the publish interval has elapsed,
  ///        then increments. Rate-limited by publish_interval_ms; returns
  ///        immediately when not yet due. Called by manager under transport lock.
  void publishAll() override;

 private:
  const HeartbeatParticipantSetup setup_;

  rcl_publisher_t publisher_;
  std_msgs__msg__Int32 msg_;

  /// Tracks time since last publish. Not reset in onCreate() so the first
  /// publish after connect/reconnect fires immediately.
  elapsedMillis elapsed_since_publish_;

  bool initialized_ = false;
};

}  // namespace Subsystem
