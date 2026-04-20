/**
 * @file GaitRosParticipant.h
 * @date 4/3/2026
 * @brief IMicroRosParticipant that subscribes to geometry_msgs/Twist on
 *        cmd_vel and forwards velocity commands to GaitController.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * ## Behaviour
 * - Subscribes to geometry_msgs/Twist on the configured topic (default:
 *   "cmd_vel") with BEST_EFFORT QoS — matches the standard Nav2 / teleop
 *   stack expectation.
 * - On every incoming message:
 *     - Calls GaitController::setVelocity(linear.x, linear.y, angular.z).
 *     - If the magnitude of the command exceeds a small dead-band, also calls
 *       enable() so the gait starts automatically when velocity arrives.
 *     - If the command is near-zero, calls disable() for a clean stop.
 * - publishAll() is a no-op — this participant only subscribes.
 *
 * ## Singleton constraint
 * micro-ROS rclc callbacks are plain C function pointers (no closure/context).
 * GaitRosParticipant stores a static instance pointer so the callback can
 * reach the object. Only one instance may exist at a time.
 *
 * ## Usage (follows HeartbeatParticipant / MicroRosBridge pattern)
 *   static Gait::GaitRosParticipantSetup ros_setup{&gait_controller};
 *   static Gait::GaitRosParticipant      gait_ros(ros_setup);
 *   manager.registerParticipant(&gait_ros);
 */
#pragma once

#include <GaitController.h>
#include <geometry_msgs/msg/twist.h>
#include <microros_manager_robot.h>

namespace Gait {

struct GaitRosParticipantSetup {
  GaitController* gait = nullptr;
  const char* cmd_vel_topic = "cmd_vel";

  /// @brief Dead-band below which an incoming Twist is treated as a stop
  ///        command (m/s equivalent).
  float vel_dead_band = 0.001f;

  /// @brief Per-axis velocity caps applied before forwarding to the gait.
  ///        Mirrors test_sub_movement::applyVelClamp so the ROS cmd_vel path
  ///        produces the same motion as the console's `forward/move` commands.
  ///        Zero disables that axis's cap.
  float max_vx = 0.0f;    // m/s
  float max_vy = 0.0f;    // m/s
  float max_wz = 0.0f;    // rad/s

  /// @brief Combined horizontal speed cap — if sqrt(vx^2+vy^2) exceeds this
  ///        after per-axis clamping, (vx, vy) are scaled down jointly. Zero
  ///        disables.
  float max_hvel = 0.0f;  // m/s
};

class GaitRosParticipant : public Subsystem::IMicroRosParticipant {
 public:
  explicit GaitRosParticipant(const GaitRosParticipantSetup& setup);

  /// @brief Creates the cmd_vel subscription and adds it to the executor.
  ///        Returns false if the gait pointer is null or subscription fails.
  bool onCreate(Subsystem::MicroRosContext& ctx) override;

  /// @brief Zeroes the subscription handle and clears initialized_.
  ///        Does NOT call rcl_subscription_fini — RCL session already torn
  ///        down by the manager before this fires.
  void onDestroy() override;

  // publishAll() intentionally omitted — inherits the default no-op.

 private:
  GaitRosParticipantSetup setup_;
  rcl_subscription_t sub_;
  geometry_msgs__msg__Twist twist_msg_;
  bool initialized_ = false;

  // Static instance pointer — only one participant may exist at a time.
  // The micro-ROS executor callback is a plain C function pointer with no
  // context parameter, so we route through this singleton.
  static GaitRosParticipant* s_instance_;

  /// @brief Subscription callback: extract Twist fields and forward to gait.
  static void twistCb(const void* msg_in);
};

}  // namespace Gait
