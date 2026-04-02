/**
 * @file CameraSubscriber.h
 * @brief Camera subscriber interface for SLAM — subscribes to
 *        sensor_msgs/msg/CompressedImage on /camera/image/compressed.
 *
 * This subsystem only defines the ROS interface expected by the SLAM pipeline.
 * The actual camera driver is implemented elsewhere.  On the MCU side, this
 * subscriber monitors whether frames are being received and exposes lightweight
 * status information (frame count, reception flag).  It does NOT buffer full
 * image data to conserve ESP32 RAM.
 */
#pragma once

#include <BaseSubsystem.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <uxr/client/util/time.h>

#include "microros_manager_robot.h"

namespace Subsystem {

class CameraSubscriberSetup : public Classes::BaseSetup {
 public:
  CameraSubscriberSetup(const char* _id) : Classes::BaseSetup(_id) {}
};

class CameraSubscriber : public IMicroRosParticipant,
                         public Classes::BaseSubsystem {
 public:
  explicit CameraSubscriber(const CameraSubscriberSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  // BaseSubsystem lifecycle
  bool init() override { return true; }
  void begin() override {}
  void update() override {
    // Nothing to poll — the executor drives the subscription callback.
    // We can use update() to check for stale frames (optional watchdog).
    if (last_frame_ms_ > 0) {
      uint64_t now = uxr_millis();
      receiving_ = (now - last_frame_ms_) < 2000;  // 2 s timeout
    }
  }
  void pause() override {}
  void reset() override { pause(); }
  const char* getInfo() override {
    static const char info[] = "CameraSubscriber";
    return info;
  }

  void setManager(MicrorosManager* manager) { manager_ = manager; }

  /// Returns true if frames have been received within the last 2 seconds.
  bool isReceiving() const { return receiving_; }

  /// Total number of frames received since connection.
  uint32_t getFrameCount() const { return frame_count_; }

  // IMicroRosParticipant -------------------------------------------------

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;
    s_instance_ = this;
    if (rclc_subscription_init_best_effort(
            &sub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
            "camera/image/compressed") != RCL_RET_OK) {
      return false;
    }

    // Allocate a minimal message — we only inspect metadata, not pixel data.
    // The format string needs a small buffer; image data gets a tiny stub
    // so the deserialiser has somewhere to write (micro-ROS will truncate
    // to the capacity we provide, which is fine for monitoring).
    msg_.format.data = format_buf_;
    msg_.format.size = 0;
    msg_.format.capacity = sizeof(format_buf_);

    msg_.data.data = img_stub_;
    msg_.data.size = 0;
    msg_.data.capacity = sizeof(img_stub_);

    if (rclc_executor_add_subscription(executor, &sub_, &msg_,
                                       &CameraSubscriber::subscriptionCallback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }
    return true;
  }

  void onDestroy() override {
    if (sub_.impl) {
      rcl_subscription_fini(&sub_, node_);
    }
    frame_count_ = 0;
    receiving_ = false;
    last_frame_ms_ = 0;
  }

#ifdef USE_FREERTOS
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<CameraSubscriber*>(pvParams);
    while (true) {
      self->update();
      vTaskDelay(pdMS_TO_TICKS(100));  // Low-priority watchdog check
    }
  }

  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     BaseType_t core = 1) {
    xTaskCreatePinnedToCore(taskFunction, getInfo(), stackSize, this, priority,
                            nullptr, core);
  }
#endif

 private:
  static void subscriptionCallback(const void* msg_in) {
    // The callback is invoked by the executor on message reception.
    // We cannot easily get `this` from a C callback, so we use the
    // singleton manager to find this subscriber.  However, the simplest
    // approach is a file-scoped pointer set during onCreate.
    if (s_instance_) {
      s_instance_->onFrameReceived();
    }
  }

  void onFrameReceived() {
    ++frame_count_;
    last_frame_ms_ = uxr_millis();
    receiving_ = true;
  }

  // Allow the static callback to reach this instance
  static inline CameraSubscriber* s_instance_ = nullptr;

  const CameraSubscriberSetup setup_;
  MicrorosManager* manager_ = nullptr;
  rcl_subscription_t sub_{};
  sensor_msgs__msg__CompressedImage msg_{};
  rcl_node_t* node_ = nullptr;

  // Small fixed buffers so micro-ROS has somewhere to deserialise into
  // without eating ESP32 heap.  Actual image data is discarded.
  char format_buf_[32]{};
  uint8_t img_stub_[64]{};

  uint32_t frame_count_ = 0;
  uint64_t last_frame_ms_ = 0;
  bool receiving_ = false;
};

}  // namespace Subsystem
