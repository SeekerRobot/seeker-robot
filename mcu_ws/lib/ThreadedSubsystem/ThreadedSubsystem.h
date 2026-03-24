/**
 * @file ThreadedSubsystem.h
 * @author Aldem Pido
 * @date 3/24/2026
 * @brief Adds support for a Threaded Subsystem
 */
#pragma once

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <elapsedMillis.h>

namespace Subsystem {
class ThreadedSubsystem : public Classes::BaseSubsystem {
 public:
  explicit ThreadedSubsystem(const Classes::BaseSetup& setup)
      : Classes::BaseSubsystem(setup) {}

  // Inherited Pure Virtual BaseSubsystem functions redefined here for clarity
  virtual void update() = 0;
  virtual void begin() = 0;
  virtual void pause() = 0;
  virtual void reset() = 0;

  /// @brief Begins a pinned-to-core threaded task, utilizing defined update()
  /// and begin() functions.
  /// @param stackSize Stack size in words (word = 4 bytes).
  /// @param priority Priority. Higher number = higher priority.
  /// @param updateDelayMs Delay between updates. Uses relative scheduling, not
  /// absolute.
  /// @param core Core to pin to. For ESP32, core 0 or core 1.
  void beginThreadedPinned(uint32_t stackSize, int priority,
                           uint32_t updateDelayMs, int core) {
    task_delay_ms_ = updateDelayMs;
    BaseType_t rc =
        xTaskCreatePinnedToCore(taskFunction, setup_.getId(), stackSize, this,
                                priority, &task_handle_, core);
    if (rc != pdPASS || task_handle_ == nullptr) {
      // TODO: add debug statement here
    } else {
      // TODO: add debug statement here
    }
  }

 private:
  uint32_t task_delay_ms_ = 20;
  TaskHandle_t task_handle_ = nullptr;

  static void taskFunction(void* pv) {
    auto* self = static_cast<ThreadedSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
    }
  }
};
};  // namespace Subsystem