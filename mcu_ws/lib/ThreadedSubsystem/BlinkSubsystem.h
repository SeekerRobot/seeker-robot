/**
 * @file BlinkSubsystem
 * @author Aldem Pido
 * @date 3/26/26
 * @brief Example implementation of a Threaded Subsystem.
 */
#pragma once
#include <Arduino.h>

#include "ThreadedSubsystem.h"

namespace Subsystem {
class BlinkSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  explicit BlinkSubsystem(const Classes::BaseSetup& setup)
      : ThreadedSubsystem(setup) {}

  void begin() override {
    pinMode(LED_BUILTIN, OUTPUT);
    initSuccess_ = true;
  }

  void update() override {
    if (!initSuccess_) return;
    led_state_ = !led_state_;
    digitalWrite(LED_BUILTIN, led_state_ ? HIGH : LOW);
  }

  bool init() override { return true; }
  const char* getInfo() override { return setup_.getId(); }
  void pause() override {}
  void reset() override {}

 private:
  bool led_state_ = false;
};
};  // namespace Subsystem
