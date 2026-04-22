/**
 * @file StatusLedController.h
 * @author Claude Code
 * @date 4/18/2026
 * @brief Robot-state FSM driving the onboard SK6812 chain via LedSubsystem.
 *
 * Polls local subsystem state (battery voltage, WiFi, micro-ROS, gait) at
 * ~10 Hz and picks a single RobotState by priority:
 *
 *   BOOT > LOW_BATTERY > WIFI_WAIT > MICROROS_WAIT > WALKING > STOPPING >
 *   AUDIO_PLAYING > IDLE
 *
 * On every state transition the mapped LedSubsystem effect is re-applied.
 * When the state is unchanged the effect is left alone so LedSubsystem's
 * animation phase keeps running smoothly (it resets anim_phase_ on each
 * setEffect() call).
 *
 * Missing subsystems (battery, gait) skip the states that depend on them.
 *
 * Usage:
 *   static StatusLedSetup slc_setup{
 *       .leds = &leds, .battery = &batt, .wifi = &wifi,
 *       .manager = &manager, .gait = &gait_ctrl};
 *   static StatusLedController slc(slc_setup);
 *   slc.init();
 *   slc.beginThreadedPinned(3072, 2, 100, 1);
 */
#pragma once

#include <Arduino.h>
#include <FastLED.h>
#include <RobotConfig.h>
#include <ThreadedSubsystem.h>
#include <atomic>
#include <hal_thread.h>

#include "LedSubsystem.h"

// Forward declarations keep this header cheap to include.
namespace Subsystem {
class BatterySubsystem;
class ESP32WifiSubsystem;
class MicrorosManager;
enum class WifiState : uint8_t;
}  // namespace Subsystem
namespace Gait {
class GaitController;
enum class GaitState : uint8_t;
}  // namespace Gait

#include <BatterySubsystem.h>
#include <ESP32WifiSubsystem.h>
#include <GaitController.h>
#include <microros_manager_robot.h>

namespace Subsystem {

enum class RobotState : uint8_t {
  BOOT,
  LOW_BATTERY,
  WIFI_WAIT,
  MICROROS_WAIT,
  WALKING,
  STOPPING,
  AUDIO_PLAYING,
  IDLE,
};

struct StatusLedSetup : public Classes::BaseSetup {
  LedSubsystem<Config::rgb_data>* leds = nullptr;   ///< required
  BatterySubsystem* battery = nullptr;              ///< optional
  ESP32WifiSubsystem* wifi = nullptr;               ///< required
  MicrorosManager* manager = nullptr;               ///< required
  Gait::GaitController* gait = nullptr;             ///< optional
  /// Optional flag watched for the AUDIO_PLAYING state. When true and the
  /// FSM would otherwise return IDLE, the LEDs pulse cyan faster instead.
  /// Point at SpeakerSubsystem::playing to mirror on-board speaker output.
  const std::atomic<bool>* audio_active = nullptr;  ///< optional

  float low_batt_volts = 11.1f;   ///< enter LOW_BATTERY below this
  float low_batt_clear = 11.4f;   ///< leave LOW_BATTERY only above this
  uint32_t boot_hold_ms = 2000;   ///< show BOOT for at least this long
  uint8_t brightness = 96;        ///< FastLED global brightness

  StatusLedSetup() : Classes::BaseSetup("StatusLedController") {}
};

class StatusLedController : public Subsystem::ThreadedSubsystem {
 public:
  explicit StatusLedController(const StatusLedSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  bool init() override {
    // Required pointers.
    return setup_.leds != nullptr && setup_.wifi != nullptr &&
           setup_.manager != nullptr;
  }

  void begin() override {
    if (!setup_.leds) return;
    setup_.leds->setBrightness(setup_.brightness);
    applyState(RobotState::BOOT);
    boot_start_ms_ = millis();
    started_ = true;
  }

  void update() override {
    if (!started_) return;
    RobotState next = decideState();
    if (next != current_) {
      applyState(next);
      current_ = next;
    }
  }

  void pause() override {}
  void reset() override {
    if (setup_.leds) setup_.leds->clear();
    started_ = false;
    current_ = RobotState::BOOT;
    low_batt_latched_ = false;
  }

  const char* getInfo() override { return setup_.getId(); }

  RobotState getState() const { return current_; }

 private:
  RobotState decideState() {
    // BOOT holds until the fuse expires, regardless of other signals.
    if (millis() - boot_start_ms_ < setup_.boot_hold_ms) {
      return RobotState::BOOT;
    }

    // LOW_BATTERY with hysteresis: enter below low_batt_volts, leave only
    // when voltage climbs past low_batt_clear.
    if (setup_.battery) {
      float v = setup_.battery->getVoltage();
      if (low_batt_latched_) {
        if (v >= setup_.low_batt_clear) low_batt_latched_ = false;
      } else {
        if (v > 0.5f && v < setup_.low_batt_volts) low_batt_latched_ = true;
      }
      if (low_batt_latched_) return RobotState::LOW_BATTERY;
    }

    if (!setup_.wifi->isConnected()) return RobotState::WIFI_WAIT;
    if (!setup_.manager->isConnected()) return RobotState::MICROROS_WAIT;

    if (setup_.gait) {
      Gait::GaitState gs = setup_.gait->getState();
      if (gs == Gait::GaitState::WALKING) return RobotState::WALKING;
      if (gs == Gait::GaitState::STOPPING) return RobotState::STOPPING;
    }

    if (setup_.audio_active &&
        setup_.audio_active->load(std::memory_order_acquire)) {
      return RobotState::AUDIO_PLAYING;
    }

    return RobotState::IDLE;
  }

  void applyState(RobotState s) {
    if (!setup_.leds) return;
    switch (s) {
      case RobotState::BOOT:
        setup_.leds->setEffect(LedMode::RAINBOW_PULSE, CRGB::White, 200);
        break;
      case RobotState::LOW_BATTERY:
        setup_.leds->setEffect(LedMode::PULSE, CRGB::Red, 512);
        break;
      case RobotState::WIFI_WAIT:
        setup_.leds->setEffect(LedMode::CHASE, CRGB::Red, 256);
        break;
      case RobotState::MICROROS_WAIT:
        setup_.leds->setEffect(LedMode::PULSE, CRGB::Yellow, 1500);
        break;
      case RobotState::WALKING:
        setup_.leds->setEffect(LedMode::CHASE_RAINBOW, CRGB::White, 300);
        break;
      case RobotState::STOPPING:
        setup_.leds->setEffect(LedMode::PULSE, CRGB(255, 80, 0), 200);
        break;
      case RobotState::AUDIO_PLAYING:
        setup_.leds->setEffect(LedMode::PULSE, CRGB::Orange, 1500);
        break;
      case RobotState::IDLE:
        setup_.leds->setEffect(LedMode::PULSE, CRGB::Cyan, 250);
        break;
    }
  }

  const StatusLedSetup setup_;
  RobotState current_ = RobotState::BOOT;
  uint32_t boot_start_ms_ = 0;
  bool started_ = false;
  bool low_batt_latched_ = false;
};

}  // namespace Subsystem
