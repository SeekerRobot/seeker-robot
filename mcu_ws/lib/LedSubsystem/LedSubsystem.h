/**
 * @file LedSubsystem.h
 * @author Aldem Pido, Claude Code
 * @date 4/2/2026
 * @brief Threaded LED subsystem for an SK6812 chain via FastLED.
 *
 * Disclaimer: This file was written mostly with Claude Code.
 *
 * Usage:
 *   1. Declare with data pin as template arg: LedSubsystem<Config::rgb_data>
 *   2. Construct with LedSetup (LED count).
 *   3. beginThreadedPinned(...) — calls begin() then loops update() at ~50 Hz.
 *   4. Use setEffect(), setAll(), setLed(), setBrightness() from any thread.
 *
 * Color/brightness separation: per-LED colors are stored independently of
 * global brightness, which is applied by FastLED at show() time.
 */
#pragma once

#include <Arduino.h>
#include <FastLED.h>
#include <ThreadedSubsystem.h>
#include <hal_thread.h>

namespace Subsystem {

enum class LedMode : uint8_t {
  CLEAR,           ///< All LEDs off.
  SOLID,           ///< All LEDs show their individually-set color.
  PULSE,           ///< All LEDs breathe (sine-wave brightness) one color.
  CHASE,           ///< Single moving dot, static color.
  RAINBOW_PULSE,   ///< All LEDs pulse with the same cycling hue.
  CHASE_RAINBOW,   ///< Single moving dot with cycling hue.
  RAINBOW_SPREAD,  ///< Each LED a different hue; all cycle together.
};

class LedSetup : public Classes::BaseSetup {
 public:
  LedSetup() = delete;
  ~LedSetup() = default;

  /// @param num_leds  Number of SK6812 LEDs in the chain.
  explicit LedSetup(uint8_t num_leds)
      : Classes::BaseSetup("LedSubsystem"), num_leds_(num_leds) {}

  const uint8_t num_leds_;
};

/// @tparam DataPin FastLED data pin — must be a compile-time constant.
///                 Use Config::rgb_data from RobotConfig.h.
template <int DataPin>
class LedSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  static constexpr uint8_t kMaxLeds = 32;

  LedSubsystem(const LedSubsystem&) = delete;
  LedSubsystem& operator=(const LedSubsystem&) = delete;

  explicit LedSubsystem(const LedSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup), total_leds_(setup.num_leds_) {}

  // -------------------------------------------------------------------------
  // ThreadedSubsystem overrides
  // -------------------------------------------------------------------------

  bool init() override { return true; }
  const char* getInfo() override { return setup_.getId(); }
  void pause() override {}
  void reset() override {
    Threads::Scope lock(mutex_);
    mode_ = LedMode::CLEAR;
    fill_solid(colors_, total_leds_, CRGB::Black);
  }

  void begin() override {
    // Register FastLED against display_ (the DMA source buffer).
    // Rendering happens into leds_ so DMA reads and CPU writes never race.
    FastLED.addLeds<SK6812, DataPin, GRB>(display_, total_leds_);
    FastLED.setBrightness(global_brightness_);
    fill_solid(leds_, total_leds_, CRGB::Black);
    fill_solid(display_, total_leds_, CRGB::Black);
    FastLED.show();
    init_success_ = true;
  }

  void update() override {
    if (!init_success_) return;

    // Snapshot effect params under lock — show() runs outside.
    LedMode mode;
    CRGB color;
    uint16_t speed;
    uint8_t brightness;
    CRGB colors_snap[kMaxLeds];

    {
      Threads::Scope lock(mutex_);
      mode = mode_;
      color = effect_color_;
      speed = effect_speed_;
      brightness = global_brightness_;
      memcpy(colors_snap, colors_, total_leds_ * sizeof(CRGB));
    }

    FastLED.setBrightness(brightness);

    switch (mode) {
      case LedMode::CLEAR:
        fill_solid(leds_, total_leds_, CRGB::Black);
        break;

      case LedMode::SOLID:
        memcpy(leds_, colors_snap, total_leds_ * sizeof(CRGB));
        break;

      case LedMode::PULSE: {
        // sin8 maps [0,255] → [0,255] with a sine wave
        uint8_t bri = sin8(static_cast<uint8_t>(anim_phase_ >> 8));
        CRGB c = color;
        c.nscale8(bri);
        fill_solid(leds_, total_leds_, c);
        break;
      }

      case LedMode::CHASE: {
        fill_solid(leds_, total_leds_, CRGB::Black);
        uint8_t idx = (anim_phase_ >> 8) % total_leds_;
        leds_[idx] = color;
        break;
      }

      case LedMode::RAINBOW_PULSE: {
        uint8_t hue = static_cast<uint8_t>(anim_phase_ >> 8);
        uint8_t bri = sin8(static_cast<uint8_t>(anim_phase_ >> 7));
        fill_solid(leds_, total_leds_, CHSV(hue, 255, bri));
        break;
      }

      case LedMode::CHASE_RAINBOW: {
        fill_solid(leds_, total_leds_, CRGB::Black);
        uint8_t idx = (anim_phase_ >> 8) % total_leds_;
        uint8_t hue = static_cast<uint8_t>(anim_phase_ >> 8);
        leds_[idx] = CHSV(hue, 255, 255);
        break;
      }

      case LedMode::RAINBOW_SPREAD: {
        uint8_t base_hue = static_cast<uint8_t>(anim_phase_ >> 8);
        for (uint8_t i = 0; i < total_leds_; i++) {
          uint8_t hue = base_hue + (uint8_t)((uint16_t)i * 256 / total_leds_);
          leds_[i] = CHSV(hue, 255, 255);
        }
        break;
      }
    }

    anim_phase_ += speed;

    // Copy render buffer → display buffer, then show.
    // FastLED's async DMA reads display_[] while the next frame renders into
    // leds_[].
    memcpy(display_, leds_, total_leds_ * sizeof(CRGB));
    FastLED.show();
  }

  // -------------------------------------------------------------------------
  // Public API — thread-safe, callable from any task
  // -------------------------------------------------------------------------

  /// @brief Set the color of a single LED. Takes effect on next SOLID update.
  void setLed(uint8_t index, CRGB color) {
    if (index >= total_leds_) return;
    Threads::Scope lock(mutex_);
    colors_[index] = color;
  }

  /// @brief Set all LEDs to one color and switch to SOLID mode.
  void setAll(CRGB color) {
    Threads::Scope lock(mutex_);
    fill_solid(colors_, total_leds_, color);
    mode_ = LedMode::SOLID;
  }

  /// @brief Clear all LEDs (all to black).
  void clear() {
    Threads::Scope lock(mutex_);
    fill_solid(colors_, total_leds_, CRGB::Black);
    mode_ = LedMode::CLEAR;
  }

  /// @brief Set global brightness (0–255). Applied via FastLED.setBrightness.
  void setBrightness(uint8_t brightness) {
    Threads::Scope lock(mutex_);
    global_brightness_ = brightness;
  }

  /// @brief Set an animation effect.
  /// @param mode   The LedMode to use.
  /// @param color  Base color (used by PULSE, CHASE; ignored for rainbow
  /// modes).
  /// @param speed  Animation speed 1–4096 (phase increment per update tick).
  void setEffect(LedMode mode, CRGB color = CRGB::White, uint16_t speed = 128) {
    Threads::Scope lock(mutex_);
    mode_ = mode;
    effect_color_ = color;
    effect_speed_ = (speed == 0) ? 1 : (speed > 4096 ? 4096 : speed);
    anim_phase_ = 0;
  }

  // Getters
  LedMode getMode() const {
    Threads::Scope lock(mutex_);
    return mode_;
  }
  uint8_t getTotalLeds() const { return total_leds_; }

 private:
  const LedSetup setup_;
  const uint8_t total_leds_;

  CRGB leds_[kMaxLeds] = {};     ///< Render buffer — animation writes here.
  CRGB display_[kMaxLeds] = {};  ///< Display buffer — FastLED DMA reads here.
  CRGB colors_[kMaxLeds] = {};  ///< Write buffer — API callers set colors here.

  LedMode mode_ = LedMode::CLEAR;
  CRGB effect_color_ = CRGB::White;
  uint16_t effect_speed_ = 128;
  uint8_t global_brightness_ = 128;
  uint16_t anim_phase_ = 0;

  bool init_success_ = false;

  mutable Threads::Mutex mutex_;
};

}  // namespace Subsystem
