/**
 * @file OledSubsystem.h
 * @author Claude Code
 * @date 4/10/2026
 * @brief Threaded OLED display subsystem for SSD1306 128x64 over I2C.
 *
 * Uses a frame + text-overlay model:
 *   - Set a background frame by string key (PROGMEM bitmaps from OledFrames.h)
 *   - Overlay up to kMaxOverlays independent text strings at pixel positions
 *   - All rendering is CPU-only into a local NanoCanvas buffer
 *   - Only the final blt() touches the I2C bus, protected by a shared mutex
 *
 * Usage:
 *   1. Construct OledSetup with a shared I2C mutex
 *   2. Get singleton via OledSubsystem::getInstance(setup)
 *   3. Call init() to configure the SSD1306 hardware
 *   4. Call beginThreadedPinned(4096, 2, 100, 1) for ~10 Hz on Core 1
 *   5. Use setFrame(), setOverlay(), clear() etc. from any thread
 */
#pragma once

#include <Arduino.h>
#include <ThreadedSubsystem.h>
#include <hal_thread.h>

namespace Subsystem {

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

class OledSetup : public Classes::BaseSetup {
 public:
  OledSetup() = delete;
  ~OledSetup() = default;

  /// @param i2c_mutex Shared I2C bus mutex (same one used by GyroSubsystem,
  ///                  ServoSubsystem, etc.)
  explicit OledSetup(Threads::Mutex& i2c_mutex)
      : Classes::BaseSetup("OledSubsystem"), i2c_mutex_(i2c_mutex) {}

  Threads::Mutex& i2c_mutex_;
};

// ---------------------------------------------------------------------------
// Subsystem
// ---------------------------------------------------------------------------

class OledSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  OledSubsystem(const OledSubsystem&) = delete;
  OledSubsystem& operator=(const OledSubsystem&) = delete;

  static OledSubsystem& getInstance(const OledSetup& setup) {
    static OledSubsystem instance(setup);
    return instance;
  }

  // ---- ThreadedSubsystem overrides ----------------------------------------

  bool init() override;
  void begin() override { return; }
  void update() override;
  void pause() override { return; }
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  // ---- Thread-safe public API ---------------------------------------------

  /// @brief Set the background frame by looking up a key in OledFrames.
  ///        If the key is not found, the display is unchanged.
  void setFrame(const char* key);

  /// @brief Get the current frame key (nullptr if no frame set).
  const char* getCurrentFrame() const;

  /// @brief Set a text overlay at a given slot (0..kMaxOverlays-1).
  /// @param slot  Overlay slot index.
  /// @param x     Pixel x position (0-based).
  /// @param y     Pixel y position (0-based, must be page-aligned for best
  ///              results, i.e. multiples of 8).
  /// @param text  Text string (truncated to kMaxTextLen chars).
  void setOverlay(uint8_t slot, uint8_t x, uint8_t y, const char* text);

  /// @brief Deactivate a single overlay slot.
  void clearOverlay(uint8_t slot);

  /// @brief Deactivate all overlay slots (frame remains).
  void clearAllOverlays();

  /// @brief Clear everything: remove frame and all overlays (blank screen).
  void clear();

  /// @brief Set display hardware contrast (brightness), 0-255.
  void setContrast(uint8_t value);

  /// @brief Enable or disable hardware pixel inversion.
  void setInverted(bool enable);

  // ---- Constants ----------------------------------------------------------

  static constexpr uint8_t kWidth = 128;
  static constexpr uint8_t kHeight = 64;
  static constexpr uint16_t kBufferSize = kWidth * kHeight / 8;  // 1024
  static constexpr uint8_t kMaxOverlays = 4;
  static constexpr uint8_t kMaxTextLen = 21;  // 128 / 6 = 21 chars (6x8 font)

 private:
  explicit OledSubsystem(const OledSetup& setup)
      : ThreadedSubsystem(setup),
        setup_(setup),
        i2c_mutex_(setup.i2c_mutex_) {}

  // ---- Internal types -----------------------------------------------------

  struct TextOverlay {
    uint8_t x = 0;
    uint8_t y = 0;
    char text[kMaxTextLen + 1] = {};
    bool active = false;
  };

  // ---- State --------------------------------------------------------------

  const OledSetup setup_;
  Threads::Mutex& i2c_mutex_;
  mutable Threads::Mutex data_mutex_;

  uint8_t framebuffer_[kBufferSize] = {};

  const char* current_frame_key_ = nullptr;
  const uint8_t* current_frame_data_ = nullptr;  // PROGMEM pointer
  TextOverlay overlays_[kMaxOverlays] = {};
  bool dirty_ = true;

  uint8_t pending_contrast_ = 0xFF;
  bool contrast_dirty_ = false;
  bool pending_invert_ = false;
  bool invert_dirty_ = false;
};

}  // namespace Subsystem
