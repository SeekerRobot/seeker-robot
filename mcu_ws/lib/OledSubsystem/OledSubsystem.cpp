#include "OledSubsystem.h"

#include <CustomDebug.h>
#include <nano_gfx.h>
#include <ssd1306.h>

#include "OledFrames.h"

namespace Subsystem {

// ---------------------------------------------------------------------------
// ThreadedSubsystem overrides
// ---------------------------------------------------------------------------

bool OledSubsystem::init() {
  Threads::Scope lock(i2c_mutex_);

  ssd1306_128x64_i2c_init();
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_clearScreen();

  Debug::printf(Debug::Level::INFO, "[OLED] Init success (128x64 I2C)");
  return true;
}

void OledSubsystem::update() {
  // 1. Snapshot state under data lock. For the raw framebuffer path, copy the
  //    source bytes directly into framebuffer_ under the lock so writers can't
  //    tear the buffer mid-copy. Single memcpy avoids a second stack buffer.
  bool dirty;
  bool contrast_dirty;
  bool invert_dirty;
  uint8_t contrast_val;
  bool invert_val;
  bool use_raw;
  const uint8_t* frame_data = nullptr;
  TextOverlay overlays_snap[kMaxOverlays];

  {
    Threads::Scope lock(data_mutex_);
    dirty = dirty_;
    contrast_dirty = contrast_dirty_;
    invert_dirty = invert_dirty_;
    contrast_val = pending_contrast_;
    invert_val = pending_invert_;
    use_raw = use_raw_framebuffer_;
    frame_data = current_frame_data_;
    memcpy(overlays_snap, overlays_, sizeof(overlays_));

    // While still under lock, copy raw framebuffer source if active.
    if (dirty && use_raw) {
      memcpy(framebuffer_, raw_framebuffer_, kBufferSize);
    }

    dirty_ = false;
    contrast_dirty_ = false;
    invert_dirty_ = false;
  }

  // 2. If nothing changed, skip I2C entirely
  if (!dirty && !contrast_dirty && !invert_dirty) return;

  // 3. Render to framebuffer (CPU only — no I2C, no mutex needed).
  //    framebuffer_ is only touched by this task after init, so no lock needed.
  if (dirty) {
    NanoCanvas canvas(kWidth, kHeight, framebuffer_);

    if (!use_raw) {
      if (frame_data) {
        // Copy PROGMEM frame directly into framebuffer
        memcpy_P(framebuffer_, frame_data, kBufferSize);
      } else {
        canvas.clear();
      }
    }
    // (raw path already copied into framebuffer_ under the lock above)

    // Draw text overlays on top
    for (uint8_t i = 0; i < kMaxOverlays; i++) {
      if (overlays_snap[i].active && overlays_snap[i].text[0] != '\0') {
        canvas.printFixed(overlays_snap[i].x, overlays_snap[i].y,
                          overlays_snap[i].text, STYLE_NORMAL);
      }
    }

    // 4. Push framebuffer to display (I2C)
    {
      Threads::Scope lock(i2c_mutex_);
      canvas.blt(0, 0);
    }
  }

  // 5. Apply hardware commands under I2C lock
  if (contrast_dirty || invert_dirty) {
    Threads::Scope lock(i2c_mutex_);
    if (contrast_dirty) {
      ssd1306_setContrast(contrast_val);
    }
    if (invert_dirty) {
      if (invert_val) {
        ssd1306_invertMode();
      } else {
        ssd1306_normalMode();
      }
    }
  }
}

void OledSubsystem::reset() {
  Threads::Scope lock(data_mutex_);
  current_frame_key_ = nullptr;
  current_frame_data_ = nullptr;
  use_raw_framebuffer_ = false;
  memset(overlays_, 0, sizeof(overlays_));
  memset(framebuffer_, 0, sizeof(framebuffer_));
  memset(raw_framebuffer_, 0, sizeof(raw_framebuffer_));
  dirty_ = true;
}

// ---------------------------------------------------------------------------
// Public API — thread-safe
// ---------------------------------------------------------------------------

void OledSubsystem::setFrame(const char* key) {
  const uint8_t* data = OledFrames::findFrame(key);
  if (!data) {
    Debug::printf(Debug::Level::WARN, "[OLED] Frame '%s' not found", key);
    return;
  }
  Threads::Scope lock(data_mutex_);
  current_frame_key_ = key;
  current_frame_data_ = data;
  use_raw_framebuffer_ = false;  // PROGMEM frame takes over
  dirty_ = true;
}

void OledSubsystem::setFramebuffer(const uint8_t* data) {
  if (!data) return;
  Threads::Scope lock(data_mutex_);
  memcpy(raw_framebuffer_, data, kBufferSize);
  use_raw_framebuffer_ = true;
  // current_frame_key_ is left alone so getCurrentFrame() still reports
  // the last PROGMEM key if any, but the raw buffer takes precedence.
  dirty_ = true;
}

const char* OledSubsystem::getCurrentFrame() const {
  Threads::Scope lock(data_mutex_);
  return current_frame_key_;
}

void OledSubsystem::setOverlay(uint8_t slot, uint8_t x, uint8_t y,
                               const char* text) {
  if (slot >= kMaxOverlays || !text) return;
  Threads::Scope lock(data_mutex_);
  overlays_[slot].x = x;
  overlays_[slot].y = y;
  strncpy(overlays_[slot].text, text, kMaxTextLen);
  overlays_[slot].text[kMaxTextLen] = '\0';
  overlays_[slot].active = true;
  dirty_ = true;
}

void OledSubsystem::clearOverlay(uint8_t slot) {
  if (slot >= kMaxOverlays) return;
  Threads::Scope lock(data_mutex_);
  overlays_[slot].active = false;
  overlays_[slot].text[0] = '\0';
  dirty_ = true;
}

void OledSubsystem::clearAllOverlays() {
  Threads::Scope lock(data_mutex_);
  for (uint8_t i = 0; i < kMaxOverlays; i++) {
    overlays_[i].active = false;
    overlays_[i].text[0] = '\0';
  }
  dirty_ = true;
}

void OledSubsystem::clear() {
  Threads::Scope lock(data_mutex_);
  current_frame_key_ = nullptr;
  current_frame_data_ = nullptr;
  use_raw_framebuffer_ = false;
  for (uint8_t i = 0; i < kMaxOverlays; i++) {
    overlays_[i].active = false;
    overlays_[i].text[0] = '\0';
  }
  dirty_ = true;
}

void OledSubsystem::setContrast(uint8_t value) {
  Threads::Scope lock(data_mutex_);
  pending_contrast_ = value;
  contrast_dirty_ = true;
}

void OledSubsystem::setInverted(bool enable) {
  Threads::Scope lock(data_mutex_);
  pending_invert_ = enable;
  invert_dirty_ = true;
}

}  // namespace Subsystem
