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
  // 1. Snapshot state under data lock.
  //    NanoCanvas constructor calls clear() and zeroes any buffer passed to it,
  //    so we cannot copy raw_framebuffer_ into framebuffer_ here — it would be
  //    wiped on canvas construction. Instead, snapshot it into raw_snap_ and
  //    memcpy after canvas construction (mirroring the PROGMEM path).
  bool dirty;
  bool contrast_dirty;
  bool invert_dirty;
  uint8_t contrast_val;
  bool invert_val;
  bool use_raw;
  const uint8_t* frame_data = nullptr;
  TextOverlay overlays_snap[kMaxOverlays];
  uint8_t raw_snap[kBufferSize];

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

    if (dirty && use_raw) {
      memcpy(raw_snap, raw_framebuffer_, kBufferSize);
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
    NanoCanvas canvas(kWidth, kHeight, framebuffer_);  // zeroes framebuffer_

    if (!use_raw) {
      if (frame_data) {
        // Copy PROGMEM frame directly into framebuffer (after canvas zeroed it)
        memcpy_P(framebuffer_, frame_data, kBufferSize);
      }
      // else: canvas already zeroed it — blank screen
    } else {
      // Copy raw snapshot into framebuffer_ now that canvas construction is
      // done
      memcpy(framebuffer_, raw_snap, kBufferSize);
    }

    // Draw text overlays on top (after raw/PROGMEM copy so they composite
    // correctly)
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

void OledSubsystem::begin() {
  if (setup_.lcd_port_ > 0) {
    xTaskCreatePinnedToCore(lcdFetchTask, "oled_fetch", 4096, this, 3,
                            &fetch_task_, 1);
    Debug::printf(Debug::Level::INFO,
                  "[OLED] LCD fetch task started — http://%s:%u/lcd_out",
                  setup_.host_ip_.toString().c_str(), setup_.lcd_port_);
  }
}

void OledSubsystem::pause() {
  if (fetch_task_) {
    vTaskDelete(fetch_task_);
    fetch_task_ = nullptr;
  }
}

void OledSubsystem::reset() {
  pause();
  {
    Threads::Scope lock(data_mutex_);
    current_frame_key_ = nullptr;
    current_frame_data_ = nullptr;
    use_raw_framebuffer_ = false;
    memset(overlays_, 0, sizeof(overlays_));
    memset(framebuffer_, 0, sizeof(framebuffer_));
    memset(raw_framebuffer_, 0, sizeof(raw_framebuffer_));
    dirty_ = true;
  }
  begin();
}

// ---------------------------------------------------------------------------
// LCD HTTP client (host → ESP32)
// ---------------------------------------------------------------------------

void OledSubsystem::lcdFetchTask(void* arg) {
  auto* self = static_cast<OledSubsystem*>(arg);
  while (true) {
    uint32_t now = millis();

    if (now - self->last_log_ms_ >= 1000) {
      self->last_log_ms_ = now;
      Debug::printf(
          Debug::Level::VERBOSE, "[OLED] Polling http://%s:%u/lcd_out",
          self->setup_.host_ip_.toString().c_str(), self->setup_.lcd_port_);
    }

    if (now - self->last_fail_ms_ < kRetryIntervalMs) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    self->fetchFrames();
    self->last_fail_ms_ = millis();
  }
}

void OledSubsystem::fetchFrames() {
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%u/lcd_out",
           setup_.host_ip_.toString().c_str(), setup_.lcd_port_);

  esp_http_client_config_t cfg = {};
  cfg.url = url;
  cfg.timeout_ms = kHttpTimeoutMs;

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) return false;

  if (esp_http_client_open(client, 0) != ESP_OK) {
    esp_http_client_cleanup(client);
    return false;
  }

  esp_http_client_fetch_headers(client);
  int status = esp_http_client_get_status_code(client);
  if (status != 200) {
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return false;
  }

  Debug::printf(Debug::Level::INFO, "[OLED] LCD stream connected");

  uint8_t buf[kBufferSize];
  int got = 0;
  while (true) {
    int r = esp_http_client_read(client, reinterpret_cast<char*>(buf) + got,
                                 kBufferSize - got);
    if (r <= 0) break;
    got += r;
    if (got >= static_cast<int>(kBufferSize)) {
      setFramebuffer(buf);
      got = 0;
    }
  }

  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  Debug::printf(Debug::Level::INFO, "[OLED] LCD stream disconnected");
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
