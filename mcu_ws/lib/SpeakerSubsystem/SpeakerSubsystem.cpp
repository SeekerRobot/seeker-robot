/**
 * @file SpeakerSubsystem.cpp
 */
#include "SpeakerSubsystem.h"

#include <MicSubsystem.h>

namespace Subsystem {

bool SpeakerSubsystem::init() {
  initSuccess_ = true;
  return true;
}

void SpeakerSubsystem::begin() {
  if (!initI2s()) return;
  Debug::printf(Debug::Level::INFO,
                "[Speaker] Ready — polling http://%s:%u/audio_out",
                setup_.host_ip_.toString().c_str(), setup_.host_port_);
}

void SpeakerSubsystem::update() {
  if (!i2s_ready_) return;

  uint32_t now = millis();
  if (now - last_log_ms_ >= kLogIntervalMs) {
    last_log_ms_ = now;
    Debug::printf(Debug::Level::INFO, "[Speaker] Polling %s:%u",
                  setup_.host_ip_.toString().c_str(), setup_.host_port_);
  }

  fetchAndPlay();
}

void SpeakerSubsystem::pause() { deinitI2s(); }

void SpeakerSubsystem::reset() {
  deinitI2s();
  begin();
}

// ---- I2S lifecycle ----------------------------------------------------------

bool SpeakerSubsystem::initI2s() {
  i2s_config_t i2s_cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = setup_.sample_rate_,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = kDmaBufCount,
      .dma_buf_len = kDmaBufLen,
      .use_apll = false,
  };
  i2s_pin_config_t pin_cfg = {
      .bck_io_num = setup_.bclk_pin_,
      .ws_io_num = setup_.lrclk_pin_,
      .data_out_num = setup_.dout_pin_,
      .data_in_num = I2S_PIN_NO_CHANGE,
  };

  if (i2s_driver_install(setup_.i2s_port_, &i2s_cfg, 0, NULL) != ESP_OK ||
      i2s_set_pin(setup_.i2s_port_, &pin_cfg) != ESP_OK) {
    Debug::printf(Debug::Level::ERROR, "[Speaker] I2S init failed");
    return false;
  }
  i2s_zero_dma_buffer(setup_.i2s_port_);
  i2s_ready_ = true;
  Debug::printf(Debug::Level::INFO,
                "[Speaker] I2S init OK (%u Hz, BCLK=%d, LRCLK=%d, DOUT=%d)",
                setup_.sample_rate_, setup_.bclk_pin_, setup_.lrclk_pin_,
                setup_.dout_pin_);
  return true;
}

void SpeakerSubsystem::deinitI2s() {
  if (i2s_ready_) {
    i2s_driver_uninstall(setup_.i2s_port_);
    i2s_ready_ = false;
  }
}

// ---- HTTP fetch + playback --------------------------------------------------

bool SpeakerSubsystem::fetchAndPlay() {
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%u/audio_out",
           setup_.host_ip_.toString().c_str(), setup_.host_port_);

  esp_http_client_config_t cfg = {};
  cfg.url = url;
  cfg.timeout_ms = kHttpTimeoutMs;
  cfg.keep_alive_enable = true;

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) return false;

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
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

  Debug::printf(Debug::Level::INFO, "[Speaker] Connected to audio stream");

  uint8_t* buf = static_cast<uint8_t*>(malloc(setup_.chunk_size_));
  if (!buf) {
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return false;
  }

  // Persistent stream — read chunked data until the connection drops.
  // Each chunk from the TTS server is one complete TTS utterance.
  bool playing = false;
  while (true) {
    int read = esp_http_client_read(client, reinterpret_cast<char*>(buf),
                                    setup_.chunk_size_);
    if (read < 0) break;   // error
    if (read == 0) break;  // server closed connection

    // Mute mic on first data of a new utterance.
    if (!playing) {
      playing = true;
      if (setup_.mic_) {
        setup_.mic_->pause();
        Debug::printf(Debug::Level::INFO, "[Speaker] Mic paused");
      }
      Debug::printf(Debug::Level::INFO, "[Speaker] Playback started");
    }

    size_t written = 0;
    i2s_write(setup_.i2s_port_, buf, read, &written, portMAX_DELAY);
  }

  free(buf);

  if (playing) {
    // i2s_write() returns once data is in the DMA ring buffer, not after it
    // has played out. Wait for the full pipeline to drain before zeroing,
    // otherwise the tail of the audio is silenced mid-playback.
    const uint32_t drain_ms =
        (kDmaBufCount * kDmaBufLen * 1000u) / setup_.sample_rate_ + 50u;
    vTaskDelay(pdMS_TO_TICKS(drain_ms));
    i2s_zero_dma_buffer(setup_.i2s_port_);
    Debug::printf(Debug::Level::INFO, "[Speaker] Playback finished");
    if (setup_.mic_) {
      setup_.mic_->reset();
      Debug::printf(Debug::Level::INFO, "[Speaker] Mic resumed");
    }
  }

  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return playing;
}

}  // namespace Subsystem
