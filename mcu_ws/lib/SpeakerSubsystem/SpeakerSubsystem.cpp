/**
 * @file SpeakerSubsystem.cpp
 */
#include "SpeakerSubsystem.h"

namespace Subsystem {

std::atomic<bool> SpeakerSubsystem::playing{false};

bool SpeakerSubsystem::init() {
  // I2S DMA must be claimed before WiFi/BLE fragment internal DRAM.
  // The HTTP client that feeds it runs in update() once lwIP is up.
  if (!initI2s()) {
    initSuccess_ = false;
    return false;
  }
  initSuccess_ = true;
  return true;
}

void SpeakerSubsystem::begin() {
  if (!i2s_ready_) return;
  Debug::printf(Debug::Level::INFO,
                "[Speaker] Ready — polling http://%s:%u/audio_out",
                setup_.host_ip_.toString().c_str(), setup_.host_port_);
}

void SpeakerSubsystem::update() {
  if (!i2s_ready_) return;

  uint32_t now = millis();
  if (now - last_fail_ms_ < kRetryIntervalMs) return;

  if (now - last_log_ms_ >= kLogIntervalMs) {
    last_log_ms_ = now;
    Debug::printf(Debug::Level::VERBOSE, "[Speaker] Polling %s:%u",
                  setup_.host_ip_.toString().c_str(), setup_.host_port_);
  }

  if (!fetchAndPlay()) last_fail_ms_ = millis();
}

void SpeakerSubsystem::pause() { deinitI2s(); }

void SpeakerSubsystem::reset() {
  deinitI2s();
  if (init()) begin();
}

// ---- I2S lifecycle ----------------------------------------------------------

bool SpeakerSubsystem::initI2s() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(setup_.i2s_port_, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = kDmaBufCount;
  chan_cfg.dma_frame_num = kDmaBufLen;
  // auto_clear: DMA fills with zeros on underrun, so playback tail is silence
  // instead of the last sample repeating. Replaces i2s_zero_dma_buffer().
  chan_cfg.auto_clear = true;
  if (i2s_new_channel(&chan_cfg, &tx_handle_, nullptr) != ESP_OK) {
    Debug::printf(Debug::Level::ERROR, "[Speaker] i2s_new_channel failed");
    return false;
  }

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(setup_.sample_rate_),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_MONO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = static_cast<gpio_num_t>(setup_.bclk_pin_),
              .ws = static_cast<gpio_num_t>(setup_.lrclk_pin_),
              .dout = static_cast<gpio_num_t>(setup_.dout_pin_),
              .din = I2S_GPIO_UNUSED,
              .invert_flags = {},
          },
  };

  if (i2s_channel_init_std_mode(tx_handle_, &std_cfg) != ESP_OK ||
      i2s_channel_enable(tx_handle_) != ESP_OK) {
    Debug::printf(Debug::Level::ERROR, "[Speaker] I2S init failed");
    i2s_del_channel(tx_handle_);
    tx_handle_ = nullptr;
    return false;
  }
  i2s_ready_ = true;
  Debug::printf(Debug::Level::INFO,
                "[Speaker] I2S init OK (%u Hz, BCLK=%d, LRCLK=%d, DOUT=%d)",
                setup_.sample_rate_, setup_.bclk_pin_, setup_.lrclk_pin_,
                setup_.dout_pin_);
  return true;
}

void SpeakerSubsystem::deinitI2s() {
  if (i2s_ready_) {
    i2s_channel_disable(tx_handle_);
    i2s_del_channel(tx_handle_);
    tx_handle_ = nullptr;
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
  bool is_playing = false;
  while (true) {
    int read = esp_http_client_read(client, reinterpret_cast<char*>(buf),
                                    setup_.chunk_size_);
    if (read < 0) break;   // error
    if (read == 0) break;  // server closed connection

    if (!is_playing) {
      is_playing = true;
      playing.store(true, std::memory_order_release);
      Debug::printf(Debug::Level::INFO, "[Speaker] Playback started");
    }

    size_t written = 0;
    i2s_channel_write(tx_handle_, buf, read, &written, portMAX_DELAY);
  }

  free(buf);

  if (is_playing) {
    // i2s_channel_write() returns once data is in the DMA ring buffer, not
    // after it has played out. Wait for the full pipeline to drain;
    // auto_clear keeps the tail as silence instead of a repeating sample.
    // Hold `playing` high for the drain so the mic stays muted until the
    // last sample has physically left the amp.
    const uint32_t drain_ms =
        (kDmaBufCount * kDmaBufLen * 1000u) / setup_.sample_rate_ + 50u;
    vTaskDelay(pdMS_TO_TICKS(drain_ms));
    playing.store(false, std::memory_order_release);
    Debug::printf(Debug::Level::INFO, "[Speaker] Playback finished");
  }

  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return is_playing;
}

}  // namespace Subsystem
