/**
 * @file MicSubsystem.cpp
 * @author Tal Avital
 * @date 4/7/2026
 */
#include "MicSubsystem.h"

#include <lwip/sockets.h>

namespace Subsystem {

bool MicSubsystem::init() {
  initSuccess_ = true;
  return true;
}

void MicSubsystem::begin() {
  // Start the HTTP server before I2S init so httpd_start runs while heap is
  // plentiful.
  startServer();

  i2s_config_t i2s_cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate = setup_.sample_rate_,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false,
  };
  i2s_pin_config_t pin_cfg = {
      .bck_io_num = I2S_PIN_NO_CHANGE,
      .ws_io_num = setup_.clk_pin_,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = setup_.data_pin_,
  };

  if (i2s_driver_install(setup_.i2s_port_, &i2s_cfg, 0, NULL) != ESP_OK ||
      i2s_set_pin(setup_.i2s_port_, &pin_cfg) != ESP_OK) {
    Debug::printf(Debug::Level::ERROR, "[Mic] I2S init failed");
    stopServer();
    return;
  }
  mic_ready_ = true;
  Debug::printf(Debug::Level::INFO,
                "[Mic] I2S init OK (%u Hz, CLK=%d, DATA=%d)",
                setup_.sample_rate_, setup_.clk_pin_, setup_.data_pin_);
}

void MicSubsystem::update() {
  if (!mic_ready_) return;
  uint32_t now = millis();
  if (now - last_log_ms_ >= kLogIntervalMs) {
    last_log_ms_ = now;
    Debug::printf(Debug::Level::INFO, "[Mic] /audio %s on port %u",
                  isServerRunning() ? "up" : "down", setup_.http_port_);
  }
}

void MicSubsystem::pause() { stopServer(); }

void MicSubsystem::reset() {
  stopServer();
  if (mic_ready_) {
    i2s_driver_uninstall(setup_.i2s_port_);
    mic_ready_ = false;
  }
  begin();
}

// ---- Audio stream task ------------------------------------------------------

void MicSubsystem::audioStreamTask(void* arg) {
  auto* self = static_cast<MicSubsystem*>(arg);
  const size_t chunk = self->setup_.chunk_size_;
  uint8_t* buf = static_cast<uint8_t*>(malloc(chunk));
  if (!buf) {
    Debug::printf(Debug::Level::ERROR, "[Mic] audio task malloc failed");
    vTaskDelete(NULL);
    return;
  }

  while (true) {
    // Block until an HTTP client connects and notifies us.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    httpd_req_t* req = self->active_req_;
    Debug::printf(Debug::Level::INFO, "[Mic] Audio stream connected");

    while (true) {
      size_t bytes_read = 0;
      i2s_read(self->setup_.i2s_port_, buf, chunk, &bytes_read, portMAX_DELAY);
      if (bytes_read > 0) {
        // Apply gain and clip to int16 range.
        int16_t* samples = reinterpret_cast<int16_t*>(buf);
        size_t n = bytes_read / sizeof(int16_t);
        for (size_t i = 0; i < n; i++) {
          int32_t s = static_cast<int32_t>(samples[i]) * self->setup_.gain_;
          if (s > 32767) s = 32767;
          if (s < -32768) s = -32768;
          samples[i] = static_cast<int16_t>(s);
        }
        if (httpd_resp_send_chunk(req, reinterpret_cast<const char*>(buf),
                                  bytes_read) != ESP_OK) {
          break;
        }
      }
    }

    Debug::printf(Debug::Level::INFO, "[Mic] Audio stream disconnected");
    httpd_resp_send_chunk(req, NULL, 0);
    self->active_req_ = nullptr;
    xSemaphoreGive(self->req_ready_);
  }

  free(buf);
  vTaskDelete(NULL);
}

// ---- HTTP handler -----------------------------------------------------------

esp_err_t MicSubsystem::audioHandler(httpd_req_t* req) {
  auto* self = static_cast<MicSubsystem*>(req->user_ctx);
  if (!self->mic_ready_) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mic not ready");
    return ESP_FAIL;
  }
  if (self->active_req_ != nullptr) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream busy");
    return ESP_FAIL;
  }
  // Disable Nagle so TCP doesn't batch small chunks behind delayed ACKs.
  // Without this, Nagle + 200ms delayed-ACK on the client causes 5Hz gaps.
  int fd = httpd_req_to_sockfd(req);
  int nodelay = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  httpd_resp_set_type(req, "application/octet-stream");
  self->active_req_ = req;
  xTaskNotifyGive(self->audio_task_);
  xSemaphoreTake(self->req_ready_, portMAX_DELAY);
  return ESP_OK;
}

// ---- Server lifecycle -------------------------------------------------------

void MicSubsystem::startServer() {
  req_ready_ = xSemaphoreCreateBinary();

  // Pin to Core 0 — camera_task (from esp_camera_init) runs on Core 1;
  // separate cores prevent camera_task from starving i2s_read.
  xTaskCreatePinnedToCore(audioStreamTask, "mic_audio", 4096, this, 5,
                          &audio_task_, 0);

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = setup_.http_port_;
  config.ctrl_port = setup_.ctrl_port_;
  config.send_wait_timeout = 30;
  config.max_open_sockets = 2;

  httpd_uri_t audio_uri = {.uri = "/audio",
                           .method = HTTP_GET,
                           .handler = audioHandler,
                           .user_ctx = this};

  if (httpd_start(&stream_httpd_, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd_, &audio_uri);
    Debug::printf(Debug::Level::INFO,
                  "[Mic] Audio stream at http://<ip>:%u/audio",
                  setup_.http_port_);
  } else {
    Debug::printf(Debug::Level::ERROR, "[Mic] httpd_start failed");
  }
}

void MicSubsystem::stopServer() {
  if (stream_httpd_) {
    httpd_stop(stream_httpd_);
    stream_httpd_ = nullptr;
    Debug::printf(Debug::Level::INFO, "[Mic] Stream server stopped");
  }
  if (audio_task_) {
    vTaskDelete(audio_task_);
    audio_task_ = nullptr;
  }
  if (req_ready_) {
    vSemaphoreDelete(req_ready_);
    req_ready_ = nullptr;
  }
}

}  // namespace Subsystem
