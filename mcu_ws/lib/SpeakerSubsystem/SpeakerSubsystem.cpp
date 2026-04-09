/**
 * @file SpeakerSubsystem.cpp
 */
#include "SpeakerSubsystem.h"

#include <MicSubsystem.h>
#include <lwip/sockets.h>

namespace Subsystem {

bool SpeakerSubsystem::init() {
  initSuccess_ = true;
  return true;
}

void SpeakerSubsystem::begin() {
  startServer();

  i2s_config_t i2s_cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = setup_.sample_rate_,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
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
    stopServer();
    return;
  }
  i2s_zero_dma_buffer(setup_.i2s_port_);
  i2s_ready_ = true;
  Debug::printf(Debug::Level::INFO,
                "[Speaker] I2S init OK (%u Hz, BCLK=%d, LRCLK=%d, DOUT=%d)",
                setup_.sample_rate_, setup_.bclk_pin_, setup_.lrclk_pin_,
                setup_.dout_pin_);
}

void SpeakerSubsystem::update() {
  if (!i2s_ready_) return;
  uint32_t now = millis();
  if (now - last_log_ms_ >= kLogIntervalMs) {
    last_log_ms_ = now;
    Debug::printf(Debug::Level::INFO, "[Speaker] /speak %s on port %u",
                  isServerRunning() ? "up" : "down", setup_.http_port_);
  }
}

void SpeakerSubsystem::pause() { stopServer(); }

void SpeakerSubsystem::reset() {
  stopServer();
  if (i2s_ready_) {
    i2s_driver_uninstall(setup_.i2s_port_);
    i2s_ready_ = false;
  }
  begin();
}

// ---- Audio play task --------------------------------------------------------

void SpeakerSubsystem::audioPlayTask(void* arg) {
  auto* self = static_cast<SpeakerSubsystem*>(arg);
  const size_t chunk = self->setup_.chunk_size_;
  uint8_t* buf = static_cast<uint8_t*>(malloc(chunk));
  if (!buf) {
    Debug::printf(Debug::Level::ERROR, "[Speaker] play task malloc failed");
    vTaskDelete(NULL);
    return;
  }

  while (true) {
    // Block until an HTTP client POSTs audio data.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    httpd_req_t* req = self->active_req_;
    Debug::printf(Debug::Level::INFO, "[Speaker] Playback started");

    // Mute mic while playing.
    if (self->setup_.mic_) {
      self->setup_.mic_->pause();
      Debug::printf(Debug::Level::INFO, "[Speaker] Mic paused");
    }

    int remaining = req->content_len;
    while (remaining > 0) {
      int to_read = (remaining < (int)chunk) ? remaining : (int)chunk;
      int ret = httpd_req_recv(req, reinterpret_cast<char*>(buf), to_read);
      if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
        break;
      }
      remaining -= ret;
      size_t bytes_written = 0;
      i2s_write(self->setup_.i2s_port_, buf, ret, &bytes_written,
                portMAX_DELAY);
    }

    // Drain DMA buffers so the tail of the audio is heard.
    i2s_zero_dma_buffer(self->setup_.i2s_port_);

    Debug::printf(Debug::Level::INFO, "[Speaker] Playback finished");

    // Resume mic.
    if (self->setup_.mic_) {
      self->setup_.mic_->reset();
      Debug::printf(Debug::Level::INFO, "[Speaker] Mic resumed");
    }

    self->active_req_ = nullptr;
    xSemaphoreGive(self->req_ready_);
  }

  free(buf);
  vTaskDelete(NULL);
}

// ---- HTTP handler -----------------------------------------------------------

esp_err_t SpeakerSubsystem::speakHandler(httpd_req_t* req) {
  auto* self = static_cast<SpeakerSubsystem*>(req->user_ctx);
  if (!self->i2s_ready_) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Speaker not ready");
    return ESP_FAIL;
  }
  if (self->active_req_ != nullptr) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Playback busy");
    return ESP_FAIL;
  }

  int fd = httpd_req_to_sockfd(req);
  int nodelay = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

  self->active_req_ = req;
  xTaskNotifyGive(self->play_task_);
  xSemaphoreTake(self->req_ready_, portMAX_DELAY);

  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

// ---- Server lifecycle -------------------------------------------------------

void SpeakerSubsystem::startServer() {
  req_ready_ = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(audioPlayTask, "spk_play", 4096, this, 5, &play_task_,
                          1);

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = setup_.http_port_;
  config.ctrl_port = setup_.ctrl_port_;
  config.send_wait_timeout = 30;
  config.max_open_sockets = 2;

  httpd_uri_t speak_uri = {.uri = "/speak",
                           .method = HTTP_POST,
                           .handler = speakHandler,
                           .user_ctx = this};

  if (httpd_start(&httpd_, &config) == ESP_OK) {
    httpd_register_uri_handler(httpd_, &speak_uri);
    Debug::printf(Debug::Level::INFO,
                  "[Speaker] Accepting audio at http://<ip>:%u/speak",
                  setup_.http_port_);
  } else {
    Debug::printf(Debug::Level::ERROR, "[Speaker] httpd_start failed");
  }
}

void SpeakerSubsystem::stopServer() {
  if (httpd_) {
    httpd_stop(httpd_);
    httpd_ = nullptr;
    Debug::printf(Debug::Level::INFO, "[Speaker] Server stopped");
  }
  if (play_task_) {
    vTaskDelete(play_task_);
    play_task_ = nullptr;
  }
  if (req_ready_) {
    vSemaphoreDelete(req_ready_);
    req_ready_ = nullptr;
  }
}

}  // namespace Subsystem
