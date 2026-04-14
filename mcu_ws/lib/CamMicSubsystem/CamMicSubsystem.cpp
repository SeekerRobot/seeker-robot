/**
 * @file CamMicSubsystem.cpp
 * @author Tal Avital
 * @date 4/7/2026
 */
#include "CamMicSubsystem.h"

#include <lwip/sockets.h>

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* kStreamContentType =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* kStreamBoundary = "\r\n--" PART_BOUNDARY "\r\n";
static const char* kStreamPart =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

namespace Subsystem {

bool CamMicSubsystem::init() {
  initSuccess_ = true;
  return true;
}

void CamMicSubsystem::begin() {
  startServers();

  // Camera before I2S — initialising I2S first disrupts the LCD_CAM peripheral
  // on ESP32-S3 and causes esp_camera_init to fail.
  camera_config_t cfg = setup_.cam_cfg_;
  if (psramFound()) {
    cfg.frame_size = FRAMESIZE_VGA;
    cfg.jpeg_quality = 10;
    cfg.fb_count = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size = FRAMESIZE_QQVGA;
    cfg.fb_count = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }
  if (esp_camera_init(&cfg) == ESP_OK) {
    camera_ready_ = true;
    Debug::printf(Debug::Level::INFO, "[CamMic] Camera OK (PSRAM: %s)",
                  psramFound() ? "yes" : "no");
  } else {
    Debug::printf(Debug::Level::ERROR, "[CamMic] Camera init failed");
  }

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
  if (i2s_driver_install(setup_.i2s_port_, &i2s_cfg, 0, NULL) == ESP_OK &&
      i2s_set_pin(setup_.i2s_port_, &pin_cfg) == ESP_OK) {
    mic_ready_ = true;
    Debug::printf(Debug::Level::INFO,
                  "[CamMic] Mic OK (%u Hz, CLK=%d, DATA=%d)",
                  setup_.sample_rate_, setup_.clk_pin_, setup_.data_pin_);
    req_ready_ = xSemaphoreCreateBinary();
    // Pin to Core 0 — camera_task (from esp_camera_init) runs on Core 1;
    // separate cores prevent camera_task from starving i2s_read.
    xTaskCreatePinnedToCore(audioStreamTask, "cammic_audio", 4096, this, 5,
                            &audio_task_, 0);
  } else {
    Debug::printf(Debug::Level::ERROR, "[CamMic] Mic init failed");
  }
}

void CamMicSubsystem::update() {
  if (!camera_ready_ && !mic_ready_) return;
  uint32_t now = millis();
  if (now - last_log_ms_ >= kLogIntervalMs) {
    last_log_ms_ = now;
    Debug::printf(Debug::Level::VERBOSE, "[CamMic] cam=%s(:%u) mic=%s(:%u)",
                  (camera_ready_ && isCamServerRunning()) ? "up" : "down",
                  setup_.cam_port_,
                  (mic_ready_ && isAudioServerRunning()) ? "up" : "down",
                  setup_.audio_port_);
  }
}

void CamMicSubsystem::pause() { stopServers(); }

void CamMicSubsystem::reset() {
  stopServers();
  if (camera_ready_) {
    esp_camera_deinit();
    camera_ready_ = false;
  }
  if (mic_ready_) {
    i2s_driver_uninstall(setup_.i2s_port_);
    mic_ready_ = false;
  }
  begin();
}

// ---- Audio stream task ------------------------------------------------------

void CamMicSubsystem::audioStreamTask(void* arg) {
  auto* self = static_cast<CamMicSubsystem*>(arg);
  const size_t chunk = self->setup_.chunk_size_;
  uint8_t* buf = static_cast<uint8_t*>(malloc(chunk));
  if (!buf) {
    Debug::printf(Debug::Level::ERROR, "[CamMic] audio task malloc failed");
    vTaskDelete(NULL);
    return;
  }

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    httpd_req_t* req = self->active_audio_req_;
    Debug::printf(Debug::Level::INFO, "[CamMic] Audio stream connected");

    while (true) {
      size_t bytes_read = 0;
      i2s_read(self->setup_.i2s_port_, buf, chunk, &bytes_read, portMAX_DELAY);
      if (bytes_read > 0) {
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

    Debug::printf(Debug::Level::INFO, "[CamMic] Audio stream disconnected");
    httpd_resp_send_chunk(req, NULL, 0);
    self->active_audio_req_ = nullptr;
    xSemaphoreGive(self->req_ready_);
  }

  free(buf);
  vTaskDelete(NULL);
}

// ---- HTTP handlers ----------------------------------------------------------

esp_err_t CamMicSubsystem::camHandler(httpd_req_t* req) {
  auto* self = static_cast<CamMicSubsystem*>(req->user_ctx);
  if (!self->camera_ready_) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Camera not ready");
    return ESP_FAIL;
  }

  camera_fb_t* fb = nullptr;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, kStreamContentType);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Debug::printf(Debug::Level::ERROR, "[CamMic] Frame capture failed");
      res = ESP_FAIL;
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), kStreamPart, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }
    if (res == ESP_OK) {
      res =
          httpd_resp_send_chunk(req, kStreamBoundary, strlen(kStreamBoundary));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = nullptr;
    }
    if (res != ESP_OK) break;
  }
  return res;
}

esp_err_t CamMicSubsystem::audioHandler(httpd_req_t* req) {
  auto* self = static_cast<CamMicSubsystem*>(req->user_ctx);
  if (!self->mic_ready_) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mic not ready");
    return ESP_FAIL;
  }
  if (self->active_audio_req_ != nullptr) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream busy");
    return ESP_FAIL;
  }
  int fd = httpd_req_to_sockfd(req);
  int nodelay = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  httpd_resp_set_type(req, "application/octet-stream");
  self->active_audio_req_ = req;
  xTaskNotifyGive(self->audio_task_);
  xSemaphoreTake(self->req_ready_, portMAX_DELAY);
  return ESP_OK;
}

// ---- Server lifecycle -------------------------------------------------------

void CamMicSubsystem::startServers() {
  // Camera server
  {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = setup_.cam_port_;
    config.ctrl_port = 32768;  // IDF default; must differ from audio ctrl_port
    config.send_wait_timeout = 30;
    config.max_open_sockets = 2;

    httpd_uri_t cam_uri = {.uri = "/cam",
                           .method = HTTP_GET,
                           .handler = camHandler,
                           .user_ctx = this};

    if (httpd_start(&cam_httpd_, &config) == ESP_OK) {
      httpd_register_uri_handler(cam_httpd_, &cam_uri);
      Debug::printf(Debug::Level::INFO, "[CamMic] /cam  on port %u",
                    setup_.cam_port_);
    } else {
      Debug::printf(Debug::Level::ERROR, "[CamMic] cam httpd_start failed");
    }
  }

  // Audio server
  {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = setup_.audio_port_;
    config.send_wait_timeout = 30;
    config.max_open_sockets = 2;
    config.ctrl_port =
        setup_.audio_port_ + 1;  // must differ from cam ctrl port

    httpd_uri_t audio_uri = {.uri = "/audio",
                             .method = HTTP_GET,
                             .handler = audioHandler,
                             .user_ctx = this};

    if (httpd_start(&audio_httpd_, &config) == ESP_OK) {
      httpd_register_uri_handler(audio_httpd_, &audio_uri);
      Debug::printf(Debug::Level::INFO, "[CamMic] /audio on port %u",
                    setup_.audio_port_);
    } else {
      Debug::printf(Debug::Level::ERROR, "[CamMic] audio httpd_start failed");
    }
  }
}

void CamMicSubsystem::stopServers() {
  if (cam_httpd_) {
    httpd_stop(cam_httpd_);
    cam_httpd_ = nullptr;
  }
  if (audio_httpd_) {
    httpd_stop(audio_httpd_);
    audio_httpd_ = nullptr;
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
