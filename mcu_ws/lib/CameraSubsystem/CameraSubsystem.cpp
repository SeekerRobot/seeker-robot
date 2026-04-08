/**
 * @file CameraSubsystem.cpp
 * @author Tal Avital
 * @date 4/4/26
 */
#include "CameraSubsystem.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* kStreamContentType =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* kStreamBoundary = "\r\n--" PART_BOUNDARY "\r\n";
static const char* kStreamPart =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

namespace Subsystem {

bool CameraSubsystem::init() {
  initSuccess_ = true;
  return true;
}

void CameraSubsystem::begin() {
  startServer();

  camera_config_t cfg = setup_.config_;

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

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Debug::printf(Debug::Level::ERROR, "[Camera] Init failed: 0x%x", err);
    stopServer();
    return;
  }
  camera_ready_ = true;
  Debug::printf(Debug::Level::INFO, "[Camera] Init OK (PSRAM: %s)",
                psramFound() ? "yes" : "no");
}

void CameraSubsystem::update() {
  if (!camera_ready_) return;
  uint32_t now = millis();
  if (now - last_log_ms_ >= kLogIntervalMs) {
    last_log_ms_ = now;
    Debug::printf(Debug::Level::INFO, "[Camera] /cam %s on port %u",
                  isServerRunning() ? "up" : "down", setup_.port_);
  }
}

void CameraSubsystem::pause() { stopServer(); }

void CameraSubsystem::reset() {
  stopServer();
  if (camera_ready_) {
    esp_camera_deinit();
    camera_ready_ = false;
  }
  begin();
}

void CameraSubsystem::startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = setup_.port_;
  config.ctrl_port = setup_.ctrl_port_;
  config.send_wait_timeout = 30;
  config.max_open_sockets = 2;

  httpd_uri_t stream_uri = {.uri = "/cam",
                            .method = HTTP_GET,
                            .handler = streamHandler,
                            .user_ctx = this};

  if (httpd_start(&stream_httpd_, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd_, &stream_uri);
    Debug::printf(Debug::Level::INFO,
                  "[Camera] MJPEG stream at http://<ip>:%u/cam", setup_.port_);
  } else {
    Debug::printf(Debug::Level::ERROR, "[Camera] httpd_start failed");
  }
}

void CameraSubsystem::stopServer() {
  if (stream_httpd_) {
    httpd_stop(stream_httpd_);
    stream_httpd_ = nullptr;
    Debug::printf(Debug::Level::INFO, "[Camera] Stream server stopped");
  }
}

esp_err_t CameraSubsystem::streamHandler(httpd_req_t* req) {
  auto* self = static_cast<CameraSubsystem*>(req->user_ctx);
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
      Debug::printf(Debug::Level::ERROR, "[Camera] Frame capture failed");
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

}  // namespace Subsystem
