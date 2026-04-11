/**
 * @file CamMicSubsystem.h
 * @author Tal Avital
 * @date 4/7/2026
 * @brief Combined camera + PDM microphone subsystem with two HTTP servers.
 *
 * Both servers are started at the top of begin() before any hardware init,
 * avoiding the resource contention that occurs when servers start after
 * esp_camera_init() has already spawned its internal tasks.
 *
 * Both servers run their own httpd task so /cam and /audio can be streamed
 * concurrently — the IDF single-threaded httpd can only serve one long-lived
 * connection per instance.
 *
 *   GET http://<ip>:<cam_port>/cam     — MJPEG video stream
 *   GET http://<ip>:<cam_port+1>/audio — raw 16-bit PCM, mono, sample_rate Hz
 *
 * Usage:
 * @code
 * // Camera on port 80 (/cam), audio on port 81 (/audio)
 * static Subsystem::CamMicSetup setup(camera_config, I2S_NUM_0, 16000,
 *                                     Config::pdm_clk, Config::pdm_data);
 * auto& cam_mic = Subsystem::CamMicSubsystem::getInstance(setup);
 * cam_mic.beginThreadedPinned(8192, 2, 5000, 1);
 * @endcode
 */
#pragma once

#include <CustomDebug.h>
#include <ThreadedSubsystem.h>
#include <driver/i2s.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Subsystem {

class CamMicSetup : public Classes::BaseSetup {
 public:
  CamMicSetup() = delete;

  /// @param cam_cfg     Camera hardware config (pins, pixel format, etc.).
  /// @param i2s_port    I2S peripheral (I2S_NUM_0 or I2S_NUM_1).
  /// @param sample_rate PDM sample rate in Hz (e.g. 16000).
  /// @param clk_pin     PDM clock GPIO (ws_io_num).
  /// @param data_pin    PDM data GPIO (data_in_num).
  /// @param gain        Linear gain multiplier per audio sample (1 = unity).
  /// @param cam_port    HTTP port for /cam (audio is always cam_port + 1).
  /// @param chunk_size  I2S DMA read size in bytes per audio HTTP chunk.
  CamMicSetup(const camera_config_t& cam_cfg, i2s_port_t i2s_port,
              uint32_t sample_rate, int clk_pin, int data_pin, int gain = 4,
              uint16_t cam_port = 80, size_t chunk_size = 2048)
      : Classes::BaseSetup("CamMicSubsystem"),
        cam_cfg_(cam_cfg),
        i2s_port_(i2s_port),
        sample_rate_(sample_rate),
        clk_pin_(clk_pin),
        data_pin_(data_pin),
        gain_(gain),
        cam_port_(cam_port),
        audio_port_(cam_port + 1),
        chunk_size_(chunk_size) {}

  const camera_config_t cam_cfg_;
  const i2s_port_t i2s_port_;
  const uint32_t sample_rate_;
  const int clk_pin_;
  const int data_pin_;
  const int gain_;
  const uint16_t cam_port_;
  const uint16_t audio_port_;
  const size_t chunk_size_;
};

/**
 * @brief Camera + mic subsystem with two dedicated HTTP servers.
 *
 * begin() starts both httpd servers first (while heap is plentiful), then
 * initialises camera and mic hardware sequentially in the same task.
 * audioStreamTask runs on Core 0; camera_task (spawned by esp_camera_init)
 * runs on Core 1 — keeping them on separate cores prevents I2S starvation.
 */
class CamMicSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  CamMicSubsystem(const CamMicSubsystem&) = delete;
  CamMicSubsystem& operator=(const CamMicSubsystem&) = delete;

  static CamMicSubsystem& getInstance(const CamMicSetup& setup) {
    static CamMicSubsystem instance(setup);
    return instance;
  }

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  bool isCameraReady() const { return camera_ready_; }
  bool isMicReady() const { return mic_ready_; }
  bool isCamServerRunning() const { return cam_httpd_ != nullptr; }
  bool isAudioServerRunning() const { return audio_httpd_ != nullptr; }

 private:
  explicit CamMicSubsystem(const CamMicSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  void startServers();
  void stopServers();

  static void audioStreamTask(void* arg);
  static esp_err_t camHandler(httpd_req_t* req);
  static esp_err_t audioHandler(httpd_req_t* req);

  const CamMicSetup setup_;
  httpd_handle_t cam_httpd_ = nullptr;
  httpd_handle_t audio_httpd_ = nullptr;
  httpd_req_t* active_audio_req_ = nullptr;
  TaskHandle_t audio_task_ = nullptr;
  SemaphoreHandle_t req_ready_ = nullptr;
  bool camera_ready_ = false;
  bool mic_ready_ = false;
  uint32_t last_log_ms_ = 0;

  static constexpr uint32_t kLogIntervalMs = 1000;
};

}  // namespace Subsystem
