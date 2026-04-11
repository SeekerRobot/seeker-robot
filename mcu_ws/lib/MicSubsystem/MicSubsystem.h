/**
 * @file MicSubsystem.h
 * @author Tal Avital
 * @date 4/7/2026
 * @brief PDM microphone subsystem with HTTP audio streaming.
 *
 * Initialises the ESP32 I2S peripheral in PDM-RX mode and serves raw 16-bit
 * PCM audio over HTTP at /audio.
 *
 * The subsystem spawns a dedicated FreeRTOS audio task that reads I2S DMA
 * buffers and pushes them to the connected HTTP client. The ThreadedSubsystem
 * loop runs at the configured update interval and logs periodic status.
 *
 * Usage:
 * @code
 * static Subsystem::MicSetup mic_setup(I2S_NUM_0, 16000,
 *                                      Config::pdm_clk, Config::pdm_data,
 *                                      4, 81);
 * auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
 * mic.beginThreadedPinned(4096, 2, 5000, 1);
 * @endcode
 */
#pragma once

#include <CustomDebug.h>
#include <ThreadedSubsystem.h>
#include <driver/i2s.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Subsystem {

class MicSetup : public Classes::BaseSetup {
 public:
  MicSetup() = delete;

  /// @param i2s_port   I2S peripheral number (I2S_NUM_0 or I2S_NUM_1).
  /// @param sample_rate  PCM sample rate in Hz (e.g. 16000).
  /// @param clk_pin    PDM clock GPIO (ws_io_num in IDF pin config).
  /// @param data_pin   PDM data GPIO (data_in_num in IDF pin config).
  /// @param gain       Linear gain multiplier applied per sample (1 = unity).
  /// @param http_port  HTTP server port for the /audio endpoint.
  /// @param chunk_size I2S DMA read size in bytes per HTTP chunk.
  /// @param ctrl_port  httpd internal control socket port — must be unique
  ///                   across all httpd instances (default 32769).
  MicSetup(i2s_port_t i2s_port, uint32_t sample_rate, int clk_pin, int data_pin,
           int gain = 4, uint16_t http_port = 81, size_t chunk_size = 2048,
           uint16_t ctrl_port = 32769)
      : Classes::BaseSetup("MicSubsystem"),
        i2s_port_(i2s_port),
        sample_rate_(sample_rate),
        clk_pin_(clk_pin),
        data_pin_(data_pin),
        gain_(gain),
        http_port_(http_port),
        chunk_size_(chunk_size),
        ctrl_port_(ctrl_port) {}

  const i2s_port_t i2s_port_;
  const uint32_t sample_rate_;
  const int clk_pin_;
  const int data_pin_;
  const int gain_;
  const uint16_t http_port_;
  const size_t chunk_size_;
  const uint16_t ctrl_port_;
};

/**
 * @brief PDM mic subsystem — I2S init + HTTP audio stream server.
 *
 * begin() initialises I2S in PDM-RX mode and starts an httpd instance with
 * GET /audio, which blocks until a streaming task pushes raw 16-bit PCM to
 * the client. update() logs server status every kLogIntervalMs milliseconds.
 */
class MicSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  MicSubsystem(const MicSubsystem&) = delete;
  MicSubsystem& operator=(const MicSubsystem&) = delete;

  static MicSubsystem& getInstance(const MicSetup& setup) {
    static MicSubsystem instance(setup);
    return instance;
  }

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  bool isMicReady() const { return mic_ready_; }
  bool isServerRunning() const { return stream_httpd_ != nullptr; }

 private:
  explicit MicSubsystem(const MicSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  void startServer();
  void stopServer();

  static void audioStreamTask(void* arg);
  static esp_err_t audioHandler(httpd_req_t* req);

  const MicSetup setup_;
  httpd_handle_t stream_httpd_ = nullptr;
  httpd_req_t* active_req_ = nullptr;
  TaskHandle_t audio_task_ = nullptr;
  SemaphoreHandle_t req_ready_ = nullptr;
  bool mic_ready_ = false;
  uint32_t last_log_ms_ = 0;

  static constexpr uint32_t kLogIntervalMs = 1000;
};

}  // namespace Subsystem
