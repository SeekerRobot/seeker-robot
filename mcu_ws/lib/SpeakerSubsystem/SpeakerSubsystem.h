/**
 * @file SpeakerSubsystem.h
 * @brief I2S speaker subsystem with HTTP audio streaming (reverse of
 * MicSubsystem).
 *
 * Receives raw 16-bit PCM audio via HTTP POST /speak and plays it through an
 * I2S DAC/amplifier (e.g. MAX98357A). Optionally mutes the MicSubsystem while
 * audio is playing to prevent feedback.
 *
 * Usage:
 * @code
 * static Subsystem::SpeakerSetup spk_setup(
 *     I2S_NUM_1, 16000,
 *     Config::spk_bclk, Config::spk_lrclk, Config::spk_dout,
 *     82, 2048, 32770, &mic);
 * auto& spk = Subsystem::SpeakerSubsystem::getInstance(spk_setup);
 * spk.beginThreadedPinned(4096, 2, 5000, 1);
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

class MicSubsystem;

class SpeakerSetup : public Classes::BaseSetup {
 public:
  SpeakerSetup() = delete;

  /// @param i2s_port    I2S peripheral number (I2S_NUM_1 recommended).
  /// @param sample_rate PCM sample rate in Hz (e.g. 16000).
  /// @param bclk_pin    I2S bit clock GPIO.
  /// @param lrclk_pin   I2S word-select (LRCLK) GPIO.
  /// @param dout_pin    I2S data-out GPIO.
  /// @param http_port   HTTP server port for the /speak endpoint.
  /// @param chunk_size  Bytes read per HTTP recv / I2S write cycle.
  /// @param ctrl_port   httpd internal control socket port — must be unique
  ///                    across all httpd instances.
  /// @param mic         Optional MicSubsystem to pause during playback.
  SpeakerSetup(i2s_port_t i2s_port, uint32_t sample_rate, int bclk_pin,
               int lrclk_pin, int dout_pin, uint16_t http_port = 82,
               size_t chunk_size = 2048, uint16_t ctrl_port = 32770,
               MicSubsystem* mic = nullptr)
      : Classes::BaseSetup("SpeakerSubsystem"),
        i2s_port_(i2s_port),
        sample_rate_(sample_rate),
        bclk_pin_(bclk_pin),
        lrclk_pin_(lrclk_pin),
        dout_pin_(dout_pin),
        http_port_(http_port),
        chunk_size_(chunk_size),
        ctrl_port_(ctrl_port),
        mic_(mic) {}

  const i2s_port_t i2s_port_;
  const uint32_t sample_rate_;
  const int bclk_pin_;
  const int lrclk_pin_;
  const int dout_pin_;
  const uint16_t http_port_;
  const size_t chunk_size_;
  const uint16_t ctrl_port_;
  MicSubsystem* const mic_;
};

class SpeakerSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  SpeakerSubsystem(const SpeakerSubsystem&) = delete;
  SpeakerSubsystem& operator=(const SpeakerSubsystem&) = delete;

  static SpeakerSubsystem& getInstance(const SpeakerSetup& setup) {
    static SpeakerSubsystem instance(setup);
    return instance;
  }

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  bool isI2sReady() const { return i2s_ready_; }
  bool isServerRunning() const { return httpd_ != nullptr; }

 private:
  explicit SpeakerSubsystem(const SpeakerSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  void startServer();
  void stopServer();

  static void audioPlayTask(void* arg);
  static esp_err_t speakHandler(httpd_req_t* req);

  const SpeakerSetup setup_;
  httpd_handle_t httpd_ = nullptr;
  httpd_req_t* active_req_ = nullptr;
  TaskHandle_t play_task_ = nullptr;
  SemaphoreHandle_t req_ready_ = nullptr;
  bool i2s_ready_ = false;
  uint32_t last_log_ms_ = 0;

  static constexpr uint32_t kLogIntervalMs = 5000;
};

}  // namespace Subsystem
