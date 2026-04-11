/**
 * @file SpeakerSubsystem.h
 * @brief I2S speaker subsystem that fetches audio from the ROS 2 host.
 *
 * Connects to an HTTP endpoint on the micro-ROS agent (host PC) to receive
 * raw 16-bit PCM audio, then plays it through I2S to an external amplifier
 * (e.g. MAX98357A). Optionally mutes the MicSubsystem while audio is playing.
 *
 * The ESP32 acts as an HTTP client — it already knows the host IP via the
 * AGENT_IP build macro from network_config.ini, so no extra configuration is
 * needed.
 *
 * Usage:
 * @code
 * static Subsystem::SpeakerSetup spk_setup(
 *     I2S_NUM_1, 16000,
 *     Config::spk_bclk, Config::spk_lrclk, Config::spk_dout,
 *     IPAddress(AGENT_IP), 8383, 2048, &mic);
 * auto& spk = Subsystem::SpeakerSubsystem::getInstance(spk_setup);
 * spk.beginThreadedPinned(4096, 2, 100, 1);
 * @endcode
 */
#pragma once

#include <CustomDebug.h>
#include <ThreadedSubsystem.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_http_client.h>
#include <freertos/FreeRTOS.h>

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
  /// @param host_ip     IP address of the ROS 2 host (use AGENT_IP macro).
  /// @param host_port   Port the TTS server listens on.
  /// @param chunk_size  Bytes per HTTP read / I2S write cycle.
  /// @param mic         Optional MicSubsystem to pause during playback.
  SpeakerSetup(i2s_port_t i2s_port, uint32_t sample_rate, int bclk_pin,
               int lrclk_pin, int dout_pin, IPAddress host_ip,
               uint16_t host_port = 8383, size_t chunk_size = 2048,
               MicSubsystem* mic = nullptr)
      : Classes::BaseSetup("SpeakerSubsystem"),
        i2s_port_(i2s_port),
        sample_rate_(sample_rate),
        bclk_pin_(bclk_pin),
        lrclk_pin_(lrclk_pin),
        dout_pin_(dout_pin),
        host_ip_(host_ip),
        host_port_(host_port),
        chunk_size_(chunk_size),
        mic_(mic) {}

  const i2s_port_t i2s_port_;
  const uint32_t sample_rate_;
  const int bclk_pin_;
  const int lrclk_pin_;
  const int dout_pin_;
  const IPAddress host_ip_;
  const uint16_t host_port_;
  const size_t chunk_size_;
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

 private:
  explicit SpeakerSubsystem(const SpeakerSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  bool initI2s();
  void deinitI2s();
  bool fetchAndPlay();

  const SpeakerSetup setup_;
  bool i2s_ready_ = false;
  uint32_t last_log_ms_ = 0;
  uint32_t last_fail_ms_ = 0;

  static constexpr uint32_t kLogIntervalMs   = 1000;
  static constexpr uint32_t kRetryIntervalMs = 3000;
  // Socket read timeout. Failed connections return immediately via TCP
  // RST/ECONNREFUSED regardless of this value, so 30 s only fires on a
  // genuinely stalled active stream — safe for clips up to ~30 s.
  static constexpr int      kHttpTimeoutMs  = 30000;
  // DMA geometry — must match initI2s().
  static constexpr uint32_t kDmaBufCount = 8;
  static constexpr uint32_t kDmaBufLen   = 512;  // samples per buffer
};

}  // namespace Subsystem
