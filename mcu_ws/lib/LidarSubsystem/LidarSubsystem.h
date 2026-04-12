/**
 * @file LidarSubsystem.h
 * @author Aldem Pido
 * @date 4/2/2026
 * @brief ThreadedSubsystem wrapping the KAIA.AI LDS library for the
 *        LDRobot LD14P LiDAR. Runs on core 0 at priority 4 with 1 ms
 *        update delay. Scan data is double-buffered: the accumulation
 *        buffer is updated exclusively in the update() task; the published
 *        buffer is swapped under data_mutex_ on every scan_completed event
 *        and read by any thread via getScanData().
 *
 * ## Callback wiring
 * The LDS library uses plain C function pointers with no context parameter.
 * LidarSubsystem stores its address in a static instance_ pointer so that
 * static callback stubs can forward to the live object.
 *
 * ## Motor pin
 * The LD14P controls its motor entirely over serial (start/stop command
 * bytes). The motorPinCallback may still be invoked for setup bookkeeping;
 * the stub is a safe no-op.
 *
 * ## Threading
 *   beginThreadedPinned(6144, 4, 1, 0)  — core 0 | pri 4 | 1ms | 24 KB
 */
#pragma once

#include <Arduino.h>
#include <CustomDebug.h>
#include <LDS_LDROBOT_LD14P.h>
#include <ThreadedSubsystem.h>
#include <hal_thread.h>

namespace Subsystem {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
inline constexpr uint16_t kLidarMaxPoints = 720;

// ---------------------------------------------------------------------------
// LidarSetup
// ---------------------------------------------------------------------------
class LidarSetup : public Classes::BaseSetup {
 public:
  LidarSetup() = delete;

  /// @param serial       HardwareSerial port wired to the LD14P.
  /// @param rx_pin       GPIO number for the UART RX pin.
  /// @param tx_pin       GPIO number for the UART TX pin.
  /// @param rx_buf_size  HardwareSerial RX ring buffer size in bytes.
  ///                     512 bytes handles ~22 ms of burst at 230400 baud.
  /// @param scan_freq_hz Target scan frequency (2–8 Hz; default 6 Hz).
  LidarSetup(HardwareSerial& serial, int rx_pin, int tx_pin,
             uint32_t rx_buf_size = 512, float scan_freq_hz = 6.0f)
      : Classes::BaseSetup("LidarSubsystem"),
        serial_(serial),
        rx_pin_(rx_pin),
        tx_pin_(tx_pin),
        rx_buf_size_(rx_buf_size),
        scan_freq_hz_(scan_freq_hz) {}

  HardwareSerial& serial_;
  int rx_pin_;
  int tx_pin_;
  uint32_t rx_buf_size_;
  float scan_freq_hz_;
};

// ---------------------------------------------------------------------------
// LidarScanData — POD snapshot of one complete 360° scan
// ---------------------------------------------------------------------------
struct LidarScanData {
  float
      angles_deg[kLidarMaxPoints];  ///< Angle for each point (degrees, 0–360).
  float distances_mm[kLidarMaxPoints];  ///< Distance (mm); 0 = invalid.
  float qualities[kLidarMaxPoints];  ///< Intensity / quality (raw LDS value).
  uint16_t count = 0;                ///< Number of valid entries in this scan.
  bool valid = false;                ///< False until first scan completes.
  uint32_t scan_count = 0;           ///< Monotonically incrementing scan index.
};

// ---------------------------------------------------------------------------
// LidarSubsystem
// ---------------------------------------------------------------------------
class LidarSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  LidarSubsystem(const LidarSubsystem&) = delete;
  LidarSubsystem& operator=(const LidarSubsystem&) = delete;

  static LidarSubsystem& getInstance(const LidarSetup& setup) {
    static LidarSubsystem instance(setup);
    return instance;
  }

  // --- ThreadedSubsystem overrides ---
  bool init() override;
  void begin() override;   ///< Called once by task; calls lidar_.start().
  void update() override;  ///< Calls lidar_.loop(); runs at ~1 ms cadence.
  void pause() override;
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  /// @brief Thread-safe snapshot of the latest complete scan.
  /// @param out  Caller-supplied buffer filled under the data mutex.
  ///             Use a static or heap-allocated buffer — LidarScanData is
  ///             ~8.6 KB and will overflow small FreeRTOS stacks if placed
  ///             on the stack as a return-value temporary.
  void getScanData(LidarScanData& out) const;

  /// @brief Current scan frequency as reported by the LD14P (Hz).
  float getCurrentScanFreqHz();

 private:
  explicit LidarSubsystem(const LidarSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  // --- LDS callback stubs (no context param — bridged via instance_) ---
  static int serialReadCb();
  static size_t serialWriteCb(const uint8_t* buf, size_t len);
  static void scanPointCb(float angle_deg, float dist_mm, float quality,
                          bool scan_completed);
  static void motorPinCb(float value, LDS::lds_pin_t pin);
  static void infoCb(LDS::info_t code, String info);
  static void errorCb(LDS::result_t code, String aux);

  void onScanPoint(float angle_deg, float dist_mm, float quality,
                   bool scan_completed);

  const LidarSetup setup_;
  LDS_LDROBOT_LD14P lidar_;

  // Double-buffer: accum_ owned exclusively by update() task (no lock needed).
  // published_ protected by data_mutex_.
  LidarScanData accum_ = {};
  LidarScanData published_ = {};
  uint32_t completed_scans_ = 0;

  mutable Threads::Mutex data_mutex_;
  bool initialized_ = false;
  uint32_t last_log_ms_ = 0;

  static constexpr uint32_t kLogIntervalMs = 1000;

  /// Set in init(); used by static C callback stubs.
  static LidarSubsystem* instance_;
};

}  // namespace Subsystem
