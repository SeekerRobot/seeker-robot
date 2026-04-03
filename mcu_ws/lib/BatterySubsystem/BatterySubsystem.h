/**
 * @file BatterySubsystem.h
 * @author Aldem Pido
 * @date 4/2/2026
 * @brief ThreadedSubsystem that reads battery voltage via ADC and applies a
 *        two-point linear calibration model.
 *
 * ## Calibration
 * Provide two (raw_adc, voltage_V) pairs measured at known voltages.
 * The subsystem fits a line through both points and applies it to every
 * reading.  Readings outside the calibration range are linearly extrapolated.
 *
 * ## Oversampling
 * `num_samples` ADC readings are averaged per update cycle to reduce noise.
 * Values 1–64 are accepted; out-of-range values are clamped.
 */
#pragma once

#include <Arduino.h>
#include <CustomDebug.h>
#include <ThreadedSubsystem.h>
#include <hal_thread.h>

namespace Subsystem {

// ---- Calibration model -------------------------------------------------------

/// @brief Two-point linear calibration: maps raw ADC counts to real voltage.
struct BatteryCalibration {
  uint16_t raw_lo;   ///< Raw ADC reading at the low-voltage calibration point.
  float    volt_lo;  ///< Actual voltage (V) at raw_lo.
  uint16_t raw_hi;   ///< Raw ADC reading at the high-voltage calibration point.
  float    volt_hi;  ///< Actual voltage (V) at raw_hi.

  constexpr BatteryCalibration(uint16_t raw_lo, float volt_lo,
                                uint16_t raw_hi, float volt_hi)
      : raw_lo(raw_lo), volt_lo(volt_lo), raw_hi(raw_hi), volt_hi(volt_hi) {}

  /// @brief Convert a raw ADC count to volts via linear interpolation.
  float toVoltage(uint16_t raw) const {
    if (raw_hi == raw_lo) return volt_lo;  // degenerate guard
    return volt_lo + (volt_hi - volt_lo) *
               (static_cast<float>(raw) - static_cast<float>(raw_lo)) /
               static_cast<float>(raw_hi - raw_lo);
  }
};

// ---- Setup struct ------------------------------------------------------------

class BatterySetup : public Classes::BaseSetup {
 public:
  BatterySetup() = delete;

  /// @param adc_pin      GPIO pin wired to the battery voltage divider.
  /// @param calibration  Two-point linear calibration model.
  /// @param num_samples  ADC samples averaged per reading (clamped to 1–64).
  BatterySetup(int adc_pin, const BatteryCalibration& calibration,
               uint8_t num_samples = 8)
      : Classes::BaseSetup("BatterySubsystem"),
        adc_pin_(adc_pin),
        calibration_(calibration),
        num_samples_(num_samples < 1    ? 1
                     : num_samples > 64 ? 64
                                        : num_samples) {}

  int adc_pin_;
  BatteryCalibration calibration_;
  uint8_t num_samples_;
};

// ---- Subsystem ---------------------------------------------------------------

class BatterySubsystem : public Subsystem::ThreadedSubsystem {
 public:
  BatterySubsystem(const BatterySubsystem&) = delete;
  BatterySubsystem& operator=(const BatterySubsystem&) = delete;

  static BatterySubsystem& getInstance(const BatterySetup& setup) {
    static BatterySubsystem instance(setup);
    return instance;
  }

  bool        init() override;
  void        begin() override { return; }
  void        update() override;
  void        pause() override { return; }
  void        reset() override;
  const char* getInfo() override { return setup_.getId(); }

  /// @brief Latest calibrated battery voltage in volts.  Thread-safe.
  float getVoltage() const;

  /// @brief Latest raw ADC reading (0–4095).  Thread-safe.
  uint16_t getRawAdc() const;

 private:
  explicit BatterySubsystem(const BatterySetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  const BatterySetup  setup_;
  mutable Threads::Mutex data_mutex_;

  float    voltage_     = 0.0f;
  uint16_t raw_adc_     = 0;
  bool     initialized_ = false;
};

}  // namespace Subsystem
