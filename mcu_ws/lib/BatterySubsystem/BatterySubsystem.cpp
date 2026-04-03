/**
 * @file BatterySubsystem.cpp
 * @author Aldem Pido
 * @date 4/2/2026
 */
#include "BatterySubsystem.h"

namespace Subsystem {

bool BatterySubsystem::init() {
  pinMode(setup_.adc_pin_, INPUT);
  initialized_ = true;
  Debug::printf(Debug::Level::INFO,
                "[Battery] init ok — pin %d, samples %u, cal [%u→%.3fV, "
                "%u→%.3fV]",
                setup_.adc_pin_, setup_.num_samples_,
                setup_.calibration_.raw_lo, setup_.calibration_.volt_lo,
                setup_.calibration_.raw_hi, setup_.calibration_.volt_hi);
  return true;
}

void BatterySubsystem::update() {
  if (!initialized_) return;

  // Oversample: average num_samples_ ADC readings to reduce noise.
  uint32_t acc = 0;
  for (uint8_t i = 0; i < setup_.num_samples_; ++i) {
    acc += static_cast<uint32_t>(analogRead(setup_.adc_pin_));
  }
  const uint16_t raw = static_cast<uint16_t>(acc / setup_.num_samples_);
  const float v = setup_.calibration_.toVoltage(raw);

  {
    Threads::Scope lock(data_mutex_);
    raw_adc_ = raw;
    voltage_ = v;
  }

  Debug::printf(Debug::Level::VERBOSE, "[Battery] raw=%u  voltage=%.3fV", raw,
                v);
}

void BatterySubsystem::reset() {
  Threads::Scope lock(data_mutex_);
  voltage_ = 0.0f;
  raw_adc_ = 0;
}

float BatterySubsystem::getVoltage() const {
  Threads::Scope lock(data_mutex_);
  return voltage_;
}

uint16_t BatterySubsystem::getRawAdc() const {
  Threads::Scope lock(data_mutex_);
  return raw_adc_;
}

}  // namespace Subsystem
