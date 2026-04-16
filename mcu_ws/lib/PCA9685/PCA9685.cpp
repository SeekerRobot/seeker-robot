/**
 * @file PCA9685.cpp
 * @date 3/27/2026
 * @brief PCA9685A driver implementation.
 * @author Aldem Pido, Claude Code
 * Disclaimer: This file was written mostly by Claude Code.
 */
#include "PCA9685.h"

#include <Arduino.h>

namespace Driver {

PCA9685::PCA9685(TwoWire& wire, uint8_t addr, int8_t oe_pin)
    : wire_(wire), addr_(addr), oe_pin_(oe_pin) {}

bool PCA9685::begin(float freq_hz) {
  if (oe_pin_ >= 0) {
    pinMode(oe_pin_, OUTPUT);
    disableOutputs();
  }

  reset();
  // Auto-increment + allcall
  setMode1(kAutoInc | kAllCall);
  // Totem-pole outputs
  setMode2(0x04);
  setFrequency(freq_hz);

  // Zero all outputs — OE left disabled, caller must enableOutputs() when ready
  setAll(0);
  update();
  return true;
}

// --- Buffered channel API ---

void PCA9685::set(uint8_t channel, uint16_t duty) {
  if (channel >= kNumChannels) return;
  if (duty > kMaxDuty) duty = kMaxDuty;
  duty_[channel] = duty;
  dirty_ |= (1u << channel);
}

void PCA9685::setAll(uint16_t duty) {
  if (duty > kMaxDuty) duty = kMaxDuty;
  for (uint8_t i = 0; i < kNumChannels; i++) {
    duty_[i] = duty;
  }
  dirty_ = 0;
  all_dirty_ = true;
  all_duty_ = duty;
}

void PCA9685::update() {
  // Bulk write via ALL_LED registers if setAll() was called
  if (all_dirty_) {
    if (all_duty_ == 0) {
      writeAllChannels(0, kMaxDuty + 1);
    } else if (all_duty_ >= kMaxDuty) {
      writeAllChannels(kMaxDuty + 1, 0);
    } else {
      writeAllChannels(0, all_duty_);
    }
    all_dirty_ = false;
  }

  // Per-channel overrides (or standalone set() calls).
  // Phase-stagger each channel by (ch * 4096 / 16) = ch * 256 ticks so
  // pulses don't all start at tick 0 simultaneously, which would cause a
  // large current spike at the top of every PWM cycle.
  while (dirty_) {
    uint8_t ch = __builtin_ctz(dirty_);
    uint16_t d = duty_[ch];
    if (d == 0) {
      writeChannel(ch, 0, kMaxDuty + 1);  // full off via bit 12
    } else if (d >= kMaxDuty) {
      writeChannel(ch, kMaxDuty + 1, 0);  // full on via bit 12
    } else {
      uint16_t on_val  = static_cast<uint16_t>(ch) * (4096u / kNumChannels);
      uint16_t off_val = (on_val + d) & 0x0FFFu;
      writeChannel(ch, on_val, off_val);
    }
    dirty_ &= ~(1u << ch);
  }
}

// --- Configuration ---

void PCA9685::setFrequency(float freq_hz) {
  if (freq_hz < 24.0f) freq_hz = 24.0f;
  if (freq_hz > 1526.0f) freq_hz = 1526.0f;

  uint8_t prescale =
      static_cast<uint8_t>((kOscClock / (4096.0f * freq_hz)) + 0.5f) - 1;

  uint8_t old_mode = readReg(Reg::MODE1);
  writeReg(Reg::MODE1, (old_mode & ~kRestart) | kSleep);
  writeReg(Reg::PRE_SCALE, prescale);
  writeReg(Reg::MODE1, old_mode & ~kRestart);
  delayMicroseconds(500);  // oscillator stabilization (datasheet: 500 µs)
  writeReg(Reg::MODE1, old_mode | kRestart);
}

void PCA9685::setMode1(uint8_t value) { writeReg(Reg::MODE1, value); }
void PCA9685::setMode2(uint8_t value) { writeReg(Reg::MODE2, value); }
uint8_t PCA9685::getMode1() { return readReg(Reg::MODE1); }
uint8_t PCA9685::getMode2() { return readReg(Reg::MODE2); }
uint8_t PCA9685::getPrescale() { return readReg(Reg::PRE_SCALE); }

// --- OE pin ---

void PCA9685::enableOutputs() {
  if (oe_pin_ >= 0) digitalWrite(oe_pin_, LOW);
}

void PCA9685::disableOutputs() {
  if (oe_pin_ >= 0) digitalWrite(oe_pin_, HIGH);
}

// --- Power management ---

void PCA9685::reset() {
  writeReg(Reg::MODE1, 0x00);
  delay(10);
}

void PCA9685::sleep() {
  uint8_t mode = readReg(Reg::MODE1);
  writeReg(Reg::MODE1, mode | kSleep);
}

void PCA9685::wake() {
  uint8_t mode = readReg(Reg::MODE1);
  writeReg(Reg::MODE1, mode & ~kSleep);
  delayMicroseconds(500);
  if (mode & kRestart) {
    writeReg(Reg::MODE1, (mode & ~kSleep) | kRestart);
  }
}

// --- Private I2C helpers ---

void PCA9685::writeReg(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(value);
  wire_.endTransmission();
}

uint8_t PCA9685::readReg(uint8_t reg) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.endTransmission();
  wire_.requestFrom(addr_, static_cast<uint8_t>(1));
  return wire_.read();
}

void PCA9685::writeChannel(uint8_t channel, uint16_t on, uint16_t off) {
  uint8_t reg = Reg::LED0_ON_L + 4 * channel;
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(static_cast<uint8_t>(on & 0xFF));
  wire_.write(static_cast<uint8_t>((on >> 8) & 0x1F));
  wire_.write(static_cast<uint8_t>(off & 0xFF));
  wire_.write(static_cast<uint8_t>((off >> 8) & 0x1F));
  wire_.endTransmission();
}

void PCA9685::writeAllChannels(uint16_t on, uint16_t off) {
  wire_.beginTransmission(addr_);
  wire_.write(static_cast<uint8_t>(Reg::ALL_LED_ON_L));
  wire_.write(static_cast<uint8_t>(on & 0xFF));
  wire_.write(static_cast<uint8_t>((on >> 8) & 0x1F));
  wire_.write(static_cast<uint8_t>(off & 0xFF));
  wire_.write(static_cast<uint8_t>((off >> 8) & 0x1F));
  wire_.endTransmission();
}

}  // namespace Driver
