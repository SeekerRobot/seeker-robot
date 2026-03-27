/**
 * @file PCA9685.h
 * @date 3/27/2026
 * @brief PCA9685A 16-channel 12-bit PWM driver over I2C.
 * @author Aldem Pido, Claude Code
 * Disclaimer: file was written mostly by Claude Code.
 * Buffered writes with explicit flush via update().
 */
#pragma once
#include <Wire.h>
#include <cstdint>

namespace Driver {

class PCA9685 {
 public:
  static constexpr uint8_t kDefaultAddr = 0x40;
  static constexpr uint8_t kNumChannels = 16;
  static constexpr uint16_t kMaxDuty = 4095;
  static constexpr float kOscClock = 25000000.0f;

  /// @param wire I2C bus reference.
  /// @param addr 7-bit I2C address (default 0x40).
  /// @param oe_pin Output-enable pin (-1 to disable OE control).
  PCA9685(TwoWire& wire, uint8_t addr = kDefaultAddr, int8_t oe_pin = -1);

  /// @brief Initialize the PCA9685. Call once before any other method.
  /// @param freq_hz PWM frequency in Hz (24–1526).
  /// @return true on success.
  bool begin(float freq_hz = 50.0f);

  /// @brief Buffer a single channel's duty cycle. Does not touch I2C.
  /// @param channel 0–15.
  /// @param duty 12-bit value (0–4095).
  void set(uint8_t channel, uint16_t duty);

  /// @brief Buffer all channels to the same duty. Does not touch I2C.
  /// @param duty 12-bit value (0–4095).
  void setAll(uint16_t duty);

  /// @brief Flush dirty channels to the device over I2C.
  void update();

  /// @brief Change the PWM frequency. Sleeps the oscillator, sets prescaler, wakes.
  /// @param freq_hz 24–1526 Hz.
  void setFrequency(float freq_hz);

  /// @brief Write the MODE1 register directly.
  void setMode1(uint8_t value);

  /// @brief Write the MODE2 register directly.
  void setMode2(uint8_t value);

  /// @brief Read the MODE1 register.
  uint8_t getMode1();

  /// @brief Read the MODE2 register.
  uint8_t getMode2();

  /// @brief Read the PRE_SCALE register.
  uint8_t getPrescale();

  /// @brief Drive OE low (outputs active). No-op if no OE pin configured.
  void enableOutputs();

  /// @brief Drive OE high (outputs Hi-Z / low depending on OUTNE). No-op if no OE pin.
  void disableOutputs();

  /// @brief Software reset (writes MODE1 = 0x00).
  void reset();

  /// @brief Put the oscillator to sleep (low power).
  void sleep();

  /// @brief Wake the oscillator from sleep.
  void wake();

 private:
  enum Reg : uint8_t {
    MODE1         = 0x00,
    MODE2         = 0x01,
    SUBADR1       = 0x02,
    SUBADR2       = 0x03,
    SUBADR3       = 0x04,
    ALLCALLADR    = 0x05,
    LED0_ON_L     = 0x06,
    ALL_LED_ON_L  = 0xFA,
    ALL_LED_ON_H  = 0xFB,
    ALL_LED_OFF_L = 0xFC,
    ALL_LED_OFF_H = 0xFD,
    PRE_SCALE     = 0xFE,
  };

  // MODE1 bits
  static constexpr uint8_t kRestart = 0x80;
  static constexpr uint8_t kExtclk  = 0x40;
  static constexpr uint8_t kAutoInc = 0x20;
  static constexpr uint8_t kSleep   = 0x10;
  static constexpr uint8_t kAllCall = 0x01;

  void writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);
  void writeChannel(uint8_t channel, uint16_t on, uint16_t off);
  void writeAllChannels(uint16_t on, uint16_t off);

  TwoWire& wire_;
  uint8_t addr_;
  int8_t oe_pin_;

  uint16_t duty_[kNumChannels] = {};
  uint16_t dirty_ = 0;       // bitmask: bit N = channel N needs flush
  bool all_dirty_ = false;    // true when setAll() queued a bulk write
  uint16_t all_duty_ = 0;
};

}  // namespace Driver
