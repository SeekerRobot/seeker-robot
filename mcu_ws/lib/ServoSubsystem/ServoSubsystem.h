/**
 * @file ServoSubsystem.h
 * @date 3/27/2026
 * @brief PCA9685-based servo subsystem with per-servo trapezoidal motion
 * profiles and total-movement-rate limiting to prevent current spikes.
 * @author Aldem Pido, Claude Code
 * Disclaimer: This file was written mostly with Claude Code.
 *
 * Safety: OE is never enabled until the user explicitly calls arm() after
 * setting initial angles for all attached servos.
 *
 * Usage:
 *   1. Construct with ServoSetup (configs, PCA9685 params, rate budget).
 *   2. init() — configures PCA9685, OE stays disabled.
 *   3. attach(i) for each servo you want active.
 *   4. setAngle(i, angle) for each attached servo (sets initial position).
 *   5. arm() — verifies all attached servos initialized, enables OE.
 *   6. Subsequent setAngle() calls are motion-profiled targets.
 *   7. disarm() to cut OE at any time.
 */
#pragma once
#include <CustomDebug.h>
#include <PCA9685.h>
#include <ThreadedSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>

#include <cmath>

namespace Subsystem {

static constexpr uint8_t kMaxServos = 16;
/// Delay (ms) between zeroing servo PWM outputs and disabling OE on disarm.
/// Gives servos time to react to the zero-pulse before power is cut.
static constexpr uint32_t kDisarmDelayMs = 200;

struct ServoConfig {
  uint8_t channel;     // PCA9685 channel (0–15)
  float min_angle;     // lower angle limit (degrees)
  float max_angle;     // upper angle limit (degrees)
  uint16_t min_pwm;    // 12-bit PCA9685 value at min_angle
  uint16_t max_pwm;    // 12-bit PCA9685 value at max_angle
  bool inverted;       // true → PWM mapping is reversed
  float max_velocity;  // degrees per second
  float max_accel;     // degrees per second^2
  /// Servo's full physical travel range (degrees). 180 for standard servos,
  /// 270 for wide-range servos. min_pwm/max_pwm correspond to min_angle/
  /// max_angle — this field documents what fraction of the physical range the
  /// software window occupies and is used by test tooling to set safe defaults.
  float total_angle_deg;
};

class ServoSetup : public Classes::BaseSetup {
 public:
  ~ServoSetup() = default;
  ServoSetup() = delete;

  /// @param wire       I2C bus.
  /// @param addr       PCA9685 7-bit address.
  /// @param oe_pin     Output-enable pin (-1 to skip OE control).
  /// @param configs    Pointer to ServoConfig array (copied internally).
  /// @param num_servos Number of entries in configs (max 16).
  /// @param max_total_rate Total deg/s budget across all servos (current-spike
  /// limit).
  /// @param pwm_freq_hz PCA9685 PWM frequency (default 50 Hz for standard
  /// servos).
  ServoSetup(TwoWire& wire, uint8_t addr, int8_t oe_pin,
             const ServoConfig* configs, uint8_t num_servos,
             float max_total_rate, float pwm_freq_hz = 50.0f)
      : Classes::BaseSetup("ServoSubsystem"),
        wire_(wire),
        addr_(addr),
        oe_pin_(oe_pin),
        configs_(configs),
        num_servos_(num_servos),
        max_total_rate_(max_total_rate),
        pwm_freq_hz_(pwm_freq_hz) {}

  TwoWire& wire_;
  uint8_t addr_;
  int8_t oe_pin_;
  const ServoConfig* configs_;
  uint8_t num_servos_;
  float max_total_rate_;
  float pwm_freq_hz_;
};

class ServoSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  ServoSubsystem(const ServoSubsystem&) = delete;
  ServoSubsystem& operator=(const ServoSubsystem&) = delete;

  static ServoSubsystem& getInstance(const ServoSetup& setup,
                                     Threads::Mutex& i2c_mutex) {
    static ServoSubsystem instance(setup, i2c_mutex);
    return instance;
  }

  // --- ThreadedSubsystem overrides ---
  bool init() override;
  void begin() override { return; }
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  // --- Servo control (call from any thread) ---

  /// @brief Set the target angle. First call per servo snaps immediately.
  void setAngle(uint8_t index, float angle);

  /// @brief Start driving a servo channel.
  void attach(uint8_t index);

  /// @brief Stop driving a servo (channel goes full-off).
  void detach(uint8_t index);

  /// @brief Enable OE. Fails if any attached servo is uninitialized.
  bool arm();

  /// @brief Disable OE immediately.
  void disarm();

  // --- Runtime config setters ---

  void setMaxVelocity(uint8_t index, float vel);
  void setMaxAccel(uint8_t index, float accel);
  void setInverted(uint8_t index, bool inv);
  void setAngleLimits(uint8_t index, float min_angle, float max_angle);
  void setPwmLimits(uint8_t index, uint16_t min_pwm, uint16_t max_pwm);
  void setTotalRateBudget(float budget);
  void setPwmFrequency(float hz);

  // --- Getters ---

  bool isArmed() const { return armed_; }
  bool isAttached(uint8_t index) const;
  bool isInitialized(uint8_t index) const;
  bool atTarget(uint8_t index, float tolerance = 0.5f) const;
  float getCurrentAngle(uint8_t index) const;
  float getTargetAngle(uint8_t index) const;
  float getVelocity(uint8_t index) const;
  const ServoConfig& getConfig(uint8_t index) const;
  uint8_t getNumServos() const { return setup_.num_servos_; }
  float getTotalRateBudget() const { return max_total_rate_; }

 private:
  explicit ServoSubsystem(const ServoSetup& setup, Threads::Mutex& i2c_mutex)
      : ThreadedSubsystem(setup),
        setup_(setup),
        i2c_mutex_(i2c_mutex),
        pwm_(setup.wire_, setup.addr_, setup.oe_pin_),
        max_total_rate_(setup.max_total_rate_) {
    uint8_t n = setup.num_servos_;
    if (n > kMaxServos) n = kMaxServos;
    for (uint8_t i = 0; i < n; i++) {
      configs_[i] = setup.configs_[i];
    }
  }

  struct ServoState {
    float current_angle = 0.0f;
    float target_angle = 0.0f;
    float velocity = 0.0f;
    bool attached = false;
    bool initialized = false;
    bool detached_by_disarm = false;
  };

  const ServoSetup setup_;
  Threads::Mutex& i2c_mutex_;
  Driver::PCA9685 pwm_;

  ServoConfig configs_[kMaxServos] = {};
  float max_total_rate_;
  ServoState state_[kMaxServos] = {};
  bool armed_ = false;
  uint32_t last_update_us_ = 0;

  uint16_t angleToPwm(const ServoConfig& cfg, float angle) const;
  float clampAngle(const ServoConfig& cfg, float angle) const;
};

}  // namespace Subsystem
