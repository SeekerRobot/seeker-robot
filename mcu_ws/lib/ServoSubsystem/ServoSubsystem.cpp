/**
 * @file ServoSubsystem.cpp
 * @date 3/27/2026
 * @brief ServoSubsystem implementation — trapezoidal motion profiles with
 * total-movement-rate limiting.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include "ServoSubsystem.h"

#include <Arduino.h>

namespace Subsystem {

// --- ThreadedSubsystem overrides ---

bool ServoSubsystem::init() {
  if (setup_.num_servos_ > kMaxServos) {
    Debug::printf(Debug::Level::ERROR, "[Servo] num_servos %u exceeds max %u",
                  setup_.num_servos_, kMaxServos);
    return false;
  }

  Threads::Scope lock(i2c_mutex_);
  if (!setup_.wire_.begin()) {
    Debug::printf(Debug::Level::ERROR, "[Servo] Wire.begin() failure");
    return false;
  }

  // PCA9685 begin() zeroes all channels and leaves OE disabled
  pwm_.begin(setup_.pwm_freq_hz_);

  Debug::printf(
      Debug::Level::INFO,
      "[Servo] PCA9685 @ 0x%02X, %.0f Hz, %u servos, budget %.0f deg/s",
      setup_.addr_, setup_.pwm_freq_hz_, setup_.num_servos_, max_total_rate_);
  return true;
}

void ServoSubsystem::update() {
  uint32_t now = micros();
  uint32_t elapsed = now - last_update_us_;
  last_update_us_ = now;
  float dt = elapsed * 1e-6f;

  if (dt > 0.1f) dt = 0.1f;
  if (dt <= 0.0f) return;

  float desired_vel[kMaxServos] = {};
  float total_rate = 0.0f;

  // --- Phase 1: compute desired velocity per servo (trapezoidal profile) ---
  for (uint8_t i = 0; i < setup_.num_servos_; i++) {
    if (!state_[i].attached || !state_[i].initialized) continue;

    const ServoConfig& cfg = configs_[i];
    float error = state_[i].target_angle - state_[i].current_angle;
    float distance = fabsf(error);

    if (distance < 0.01f) {
      state_[i].current_angle = state_[i].target_angle;
      state_[i].velocity = 0.0f;
      desired_vel[i] = 0.0f;
      continue;
    }

    float direction = (error > 0.0f) ? 1.0f : -1.0f;

    float v_decel = sqrtf(2.0f * cfg.max_accel * distance);
    float v_target = fminf(v_decel, cfg.max_velocity) * direction;

    float dv = v_target - state_[i].velocity;
    float max_dv = cfg.max_accel * dt;
    if (dv > max_dv) dv = max_dv;
    if (dv < -max_dv) dv = -max_dv;

    desired_vel[i] = state_[i].velocity + dv;
    total_rate += fabsf(desired_vel[i]);
  }

  // --- Phase 2: scale velocities if over the total-movement budget ---
  float scale = 1.0f;
  if (total_rate > max_total_rate_ && total_rate > 0.0f) {
    scale = max_total_rate_ / total_rate;
  }

  // --- Phase 3: integrate positions, convert to PWM, flush ---
  bool any_dirty = false;
  {
    Threads::Scope lock(i2c_mutex_);
    for (uint8_t i = 0; i < setup_.num_servos_; i++) {
      if (!state_[i].attached || !state_[i].initialized) continue;

      const ServoConfig& cfg = configs_[i];

      state_[i].velocity = desired_vel[i] * scale;
      float new_angle = state_[i].current_angle + state_[i].velocity * dt;
      new_angle = clampAngle(cfg, new_angle);

      // Snap to target if the integration step crossed it — prevents the
      // profiler from oscillating around intermediate positions.
      float cur = state_[i].current_angle;
      float tgt = state_[i].target_angle;
      if ((cur <= tgt && new_angle >= tgt) ||
          (cur >= tgt && new_angle <= tgt)) {
        new_angle = tgt;
        state_[i].velocity = 0.0f;
      }
      state_[i].current_angle = new_angle;

      pwm_.set(cfg.channel, angleToPwm(cfg, state_[i].current_angle));
      any_dirty = true;
    }
    if (any_dirty) pwm_.update();
  }
}

void ServoSubsystem::pause() { disarm(); }

void ServoSubsystem::reset() {
  disarm();
  for (uint8_t i = 0; i < kMaxServos; i++) {
    state_[i] = ServoState{};
  }
  Threads::Scope lock(i2c_mutex_);
  pwm_.setAll(0);
  pwm_.update();
}

// --- Servo control ---

void ServoSubsystem::setAngle(uint8_t index, float angle) {
  if (index >= setup_.num_servos_) return;
  angle = clampAngle(configs_[index], angle);

  if (!state_[index].initialized) {
    state_[index].current_angle = angle;
    state_[index].velocity = 0.0f;
    state_[index].initialized = true;
  }
  state_[index].target_angle = angle;
}

void ServoSubsystem::attach(uint8_t index) {
  if (index >= setup_.num_servos_) return;
  state_[index].attached = true;
  Debug::printf(Debug::Level::INFO, "[Servo] ch%u attached",
                configs_[index].channel);
}

void ServoSubsystem::detach(uint8_t index) {
  if (index >= setup_.num_servos_) return;
  state_[index].attached = false;
  state_[index].velocity = 0.0f;

  Threads::Scope lock(i2c_mutex_);
  pwm_.set(configs_[index].channel, 0);
  pwm_.update();

  Debug::printf(Debug::Level::INFO, "[Servo] ch%u detached",
                configs_[index].channel);
}

bool ServoSubsystem::arm() {
  for (uint8_t i = 0; i < setup_.num_servos_; i++) {
    if (state_[i].detached_by_disarm) {
      state_[i].detached_by_disarm = false;
      attach(i);
    }
  }

  for (uint8_t i = 0; i < setup_.num_servos_; i++) {
    if (state_[i].attached && !state_[i].initialized) {
      Debug::printf(
          Debug::Level::ERROR,
          "[Servo] Cannot arm — servo %u attached but not initialized", i);
      return false;
    }
  }

  {
    Threads::Scope lock(i2c_mutex_);
    for (uint8_t i = 0; i < setup_.num_servos_; i++) {
      if (!state_[i].attached) continue;
      pwm_.set(configs_[i].channel,
               angleToPwm(configs_[i], state_[i].current_angle));
    }
    pwm_.update();
    pwm_.enableOutputs();
  }

  armed_ = true;
  last_update_us_ = micros();
  Debug::printf(Debug::Level::INFO, "[Servo] Armed — OE enabled");
  return true;
}

void ServoSubsystem::disarm() {
  for (uint8_t i = 0; i < setup_.num_servos_; i++) {
    if (state_[i].attached) {
      detach(i);
      state_[i].detached_by_disarm = true;
    }
  }
  // Give servos time to react to the zero-pulse before cutting OE, so they
  // don't jerk from whatever position they were at when power is removed.
  delay(kDisarmDelayMs);
  {
    Threads::Scope lock(i2c_mutex_);
    pwm_.disableOutputs();
  }
  armed_ = false;
  Debug::printf(Debug::Level::INFO, "[Servo] Disarmed — OE disabled");
}

// --- Runtime config setters ---

void ServoSubsystem::setMaxVelocity(uint8_t index, float vel) {
  if (index >= setup_.num_servos_ || vel <= 0.0f) return;
  configs_[index].max_velocity = vel;
}

void ServoSubsystem::setMaxAccel(uint8_t index, float accel) {
  if (index >= setup_.num_servos_ || accel <= 0.0f) return;
  configs_[index].max_accel = accel;
}

void ServoSubsystem::setInverted(uint8_t index, bool inv) {
  if (index >= setup_.num_servos_) return;
  configs_[index].inverted = inv;
}

void ServoSubsystem::setAngleLimits(uint8_t index, float min_a, float max_a) {
  if (index >= setup_.num_servos_ || min_a >= max_a) return;
  configs_[index].min_angle = min_a;
  configs_[index].max_angle = max_a;
}

void ServoSubsystem::setPwmLimits(uint8_t index, uint16_t min_p,
                                  uint16_t max_p) {
  if (index >= setup_.num_servos_ || min_p >= max_p) return;
  if (max_p > Driver::PCA9685::kMaxDuty) return;
  configs_[index].min_pwm = min_p;
  configs_[index].max_pwm = max_p;
}

void ServoSubsystem::setTotalRateBudget(float budget) {
  if (budget > 0.0f) max_total_rate_ = budget;
}

void ServoSubsystem::setPwmFrequency(float hz) {
  Threads::Scope lock(i2c_mutex_);
  pwm_.setFrequency(hz);
}

// --- Getters ---

bool ServoSubsystem::isAttached(uint8_t index) const {
  if (index >= setup_.num_servos_) return false;
  return state_[index].attached;
}

bool ServoSubsystem::isInitialized(uint8_t index) const {
  if (index >= setup_.num_servos_) return false;
  return state_[index].initialized;
}

bool ServoSubsystem::atTarget(uint8_t index, float tolerance) const {
  if (index >= setup_.num_servos_) return false;
  return fabsf(state_[index].current_angle - state_[index].target_angle) <=
         tolerance;
}

float ServoSubsystem::getCurrentAngle(uint8_t index) const {
  if (index >= setup_.num_servos_) return 0.0f;
  return state_[index].current_angle;
}

float ServoSubsystem::getTargetAngle(uint8_t index) const {
  if (index >= setup_.num_servos_) return 0.0f;
  return state_[index].target_angle;
}

float ServoSubsystem::getVelocity(uint8_t index) const {
  if (index >= setup_.num_servos_) return 0.0f;
  return state_[index].velocity;
}

const ServoConfig& ServoSubsystem::getConfig(uint8_t index) const {
  return configs_[index];
}

// --- Private helpers ---

uint16_t ServoSubsystem::angleToPwm(const ServoConfig& cfg, float angle) const {
  float t = (angle - cfg.min_angle) / (cfg.max_angle - cfg.min_angle);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  if (cfg.inverted) t = 1.0f - t;
  return static_cast<uint16_t>(cfg.min_pwm + t * (cfg.max_pwm - cfg.min_pwm));
}

float ServoSubsystem::clampAngle(const ServoConfig& cfg, float angle) const {
  if (angle < cfg.min_angle) return cfg.min_angle;
  if (angle > cfg.max_angle) return cfg.max_angle;
  return angle;
}

}  // namespace Subsystem
