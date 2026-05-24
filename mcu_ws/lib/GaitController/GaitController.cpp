/**
 * @file GaitController.cpp
 * @date 4/3/2026
 * @brief GaitController implementation — tripod gait state machine.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include "GaitController.h"

#include <Arduino.h>
#include <CustomDebug.h>

namespace Gait {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

GaitController::GaitController(const GaitSetup& setup)
    : ThreadedSubsystem(setup),
      setup_(setup),
      kin_(setup.kinematics),
      servos_(setup.servos) {}

// ---------------------------------------------------------------------------
// ThreadedSubsystem overrides
// ---------------------------------------------------------------------------

bool GaitController::init() {
  if (!kin_) {
    Debug::printf(Debug::Level::ERROR,
                  "[Gait] init failed — kinematics pointer is null");
    return false;
  }
  if (!servos_) {
    Debug::printf(Debug::Level::ERROR,
                  "[Gait] init failed — servos pointer is null");
    return false;
  }
  Debug::printf(Debug::Level::INFO, "[Gait] init OK");
  return true;
}

void GaitController::begin() {
  // Move all legs to neutral and push the resulting angles
  Kinematics::SolveResult r = kin_->standNeutral();
  for (uint8_t i = 0; i < kNumLegs; i++) {
    if (r.valid[i]) pushLegAngles(i, r.legs[i]);
  }

  // Arm the servo OE — safe because standNeutral initialised every channel
  if (!servos_->isArmed()) {
    if (!servos_->arm()) {
      Debug::printf(Debug::Level::ERROR,
                    "[Gait] begin — servo arm() failed, servos not armed");
    }
  }

  body_pos_ = {};
  body_yaw_ = 0.f;
  initPhases();
  last_us_ = micros();
  last_cmd_ms_ = millis();

  Debug::printf(Debug::Level::INFO, "[Gait] begin — neutral stance, armed");
}

void GaitController::update() {
  uint32_t now = micros();
  float dt = static_cast<float>(now - last_us_) * 1e-6f;
  last_us_ = now;
  if (dt > 0.1f) dt = 0.1f;
  if (dt <= 0.f) return;

  // Snapshot command and state under the mutex
  VelocityCommand vel;
  GaitState state;
  uint32_t last_cmd_ms;
  uint32_t cmd_timeout_ms;
  {
    Threads::Scope lock(state_mutex_);
    vel = cmd_;
    state = state_;
    last_cmd_ms = last_cmd_ms_;
    cmd_timeout_ms = cmd_timeout_ms_;
  }

  // Dead-man watchdog: if we're walking but no new command has arrived in
  // cmd_timeout_ms, treat the publisher as dead and request a clean stop.
  // Uses uint32 wrap-safe subtraction; safe as long as timeout < 35 minutes.
  if (state == GaitState::WALKING && cmd_timeout_ms > 0 &&
      (millis() - last_cmd_ms) > cmd_timeout_ms) {
    Debug::printf(Debug::Level::WARN,
                  "[Gait] cmd_vel watchdog (%u ms) — auto-disabling",
                  (unsigned)cmd_timeout_ms);
    disable();  // WALKING → STOPPING; next tick completes the transition
    state = GaitState::STOPPING;
    vel = {};
  }

  if (state == GaitState::IDLE) return;

  // During STOPPING, accept no new velocity — legs complete current arcs only
  if (state == GaitState::STOPPING) vel = {};

  // Integrate body pose (velocity is in m/s; body_pos_ is in mm)
  float yaw_rad = body_yaw_ * kDegToRad;
  float cos_yaw = cosf(yaw_rad);
  float sin_yaw = sinf(yaw_rad);
  body_pos_.x += (vel.vx * cos_yaw - vel.vy * sin_yaw) * dt * 1000.f;
  body_pos_.y += (vel.vx * sin_yaw + vel.vy * cos_yaw) * dt * 1000.f;
  body_yaw_ += vel.wz * kRadToDeg * dt;

  // Solve stance IK for the current body pose — all six legs re-solved here;
  // flight legs will override their stance angles in the loop below.
  Kinematics::SolveResult stance = kin_->setBodyPose(body_pos_, body_yaw_);

  // Per-leg gait update
  const float phase_inc = dt / (setup_.gait.cycle_time_s * 0.5f);

  for (uint8_t i = 0; i < kNumLegs; i++) {
    LegGaitState& ls = leg_state_[i];

    // Frozen stance legs (during STOPPING) don't advance phase
    if (ls.frozen) {
      if (stance.valid[i]) pushLegAngles(i, stance.legs[i]);
      continue;
    }

    // Advance phase and wrap
    ls.phase_t += phase_inc;
    if (ls.phase_t >= 1.f) ls.phase_t -= 1.f;

    const bool flying = (ls.phase_t >= 0.5f);

    // --- Flight entry ---
    if (flying && !ls.was_flying) {
      ls.lift_start = kin_->getFootWorld(i);
      ls.swing_target = computeStepTarget(i, vel);
    }

    // --- Touchdown ---
    if (!flying && ls.was_flying) {
      kin_->setFootWorld(i, ls.swing_target);
      if (state == GaitState::STOPPING) {
        ls.frozen = true;  // leg has landed — freeze it in stance
      }
    }

    ls.was_flying = flying;

    if (flying) {
      // Compute swing arc position and solve IK
      float s = (ls.phase_t - 0.5f) / 0.5f;  // [0, 1] across flight window
      Kinematics::Vec3 arc = swingArc(i, s);
      Kinematics::LegAngles a;
      if (kin_->solveFootWorld(i, arc, a)) {
        pushLegAngles(i, a);
      }
    } else {
      // Stance — use IK already solved by setBodyPose above
      if (stance.valid[i]) pushLegAngles(i, stance.legs[i]);
    }
  }

  // STOPPING → IDLE once all legs are frozen (all in-flight legs have landed)
  if (state == GaitState::STOPPING && !anyFlying()) {
    bool transitioned = false;
    {
      Threads::Scope lock(state_mutex_);
      if (state_ == GaitState::STOPPING) {
        state_ = GaitState::IDLE;
        transitioned = true;
      }
    }
    if (transitioned) {
      Debug::printf(Debug::Level::INFO, "[Gait] all legs landed — IDLE");
      // Rebase the world frame so the next walking session starts with
      // body_pos_ / body_yaw_ at zero. Prevents unbounded accumulation
      // across sessions that otherwise eventually saturates stance hips.
      rebaseWorldFrame();
    }
  }
}

void GaitController::pause() { stop(); }

void GaitController::reset() {
  stop();
  body_pos_ = {};
  body_yaw_ = 0.f;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void GaitController::setVelocity(float vx, float vy, float wz) {
  Threads::Scope lock(state_mutex_);
  cmd_ = {vx, vy, wz};
  last_cmd_ms_ = millis();
}

void GaitController::enable() {
  Threads::Scope lock(state_mutex_);
  // Refresh the watchdog timestamp even when the state transition is a
  // no-op — a publisher re-arming a walk shouldn't immediately trip the
  // timeout if its last setVelocity happened just over the threshold ago.
  last_cmd_ms_ = millis();
  if (state_ == GaitState::IDLE) {
    state_ = GaitState::WALKING;
    initPhases();  // reset phase to sync gait and unfreeze all legs
    Debug::printf(Debug::Level::INFO, "[Gait] enabled — WALKING");
  }
}

void GaitController::disable() {
  Threads::Scope lock(state_mutex_);
  if (state_ == GaitState::WALKING) {
    state_ = GaitState::STOPPING;
    cmd_ = {};  // zero velocity so step targets return to neutral

    // Freeze stance legs immediately; in-flight legs will freeze on touchdown
    for (uint8_t i = 0; i < kNumLegs; i++) {
      if (!leg_state_[i].was_flying) {
        leg_state_[i].frozen = true;
      }
    }
    Debug::printf(Debug::Level::INFO,
                  "[Gait] disable requested — STOPPING (waiting for landing)");
  }
}

void GaitController::stop() {
  {
    Threads::Scope lock(state_mutex_);
    state_ = GaitState::IDLE;
    cmd_ = {};
  }

  // Snap all legs to neutral and reset tracking state
  body_pos_ = {};
  body_yaw_ = 0.f;
  Kinematics::SolveResult r = kin_->standNeutral();
  for (uint8_t i = 0; i < kNumLegs; i++) {
    if (r.valid[i]) pushLegAngles(i, r.legs[i]);
    leg_state_[i] = {};
  }
  initPhases();
  Debug::printf(Debug::Level::INFO, "[Gait] stop — snapped to neutral");
}

GaitState GaitController::getState() const {
  Threads::Scope lock(state_mutex_);
  return state_;
}

VelocityCommand GaitController::getVelocity() const {
  Threads::Scope lock(state_mutex_);
  return cmd_;
}

void GaitController::setStepHeight(float mm) {
  Threads::Scope lock(state_mutex_);
  setup_.gait.step_height_mm = mm;
}

void GaitController::setCycleTime(float s) {
  Threads::Scope lock(state_mutex_);
  setup_.gait.cycle_time_s = s;
}

void GaitController::setStepScale(float x) {
  Threads::Scope lock(state_mutex_);
  setup_.gait.step_scale = x;
}

void GaitController::setCommandTimeout(uint32_t ms) {
  Threads::Scope lock(state_mutex_);
  cmd_timeout_ms_ = ms;
  // Reset the reference so the new timeout counts from this moment rather
  // than from a stale timestamp that could have already exceeded it.
  last_cmd_ms_ = millis();
}

GaitConfig GaitController::getGaitConfig() const {
  Threads::Scope lock(state_mutex_);
  return setup_.gait;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void GaitController::initPhases() {
  for (uint8_t leg : kGroupA) {
    leg_state_[leg].phase_t = 0.f;
    leg_state_[leg].was_flying = false;
    leg_state_[leg].frozen = false;
  }
  for (uint8_t leg : kGroupB) {
    leg_state_[leg].phase_t = 0.5f;
    // was_flying=false so the first tick fires the flight-entry block and
    // captures lift_start / swing_target from the neutral stance. Setting it
    // true here skips that init and leaves those fields zero → swingArc
    // interpolates between (0,0,0) and (0,0,0), IK fails, and the leg stays
    // stuck at neutral forever.
    leg_state_[leg].was_flying = false;
    leg_state_[leg].frozen = false;
  }
}

void GaitController::pushLegAngles(uint8_t leg,
                                   const Kinematics::LegAngles& a) {
  servos_->setAngle(setup_.gait.leg_servo_hip[leg], a.hip);
  servos_->setAngle(setup_.gait.leg_servo_knee[leg], a.knee);
}

Kinematics::Vec3 GaitController::computeStepTarget(uint8_t leg,
                                                   const VelocityCommand& vel) {
  // Neutral foot position in body frame → transform to world frame at current
  // body pose so we have a stable reference for the step offset. Uses the
  // active stand-height neutral (not the leg's built-in default) so step
  // targets track the configured body height.
  Kinematics::Vec3 neutral_body = kin_->neutralFootBody(leg);
  float cos_yaw = cosf(body_yaw_ * kDegToRad);
  float sin_yaw = sinf(body_yaw_ * kDegToRad);

  float nx_world =
      body_pos_.x + cos_yaw * neutral_body.x - sin_yaw * neutral_body.y;
  float ny_world =
      body_pos_.y + sin_yaw * neutral_body.x + cos_yaw * neutral_body.y;

  // Half-cycle velocity lookahead (m/s → mm; rotated into world frame)
  float half_t_mm =
      setup_.gait.cycle_time_s * 0.5f * setup_.gait.step_scale * 1000.f;
  float dx = (vel.vx * cos_yaw - vel.vy * sin_yaw) * half_t_mm;
  float dy = (vel.vx * sin_yaw + vel.vy * cos_yaw) * half_t_mm;

  // Foot lands at same z as it lifted from (ground level for this leg)
  Kinematics::Vec3 candidate = {nx_world + dx, ny_world + dy,
                                kin_->getFootWorld(leg).z};

  // Reachability check — binary search inward if out of bounds
  if (kin_->isReachable(leg, candidate)) return candidate;

  Kinematics::Vec3 lo = kin_->getFootWorld(leg);  // known reachable
  Kinematics::Vec3 hi = candidate;
  for (int iter = 0; iter < 8; iter++) {
    Kinematics::Vec3 mid = {(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, lo.z};
    if (kin_->isReachable(leg, mid))
      lo = mid;
    else
      hi = mid;
  }
  return lo;
}

Kinematics::Vec3 GaitController::swingArc(uint8_t leg, float s) const {
  const LegGaitState& ls = leg_state_[leg];
  float ss = smoothstep(s);
  return {ls.lift_start.x + (ls.swing_target.x - ls.lift_start.x) * ss,
          ls.lift_start.y + (ls.swing_target.y - ls.lift_start.y) * ss,
          ls.lift_start.z + setup_.gait.step_height_mm * sinf(kPi * s)};
}

bool GaitController::anyFlying() const {
  for (uint8_t i = 0; i < kNumLegs; i++) {
    if (!leg_state_[i].frozen) return true;
  }
  return false;
}

void GaitController::rebaseWorldFrame() {
  // Re-express every stored foot_world position in a body-at-origin frame
  // so we can zero body_pos_ / body_yaw_ without any physical motion:
  //   foot_body = R(-body_yaw_) · (foot_world - body_pos_)
  // With body at origin and zero yaw, body-frame == world-frame, so we
  // write foot_body back as the new foot_world.
  float cy = cosf(body_yaw_ * kDegToRad);
  float sy = sinf(body_yaw_ * kDegToRad);
  for (uint8_t i = 0; i < kNumLegs; i++) {
    Kinematics::Vec3 f = kin_->getFootWorld(i);
    float dx = f.x - body_pos_.x;
    float dy = f.y - body_pos_.y;
    float dz = f.z - body_pos_.z;
    Kinematics::Vec3 rebased = {cy * dx + sy * dy, -sy * dx + cy * dy, dz};
    kin_->setFootWorld(i, rebased);
  }
  body_pos_ = {};
  body_yaw_ = 0.f;
  // Sync kinematics' internal body_pos_ / body_yaw_ (used by worldToBody on
  // the next IK call) without changing any servo angles — the re-expressed
  // foot_world positions produce identical body-frame positions.
  kin_->setBodyPose(body_pos_, body_yaw_);
}

float GaitController::smoothstep(float s) {
  if (s <= 0.f) return 0.f;
  if (s >= 1.f) return 1.f;
  return s * s * (3.f - 2.f * s);
}

}  // namespace Gait
