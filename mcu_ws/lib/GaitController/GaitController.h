/**
 * @file GaitController.h
 * @date 4/3/2026
 * @brief Tripod gait controller for a 2-DOF hexapod robot.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * Sits between HexapodKinematics and ServoSubsystem. Accepts velocity commands
 * (linear x/y, angular z) and drives a tripod gait:
 *   Group A legs {0,3,4} (FL, MR, RL) and Group B legs {1,2,5} (FR, ML, RR)
 *   alternate stance ↔ flight, offset by half a cycle.
 *
 * ## Phase model
 * Each leg has a normalized phase_t ∈ [0.0, 1.0) that advances each update()
 * tick by dt / (cycle_time_s / 2). Values below 0.5 are stance; 0.5 and above
 * are flight. The two tripod groups are initialised at 0.0 and 0.5
 * respectively.
 *
 * ## Swing arc
 * During flight, the foot follows a parametric arc: horizontal position moves
 * via a smoothstep curve from lift-off to the step target; vertical position
 * rises and falls as step_height_mm · sin(π·s) above the lift-off height.
 *
 * ## Threading
 * Extends ThreadedSubsystem. Intended to run on Core 1 at 10 ms cadence,
 * at lower priority than ServoSubsystem so servo flushes always win.
 * setVelocity(), enable(), disable(), and stop() are thread-safe.
 *
 * ## Usage
 *   1. Construct GaitSetup with a GaitConfig and pointers to HexapodKinematics
 *      and ServoSubsystem.
 *   2. Call init() — verifies the pointers are non-null.
 *   3. beginThreadedPinned() — begin() runs standNeutral, arms servos, and
 *      staggers leg phases.
 *   4. Call enable() then setVelocity(vx, vy, wz) to start walking.
 *   5. Call disable() for a clean stop (lets in-flight legs land first).
 *   6. Call stop() for an immediate halt (snaps all legs to neutral).
 */
#pragma once

#include <HexapodKinematics.h>
#include <ServoSubsystem.h>
#include <ThreadedSubsystem.h>
#include <hal_thread.h>

#include <cmath>

namespace Gait {

static constexpr uint8_t kNumLegs = 6;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class GaitState : uint8_t { IDLE, WALKING, STOPPING };

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// @brief Fixed physical and tuning parameters. Set once at construction.
struct GaitConfig {
  float step_height_mm;  // z-lift above resting foot height at swing arc peak
  float cycle_time_s;    // duration of one full tripod cycle (both groups)
  float step_scale;      // multiplier on velocity × half-cycle → step reach
  float min_vel_threshold;  // m/s — below this magnitude treat command as zero

  uint8_t leg_servo_hip[kNumLegs];   // ServoSubsystem index for each hip
  uint8_t leg_servo_knee[kNumLegs];  // ServoSubsystem index for each knee
};

/// @brief Constructor arguments for GaitController, following BaseSetup
/// pattern.
struct GaitSetup : public Classes::BaseSetup {
  GaitConfig gait;
  Kinematics::HexapodKinematics* kinematics = nullptr;  // non-owning
  Subsystem::ServoSubsystem* servos = nullptr;          // non-owning

  explicit GaitSetup(const GaitConfig& gc, Kinematics::HexapodKinematics* k,
                     Subsystem::ServoSubsystem* s)
      : Classes::BaseSetup("GaitController"),
        gait(gc),
        kinematics(k),
        servos(s) {}
};

// ---------------------------------------------------------------------------
// Velocity command
// ---------------------------------------------------------------------------

/// @brief Desired body velocity. Written by setVelocity(), read each update().
struct VelocityCommand {
  float vx = 0.f;  // forward velocity (m/s, body frame)
  float vy = 0.f;  // lateral velocity (m/s, body frame, left positive)
  float wz = 0.f;  // yaw rate (rad/s, CCW positive)
};

// ---------------------------------------------------------------------------
// Per-leg gait state
// ---------------------------------------------------------------------------

struct LegGaitState {
  float phase_t = 0.f;      // [0.0, 1.0) — stance below 0.5, flight above
  bool was_flying = false;  // phase on the previous tick ≥ 0.5
  bool frozen = false;      // true during STOPPING for legs already in stance

  Kinematics::Vec3 lift_start;    // world-frame foot position when flight began
  Kinematics::Vec3 swing_target;  // world-frame target for this step
};

// ---------------------------------------------------------------------------
// GaitController
// ---------------------------------------------------------------------------

class GaitController : public Subsystem::ThreadedSubsystem {
 public:
  explicit GaitController(const GaitSetup& setup);

  // --- ThreadedSubsystem overrides ---

  /// @brief Verifies that kinematics and servos pointers are non-null.
  bool init() override;

  /// @brief Moves all legs to neutral, arms servos, and staggers leg phases.
  ///        Called once by the FreeRTOS task before the update loop begins.
  void begin() override;

  /// @brief Main gait state machine. Called every 10 ms by the task loop.
  ///        Integrates body pose, sequences tripod phases, computes swing arcs,
  ///        and pushes angles to ServoSubsystem.
  void update() override;

  /// @brief Delegates to stop().
  void pause() override;

  /// @brief Delegates to stop(), then resets body pose to origin.
  void reset() override;

  const char* getInfo() override { return setup_.getId(); }

  // --- Public API (thread-safe) ---

  /// @brief Set the desired body velocity.
  ///        Takes effect on the next update() cycle.
  void setVelocity(float vx, float vy, float wz);

  /// @brief Begin walking (IDLE → WALKING). No-op if already walking.
  void enable();

  /// @brief Request a clean stop: in-flight legs finish landing, then IDLE.
  ///        Stance legs are frozen immediately; flying legs complete their arc.
  void disable();

  /// @brief Immediate halt: snap all legs to neutral standing position.
  void stop();

  // --- Runtime gait tuning (thread-safe) ---

  /// @brief Set the swing-arc peak height (mm). Takes effect on the next
  ///        flight entry.
  void setStepHeight(float mm);

  /// @brief Set the tripod cycle duration (seconds). Takes effect immediately
  ///        on the next update() tick (phase_inc recomputes each cycle).
  void setCycleTime(float s);

  /// @brief Set the step-reach multiplier. Takes effect on the next flight
  ///        entry (where step targets are computed).
  void setStepScale(float x);

  // --- Queries (thread-safe) ---

  GaitState getState() const;
  VelocityCommand getVelocity() const;

  /// @brief Snapshot of current gait tuning fields (step_height_mm,
  ///        cycle_time_s, step_scale).
  GaitConfig getGaitConfig() const;

  /// @brief Read-only access to per-leg gait state (not mutex-protected).
  ///        Safe to read from the same thread as update().
  const LegGaitState& legState(uint8_t leg) const { return leg_state_[leg]; }

 private:
  GaitSetup setup_;
  Kinematics::HexapodKinematics* kin_ = nullptr;
  Subsystem::ServoSubsystem* servos_ = nullptr;

  GaitState state_ = GaitState::IDLE;
  LegGaitState leg_state_[kNumLegs] = {};
  Kinematics::Vec3 body_pos_ = {};
  float body_yaw_ = 0.f;
  uint32_t last_us_ = 0;

  VelocityCommand cmd_ = {};
  mutable Threads::Mutex state_mutex_;

  // Tripod groups — legs 0,3,4 start at phase 0 (stance first);
  //                 legs 1,2,5 start at phase 0.5 (flight first).
  static constexpr uint8_t kGroupA[3] = {0, 3, 4};  // FL, MR, RL
  static constexpr uint8_t kGroupB[3] = {1, 2, 5};  // FR, ML, RR

  // --- Private helpers ---

  /// @brief Stagger Group A at phase 0 and Group B at phase 0.5.
  void initPhases();

  /// @brief Push a solved LegAngles pair to the servo layer.
  void pushLegAngles(uint8_t leg, const Kinematics::LegAngles& a);

  /// @brief Compute the world-frame step target for a leg beginning its swing.
  ///        Projects the neutral foot position forward by the velocity
  ///        lookahead and clamps to the reachable workspace.
  Kinematics::Vec3 computeStepTarget(uint8_t leg, const VelocityCommand& vel);

  /// @brief Parametric swing arc at normalised parameter s ∈ [0, 1].
  ///        Horizontal: smoothstep interpolation from lift_start to
  ///        swing_target. Vertical: step_height_mm · sin(π·s) above
  ///        lift_start.z.
  Kinematics::Vec3 swingArc(uint8_t leg, float s) const;

  /// @brief True if any leg has not yet been frozen (still in-flight).
  ///        Used during STOPPING to detect when all legs have landed.
  bool anyFlying() const;

  /// @brief Smoothstep: s²·(3 − 2s). Zero velocity at both endpoints.
  static float smoothstep(float s);

  static constexpr float kDegToRad = 0.017453292519943f;
  static constexpr float kRadToDeg = 57.295779513082f;
  static constexpr float kPi = 3.14159265358979f;
};

}  // namespace Gait
