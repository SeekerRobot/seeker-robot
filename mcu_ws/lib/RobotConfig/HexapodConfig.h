/**
 * @file HexapodConfig.h
 * @date 4/3/2026
 * @brief Single source of truth for all Sesame-based hexapod configuration.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * Edit this file whenever the physical robot changes. Every servo calibration
 * value, link length, mount geometry, and gait tuning parameter lives here.
 * Test sketches and the main application include this file and use the
 * exported constants directly — no inline magic numbers in sketch files.
 *
 * ============================================================================
 * CALIBRATION PROCEDURE
 * ============================================================================
 *
 * ## Kinematic angle conventions
 *
 *   Hip (yaw joint):
 *     0° = femur pointing STRAIGHT OUT along the leg's mount direction.
 *          (For ML/MR this is straight sideways; for FL/FR/RL/RR it is at the
 *          configured mount angle from body +X.)
 *     Positive = femur sweeps toward the robot's FRONT (CCW from above for all
 *                legs, regardless of which side of the body they are on).
 *     Negative = femur sweeps toward the robot's REAR.
 *
 *   Knee (pitch joint):
 *     0°  = lower leg HORIZONTAL — foot at the same height as the hip joint.
 *           Reach is maximum (L1 + L2). Not a useful walking pose; a reference.
 *     +45° = typical comfortable standing height. Good starting point for
 *            neutral_knee_deg.
 *     +90° = lower leg pointing STRAIGHT DOWN — foot directly below the knee.
 *
 * ## Measuring link lengths
 *   L1: distance from the center of the HIP JOINT SHAFT to the center of the
 *       KNEE JOINT SHAFT (along the femur), in mm.
 *   L2: distance from the center of the KNEE JOINT SHAFT to the FOOT TIP, in
 * mm.
 *
 * ## Measuring mount geometry
 *   mount_pos: XY coordinates of the hip joint center relative to the body
 *              center, measured with calipers (mm). Z = 0 for a flat body.
 *   mount_angle_deg: the direction the leg faces outward, measured CCW from the
 *                    body +X axis (forward). Typical values:
 *                      FL = +60°, FR = -60°
 *                      ML = +90°, MR = -90°
 *                      RL = +120°, RR = -120°
 *
 * ## Calibrating a servo (min_pwm / max_pwm)
 *   Use test_sub_servo to send angle commands and observe the physical result.
 *
 *   Hip calibration:
 *     1. Command hip to 0°. Physically verify the femur points straight out
 *        along the mount direction. If not, adjust the servo horn position or
 *        update min_pwm/max_pwm so that the mid-point (0°) lands correctly.
 *     2. Command hip to +30° (forward sweep). Verify the femur has moved toward
 *        the robot front. If it moved BACKWARD instead, set inverted = true.
 *     3. Command hip to kHipMin and kHipMax. Verify the femur does not bind.
 *        Tighten limits if needed.
 *
 *   Knee calibration:
 *     1. Command knee to 0°. Verify the lower leg is approximately horizontal.
 *     2. Command knee to +90°. Verify the lower leg hangs straight down.
 *     3. If increasing angle moves the leg UP instead of DOWN, set inverted =
 * true.
 *     4. Set neutral_knee_deg so the robot stands at a comfortable height with
 *        all feet flat on the ground.
 *
 * ## PWM values (12-bit, 50 Hz PCA9685)
 *   Pulse width → 12-bit value: val = pulse_us / 20000 * 4096
 *     500 µs  → ~102    (servo minimum, some brands)
 *     1000 µs → ~205
 *     1500 µs → ~307    (physical center for most servos)
 *     2000 µs → ~410
 *     2500 µs → ~512    (servo maximum, some brands)
 *   Standard MG90S / SG90 range: typically 500–2500 µs (102–512).
 *   All values below are PLACEHOLDERS. Measure per servo.
 *
 * ============================================================================
 * SERVO PORT ASSIGNMENTS
 * ============================================================================
 *
 * Servos are identified by the silkscreen label on the PCB (M1–M13).
 * The mapping from M port → PCA9685 channel is fixed in RobotConfig.h.
 * Use Config::mPort(N) to look up the channel for a given M port label.
 *
 * Edit the kServo* constants in the M PORT ASSIGNMENTS section below whenever
 * you move a servo cable to a different port. The GaitConfig servo index
 * arrays are derived from those constants automatically.
 */
#pragma once

#include <GaitController.h>
#include <HexapodKinematics.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>

namespace HexapodConfig {

// ============================================================================
// Physical dimensions — MEASURE FROM ROBOT
// ============================================================================

/// Femur length: center of hip joint shaft → center of knee joint shaft (mm).
constexpr float kL1 = 45.0f;  // MEASURE

/// Lower leg length: center of knee joint shaft → foot tip (mm).
constexpr float kL2 = 50.0f;  // MEASURE

/// Hip joint X-offset from body center to front/rear leg mounts (mm).
constexpr float kBodyHalfLength = 60.0f;  // MEASURE

/// Hip joint Y-offset from body center to left/right leg mounts (mm).
constexpr float kBodyHalfWidth = 40.0f;  // MEASURE

// ============================================================================
// Kinematic angle limits — adjust after physical verification
// ============================================================================

/// Hip forward/rear sweep range (kinematic degrees, symmetric about 0°).
constexpr float kHipMin = -60.0f;
constexpr float kHipMax = 60.0f;

/// Knee range: 0° = horizontal (reference), 90° = straight down.
/// Do not allow negative values — the leg cannot reach above hip height
/// usefully.
constexpr float kKneeMin = 0.0f;
constexpr float kKneeMax = 90.0f;

/// Knee angle in the neutral standing stance (degrees).
/// Increase to stand lower; decrease to stand higher. Tune until body is level.
///
/// CONVENTION NOTE: firmware uses 0° = tibia horizontal, 90° = tibia straight
/// down (maximum depth). This is the OPPOSITE of the URDF/simulation convention
/// (0° = vertical, 90° = horizontal). The simulation's NEUTRAL_KNEE = 30°
/// (URDF convention) maps to approximately 60° here if the link geometry were
/// identical — but the firmware omits the 40 mm coxa stub, so the correct
/// value must be calibrated on the physical robot.
///
/// Maximum safe step_height_mm = L2 * sin(kNeutralKnee). At 45°: ~56.6 mm.
constexpr float kNeutralKnee = 45.0f;  // TUNE — see convention note above

/// Workspace tolerance: how close to L2 the foot must be to be considered
/// reachable (mm). Loosen if IK fails near workspace boundary.
constexpr float kReachTolerance = 5.0f;

// ============================================================================
// Servo motion profile — tune for your servos
// ============================================================================

/// Maximum angular velocity per servo (deg/s).
constexpr float kHipMaxVel = 2000.0f;
constexpr float kKneeMaxVel = 2000.0f;

/// Maximum angular acceleration per servo (deg/s²).
constexpr float kHipMaxAccel = 3000.0f;
constexpr float kKneeMaxAccel = 3000.0f;

/// Total angular velocity budget across ALL 12 servos (deg/s).
/// ServoSubsystem scales all velocities proportionally when this is exceeded,
/// preventing current spikes from moving many servos simultaneously.
constexpr float kServoBudget = 10000.0f;

/// PCA9685 PWM frequency (Hz). 50 Hz is standard for analogue servos.
constexpr float kPwmFreqHz = 50.0f;

// ============================================================================
// M PORT ASSIGNMENTS
// Change these when you move a servo cable to a different port.
// Use Config::mPort(N) — N is the number printed on the PCB silkscreen.
// ============================================================================

constexpr uint8_t kServoFL_Hip = Config::mPort(1);    // Leg 0 FL hip
constexpr uint8_t kServoFL_Knee = Config::mPort(8);   // Leg 0 FL knee
constexpr uint8_t kServoFR_Hip = Config::mPort(6);    // Leg 1 FR hip
constexpr uint8_t kServoFR_Knee = Config::mPort(12);   // Leg 1 FR knee
constexpr uint8_t kServoML_Hip = Config::mPort(2);    // Leg 2 ML hip
constexpr uint8_t kServoML_Knee = Config::mPort(9);   // Leg 2 ML knee
constexpr uint8_t kServoMR_Hip = Config::mPort(5);    // Leg 3 MR hip
constexpr uint8_t kServoMR_Knee = Config::mPort(11);  // Leg 3 MR knee
constexpr uint8_t kServoRL_Hip = Config::mPort(3);   // Leg 4 RL hip
constexpr uint8_t kServoRL_Knee = Config::mPort(10);  // Leg 4 RL knee
constexpr uint8_t kServoRR_Hip = Config::mPort(4);   // Leg 5 RR hip
constexpr uint8_t kServoRR_Knee = Config::mPort(7);   // Leg 5 RR knee

// ============================================================================
// Servo configurations — ordered leg 0..5, hip then knee.
// The kServoConfigs array is indexed 0..11 and passed to ServoSubsystem.
// GaitConfig leg_servo_hip/knee use these INDICES, not channel numbers.
//
// min_pwm / max_pwm: 12-bit values at min_angle / max_angle (kinematic °).
// Re-calibrate per the procedure in the file header if servos are swapped.
//
// total_angle_deg: the servo's full physical travel (180 for standard MG90S /
//   SG90). min_pwm/max_pwm map to the kinematic limits, not the physical
//   endpoints; this field documents the servo's hardware capability and is
//   used by test tooling to validate PWM ranges.
//
// inverted: set true if the servo moves in the wrong physical direction.
//           Typically one side of the body needs inverted hips relative to
//           the other. Verify with test_sub_servo before marking as done.
// ============================================================================

// clang-format off
const Subsystem::ServoConfig kServoConfigs[12] = {

  // ── Index 0: Leg 0 FL hip ─────────────────────────────────────────────────
  // 0° = femur pointing 60° CCW from body +X. Positive = sweeps toward front.
  {
    .channel         = kServoFL_Hip,
    .min_angle       = kHipMin,
    .max_angle       = kHipMax,
    .min_pwm         = 198,    // PWM at kHipMin (most-rearward)
    .max_pwm         = 512,    // PWM at kHipMax (most-forward)
    .inverted        = true,
    .max_velocity    = kHipMaxVel,
    .max_accel       = kHipMaxAccel,
    .total_angle_deg = 180.0f,
  },
  // ── Index 1: Leg 0 FL knee ────────────────────────────────────────────────
  // 0° = lower leg horizontal. Positive = foot drops below hip.
  {
    .channel         = kServoFL_Knee,
    .min_angle       = kKneeMin,
    .max_angle       = kKneeMax,
    .min_pwm         = 110,    // PWM at 0° (horizontal reference)
    .max_pwm         = 355,    // PWM at 90° (straight down)
    .inverted        = true,
    .max_velocity    = kKneeMaxVel,
    .max_accel       = kKneeMaxAccel,
    .total_angle_deg = 180.0f,
  },

  // ── Index 2: Leg 1 FR hip ─────────────────────────────────────────────────
  {
    .channel         = kServoFR_Hip,
    .min_angle       = kHipMin,
    .max_angle       = kHipMax,
    .min_pwm         = 198,
    .max_pwm         = 512,
    .inverted        = false,
    .max_velocity    = kHipMaxVel,
    .max_accel       = kHipMaxAccel,
    .total_angle_deg = 180.0f,
  },
  // ── Index 3: Leg 1 FR knee ────────────────────────────────────────────────
  {
    .channel         = kServoFR_Knee,
    .min_angle       = kKneeMin,
    .max_angle       = kKneeMax,
    .min_pwm         = 355,
    .max_pwm         = 605,
    .inverted        = false,
    .max_velocity    = kKneeMaxVel,
    .max_accel       = kKneeMaxAccel,
    .total_angle_deg = 180.0f,
  },

  // ── Index 4: Leg 2 ML hip ─────────────────────────────────────────────────
  // Asymmetric forward sweep: mechanical bind past +30° limits this servo's
  // usable range to [-60°, +30°]. PWM endpoints scaled to match the new max.
  {
    .channel         = kServoML_Hip,
    .min_angle       = kHipMin,
    .max_angle       = 30.0f,
    .min_pwm         = 198,
    .max_pwm         = 433,
    .inverted        = true,
    .max_velocity    = kHipMaxVel,
    .max_accel       = kHipMaxAccel,
    .total_angle_deg = 180.0f,
  },
  // ── Index 5: Leg 2 ML knee ────────────────────────────────────────────────
  {
    .channel         = kServoML_Knee,
    .min_angle       = kKneeMin,
    .max_angle       = kKneeMax,
    .min_pwm         = 355,
    .max_pwm         = 580,
    .inverted        = false,
    .max_velocity    = kKneeMaxVel,
    .max_accel       = kKneeMaxAccel,
    .total_angle_deg = 180.0f,
  },

  // ── Index 6: Leg 3 MR hip ─────────────────────────────────────────────────
  // Asymmetric forward sweep: mechanical bind past +30° limits this servo's
  // usable range to [-60°, +30°]. PWM endpoints scaled to match the new max.
  {
    .channel         = kServoMR_Hip,
    .min_angle       = kHipMin,
    .max_angle       = 30.0f,
    .min_pwm         = 198,
    .max_pwm         = 433,
    .inverted        = false,
    .max_velocity    = kHipMaxVel,
    .max_accel       = kHipMaxAccel,
    .total_angle_deg = 180.0f,
  },
  // ── Index 7: Leg 3 MR knee ────────────────────────────────────────────────
  {
    .channel         = kServoMR_Knee,
    .min_angle       = kKneeMin,
    .max_angle       = kKneeMax,
    .min_pwm         = 110,
    .max_pwm         = 355,
    .inverted        = true,
    .max_velocity    = kKneeMaxVel,
    .max_accel       = kKneeMaxAccel,
    .total_angle_deg = 180.0f,
  },

  // ── Index 8: Leg 4 RL hip ─────────────────────────────────────────────────
  {
    .channel         = kServoRL_Hip,
    .min_angle       = kHipMin,
    .max_angle       = kHipMax,
    .min_pwm         = 198,
    .max_pwm         = 512,
    .inverted        = true,
    .max_velocity    = kHipMaxVel,
    .max_accel       = kHipMaxAccel,
    .total_angle_deg = 180.0f,
  },
  // ── Index 9: Leg 4 RL knee ────────────────────────────────────────────────
  {
    .channel         = kServoRL_Knee,
    .min_angle       = kKneeMin,
    .max_angle       = kKneeMax,
    .min_pwm         = 355,
    .max_pwm         = 580,
    .inverted        = false,
    .max_velocity    = kKneeMaxVel,
    .max_accel       = kKneeMaxAccel,
    .total_angle_deg = 180.0f,
  },

  // ── Index 10: Leg 5 RR hip ────────────────────────────────────────────────
  {
    .channel         = kServoRR_Hip,
    .min_angle       = kHipMin,
    .max_angle       = kHipMax,
    .min_pwm         = 198,
    .max_pwm         = 512,
    .inverted        = false,
    .max_velocity    = kHipMaxVel,
    .max_accel       = kHipMaxAccel,
    .total_angle_deg = 180.0f,
  },
  // ── Index 11: Leg 5 RR knee ───────────────────────────────────────────────
  {
    .channel         = kServoRR_Knee,
    .min_angle       = kKneeMin,
    .max_angle       = kKneeMax,
    .min_pwm         = 120,
    .max_pwm         = 355,
    .inverted        = true,
    .max_velocity    = kKneeMaxVel,
    .max_accel       = kKneeMaxAccel,
    .total_angle_deg = 180.0f,
  },
};
// clang-format on

// ============================================================================
// Leg kinematics — one entry per leg (index matches leg numbering).
// ============================================================================

// clang-format off
const Kinematics::LegConfig kLegConfigs[Gait::kNumLegs] = {
  // Leg 0 — FL (front-left)
  {
    .mount_pos        = { kBodyHalfLength,  kBodyHalfWidth, 0.0f },
    .mount_angle_deg  =  60.0f,
    .L1               = kL1,
    .L2               = kL2,
    .hip_min_deg      = kHipMin,
    .hip_max_deg      = kHipMax,
    .knee_min_deg     = kKneeMin,
    .knee_max_deg     = kKneeMax,
    .neutral_knee_deg = kNeutralKnee,
    .reach_tolerance  = kReachTolerance,
  },
  // Leg 1 — FR (front-right)
  {
    .mount_pos        = { kBodyHalfLength, -kBodyHalfWidth, 0.0f },
    .mount_angle_deg  = -60.0f,
    .L1               = kL1,
    .L2               = kL2,
    .hip_min_deg      = kHipMin,
    .hip_max_deg      = kHipMax,
    .knee_min_deg     = kKneeMin,
    .knee_max_deg     = kKneeMax,
    .neutral_knee_deg = kNeutralKnee,
    .reach_tolerance  = kReachTolerance,
  },
  // Leg 2 — ML (mid-left). Hip sweep is asymmetric: forward bind at +30°.
  {
    .mount_pos        = { 0.0f,  kBodyHalfWidth, 0.0f },
    .mount_angle_deg  =  90.0f,
    .L1               = kL1,
    .L2               = kL2,
    .hip_min_deg      = kHipMin,
    .hip_max_deg      = 30.0f,
    .knee_min_deg     = kKneeMin,
    .knee_max_deg     = kKneeMax,
    .neutral_knee_deg = kNeutralKnee,
    .reach_tolerance  = kReachTolerance,
  },
  // Leg 3 — MR (mid-right). Hip sweep is asymmetric: forward bind at +30°.
  {
    .mount_pos        = { 0.0f, -kBodyHalfWidth, 0.0f },
    .mount_angle_deg  = -90.0f,
    .L1               = kL1,
    .L2               = kL2,
    .hip_min_deg      = kHipMin,
    .hip_max_deg      = 30.0f,
    .knee_min_deg     = kKneeMin,
    .knee_max_deg     = kKneeMax,
    .neutral_knee_deg = kNeutralKnee,
    .reach_tolerance  = kReachTolerance,
  },
  // Leg 4 — RL (rear-left)
  {
    .mount_pos        = {-kBodyHalfLength,  kBodyHalfWidth, 0.0f },
    .mount_angle_deg  =  120.0f,
    .L1               = kL1,
    .L2               = kL2,
    .hip_min_deg      = kHipMin,
    .hip_max_deg      = kHipMax,
    .knee_min_deg     = kKneeMin,
    .knee_max_deg     = kKneeMax,
    .neutral_knee_deg = kNeutralKnee,
    .reach_tolerance  = kReachTolerance,
  },
  // Leg 5 — RR (rear-right)
  {
    .mount_pos        = {-kBodyHalfLength, -kBodyHalfWidth, 0.0f },
    .mount_angle_deg  = -120.0f,
    .L1               = kL1,
    .L2               = kL2,
    .hip_min_deg      = kHipMin,
    .hip_max_deg      = kHipMax,
    .knee_min_deg     = kKneeMin,
    .knee_max_deg     = kKneeMax,
    .neutral_knee_deg = kNeutralKnee,
    .reach_tolerance  = kReachTolerance,
  },
};
// clang-format on

// ============================================================================
// Gait tuning
// ============================================================================

const Gait::GaitConfig kGaitConfig = {
    /// Height the foot lifts above its resting position at the peak of the
    /// swing arc (mm). Increase for uneven ground; decrease for speed.
    /// Must be < L2 * sin(kNeutralKnee) to stay within knee joint limits.
    /// At kNeutralKnee=45°: max ~56.6 mm. Matches simulation's ~66 mm clearance
    /// as closely as the firmware kinematics allow (no coxa modelled).
    .step_height_mm = 50.0f,

    /// Duration of one full tripod cycle — both groups complete stance + swing
    /// in this time (seconds). Lower = faster walk; too low causes missed
    /// steps.
    /// Matches fake_mcu_node CYCLE_TIME = 0.8 s.
    .cycle_time_s = 0.8f,

    /// Multiplier on (velocity × half-cycle-time) to compute step reach (mm).
    /// 1.0 = step exactly as far as the body travels per half-cycle.
    /// Reduce below 1.0 if legs consistently overshoot the workspace boundary.
    .step_scale = 1.0f,

    /// Velocity commands below this magnitude (m/s) are treated as zero and
    /// trigger a clean stop rather than zero-velocity walking.
    .min_vel_threshold = 0.005f,

    // Servo INDICES into kServoConfigs[] for each leg's hip and knee.
    // These match the channel order defined in kServoConfigs above.
    .leg_servo_hip = {0, 2, 4, 6, 8, 10},
    .leg_servo_knee = {1, 3, 5, 7, 9, 11},
};

/// Total number of servos used (one entry per channel in kServoConfigs).
constexpr uint8_t kNumServos = 12;

}  // namespace HexapodConfig
