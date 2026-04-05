/**
 * @file HexapodLeg.h
 * @date 4/3/2026
 * @brief 2-DOF hexapod leg kinematics (hip yaw + knee pitch).
 *        Provides forward and inverse kinematics for a single leg.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * Leg geometry:
 *   Body → hip joint (yaw) → femur (L1, always horizontal) → knee joint (pitch)
 *        → lower leg (L2) → foot
 *
 * Because the hip is yaw-only, the femur is always parallel to the ground.
 * The knee joint is always at the same height as the hip joint. All vertical
 * foot movement comes from knee pitch alone, which couples reach and height:
 *   reach  = L1 + L2·cos(θ_knee)
 *   height = −L2·sin(θ_knee)
 *
 * Coordinate frames:
 *   Body frame — origin at body centre, X forward, Y left, Z up.
 *   Leg-local  — origin at hip joint, X along mount direction, Z up.
 *
 * Usage:
 *   1. Construct with LegConfig (mount geometry, link lengths, servo limits).
 *   2. solveBody(foot_pos, angles) — IK in body frame → LegAngles (degrees).
 *   3. neutralPos() — default standing foot position in body frame.
 *   Caller is responsible for passing LegAngles to the servo layer.
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace Kinematics {

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct LegAngles {
  float hip  = 0.0f;  // degrees
  float knee = 0.0f;  // degrees
};

struct LegConfig {
  Vec3  mount_pos;        // hip joint position in body frame (mm)
  float mount_angle_deg;  // direction leg faces outward (deg, CCW from body +X)
  float L1;               // femur length (mm, hip to knee)
  float L2;               // lower leg length (mm, knee to foot)

  float hip_min_deg;   // hip servo travel limits
  float hip_max_deg;
  float knee_min_deg;  // knee servo travel limits
  float knee_max_deg;

  float neutral_knee_deg;  // knee angle in neutral standing stance
  float reach_tolerance;   // mm — acceptable deviation from L2 in isReachable
};

class HexapodLeg {
 public:
  /// @brief Default constructor — zeroed config, not valid for solving.
  ///        Exists so HexapodKinematics can declare a fixed-size array.
  HexapodLeg() : cfg_{}, mount_cos_(1.0f), mount_sin_(0.0f) {}

  /// @brief Construct a leg from its physical and geometric config.
  ///        Caches cos/sin of the mount angle so solves are cheap.
  explicit HexapodLeg(const LegConfig& cfg);

  // --- Inverse kinematics ---

  /// @brief IK in leg-local frame.
  ///
  /// Leg-local frame has its origin at the hip joint with +X pointing
  /// straight out along the leg's mount direction. Use this when the foot
  /// target is already expressed in that frame. Prefer solveBody for most
  /// callers.
  ///
  /// @param foot  Desired foot position in leg-local frame (mm).
  /// @param out   Solved hip and knee angles (degrees) written here on success.
  /// @return true if the target is reachable within servo travel limits.
  bool solveLocal(Vec3 foot, LegAngles& out) const;

  /// @brief IK in body frame — the primary call site for gait and pose control.
  ///
  /// Transforms the foot target from body frame into leg-local frame via
  /// toLocal(), then delegates to solveLocal(). The body frame has its origin
  /// at the body centre with +X forward, +Y left, +Z up.
  ///
  /// @param foot  Desired foot position in body frame (mm).
  /// @param out   Solved hip and knee angles (degrees) written here on success.
  /// @return true if the target is reachable within servo travel limits.
  bool solveBody(Vec3 foot, LegAngles& out) const;

  // --- Forward kinematics ---

  /// @brief FK — given joint angles, returns the foot position in body frame.
  ///
  /// Computes the foot position in leg-local frame from the joint geometry:
  ///   reach  = L1 + L2·cos(θ_knee)
  ///   height = −L2·sin(θ_knee)
  /// then rotates by +mount_angle and translates by mount_pos to produce a
  /// body-frame result. Inverse of solveBody.
  ///
  /// @param angles  Hip and knee angles in degrees.
  /// @return Foot position in body frame (mm).
  Vec3 forwardBody(LegAngles angles) const;

  // --- Queries ---

  /// @brief Returns true if the foot position (body frame) is achievable.
  ///
  /// Two conditions must both hold:
  ///   1. The foot lies on the reachable surface — its distance from the knee
  ///      joint is within reach_tolerance of L2. Because the hip is yaw-only
  ///      the knee is always horizontal, so the reachable surface is a sphere
  ///      of radius L2 centred at the knee.
  ///   2. The angles required to reach it are within the configured servo
  ///      travel limits.
  ///
  /// @param foot  Candidate foot position in body frame (mm).
  /// @return true if both conditions are satisfied.
  bool isReachable(Vec3 foot) const;

  /// @brief Returns the default standing foot position in body frame.
  ///
  /// Computed as forwardBody({hip=0, knee=neutral_knee_deg}), so the foot
  /// sits directly in front of the hip at the configured standing height.
  /// Changing neutral_knee_deg in LegConfig automatically updates this.
  Vec3 neutralPos() const;

  // --- Accessor ---

  const LegConfig& config() const { return cfg_; }

 private:
  LegConfig cfg_;
  float     mount_cos_;  // cached cosf(mount_angle_rad) — used every solve
  float     mount_sin_;  // cached sinf(mount_angle_rad)

  /// @brief Transforms a foot position from body frame into leg-local frame.
  ///
  /// Step 1 — translate: subtract mount_pos to move the origin from body
  ///   centre to the hip joint.
  /// Step 2 — rotate by −mount_angle: aligns the leg's outward direction with
  ///   local +X so that atan2(y, x) in solveLocal yields 0° when the foot is
  ///   directly in front of the hip.
  ///
  /// @param foot  Foot position in body frame (mm).
  /// @return Foot position in leg-local frame (mm).
  Vec3 toLocal(Vec3 foot) const;
};

}  // namespace Kinematics
