/**
 * @file HexapodKinematics.h
 * @date 4/3/2026
 * @brief 6-leg kinematics manager for a hexapod robot.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * Owns six HexapodLeg solvers and tracks foot positions in world frame.
 * Provides the two core operations the gait layer needs:
 *
 *   setBodyPose()   — body moved, feet stayed planted. Transforms the stored
 *                     world-frame foot positions into the new body frame and
 *                     re-solves IK for all six legs.
 *
 *   solveFootWorld() — a foot is mid-swing. Solve IK for its current position
 *                     in world frame without committing it as the planted
 *                     position. Call setFootWorld() when it touches down.
 *
 * This class is pure kinematics — it has no knowledge of swing/stance state,
 * timing, or gait phase. That logic belongs in the gait layer above it.
 *
 * Coordinate frames:
 *   World frame — fixed to the ground, origin arbitrary, Z up.
 *   Body frame  — origin at body centre, X forward, Y left, Z up.
 *                 Moves with the robot.
 *
 * Body rotation: yaw only. Roll/pitch are not modelled here; terrain
 * adaptation via body tilt can be layered on top later.
 */
#pragma once

#include "HexapodLeg.h"

namespace Kinematics {

static constexpr uint8_t kNumLegs = 6;

/// @brief Per-leg IK results returned by setBodyPose() and standNeutral().
struct SolveResult {
  LegAngles legs[kNumLegs];  // solved hip + knee angles (degrees) per leg
  bool valid[kNumLegs];      // false if IK failed for that leg
};

class HexapodKinematics {
 public:
  /// @brief Construct with one LegConfig per leg (index 0–5).
  ///        Initialises foot_world_[] to each leg's neutralPos().
  explicit HexapodKinematics(const LegConfig configs[kNumLegs]);

  // --- Stance-phase operation ---

  /// @brief Update body pose and re-solve IK for all six legs.
  ///
  /// Assumes all feet remain planted at their stored world-frame positions
  /// (foot_world_[]). For each leg, transforms that position into the new
  /// body frame and calls HexapodLeg::solveBody(). This is the primary call
  /// during stance — invoke it every control cycle after integrating velocity.
  ///
  /// @param pos  New body origin in world frame (mm).
  /// @param yaw  New body heading (degrees, CCW from world +X).
  /// @return Solved angles for all six legs. Check valid[] per leg before use.
  SolveResult setBodyPose(Vec3 pos, float yaw);

  // --- Swing-phase operation ---

  /// @brief Solve IK for a foot at an arbitrary world-frame position.
  ///
  /// Does NOT update the stored foot_world_ for this leg. Use this every
  /// control cycle while a foot is mid-swing to track its arc through the air.
  /// When the foot touches down, commit its final position with setFootWorld().
  ///
  /// @param leg        Leg index (0–5).
  /// @param foot_world Current foot position in world frame (mm).
  /// @param out        Solved angles written here on success.
  /// @return true if the position is reachable within servo limits.
  bool solveFootWorld(uint8_t leg, Vec3 foot_world, LegAngles& out) const;

  /// @brief Commit a foot's planted position after touchdown.
  ///
  /// Stores foot_world as the new world-frame position for this leg.
  /// Subsequent setBodyPose() calls will use this value when re-solving IK.
  ///
  /// @param leg        Leg index (0–5).
  /// @param foot_world Final foot position in world frame (mm).
  void setFootWorld(uint8_t leg, Vec3 foot_world);

  // --- Setup ---

  /// @brief Move all legs to their neutral standing positions.
  ///
  /// Resets body pose to the world-frame origin with zero yaw, sets each
  /// foot_world_[] to the corresponding leg's neutralPos(), then returns
  /// the solved angles. Call once on startup before the gait loop begins.
  SolveResult standNeutral();

  // --- Queries ---

  /// @brief Returns the stored world-frame foot position for a leg.
  /// @param leg  Leg index (0–5).
  Vec3 getFootWorld(uint8_t leg) const;

  /// @brief Returns true if foot_world is reachable by the specified leg
  ///        given the current body pose.
  /// @param leg        Leg index (0–5).
  /// @param foot_world Candidate foot position in world frame (mm).
  bool isReachable(uint8_t leg, Vec3 foot_world) const;

  /// @brief Direct access to a leg solver (e.g. for config inspection).
  const HexapodLeg& leg(uint8_t index) const { return legs_[index]; }

 private:
  HexapodLeg legs_[kNumLegs];
  Vec3 foot_world_[kNumLegs];  // planted foot positions in world frame
  Vec3 body_pos_;              // current body origin in world frame (mm)
  float body_yaw_;             // current body heading (degrees)

  /// @brief Transform a world-frame position into the current body frame.
  ///
  /// Subtracts body_pos_ then applies the inverse of the yaw rotation:
  ///   body.x =  cos(yaw) * dx + sin(yaw) * dy
  ///   body.y = -sin(yaw) * dx + cos(yaw) * dy
  ///   body.z = world.z - body_pos_.z
  Vec3 worldToBody(Vec3 foot_world) const;
};

}  // namespace Kinematics
