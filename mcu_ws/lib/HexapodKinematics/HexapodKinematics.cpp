/**
 * @file HexapodKinematics.cpp
 * @date 4/3/2026
 * @brief HexapodKinematics implementation.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include "HexapodKinematics.h"

namespace Kinematics {

static constexpr float kDegToRad = 0.017453292519943f;  // π / 180
static constexpr float kRadToDeg = 57.295779513082f;    // 180 / π

// --- Construction ---

HexapodKinematics::HexapodKinematics(const LegConfig configs[kNumLegs])
    : body_pos_({0.0f, 0.0f, 0.0f}), body_yaw_(0.0f) {
  for (uint8_t i = 0; i < kNumLegs; i++) {
    legs_[i] = HexapodLeg(configs[i]);
    neutral_foot_body_[i] = legs_[i].neutralPos();
    foot_world_[i] = neutral_foot_body_[i];
  }
}

// --- Private ---

Vec3 HexapodKinematics::worldToBody(Vec3 foot_world) const {
  float yaw_rad = body_yaw_ * kDegToRad;
  float cos_yaw = cosf(yaw_rad);
  float sin_yaw = sinf(yaw_rad);
  float dx = foot_world.x - body_pos_.x;
  float dy = foot_world.y - body_pos_.y;
  return {cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy,
          foot_world.z - body_pos_.z};
}

// --- Stance-phase operation ---

SolveResult HexapodKinematics::setBodyPose(Vec3 pos, float yaw) {
  body_pos_ = pos;
  body_yaw_ = yaw;

  SolveResult result{};
  for (uint8_t i = 0; i < kNumLegs; i++) {
    Vec3 foot_body = worldToBody(foot_world_[i]);
    result.valid[i] = legs_[i].solveBody(foot_body, result.legs[i]);
  }
  return result;
}

// --- Swing-phase operation ---

bool HexapodKinematics::solveFootWorld(uint8_t leg, Vec3 foot_world,
                                       LegAngles& out) const {
  if (leg >= kNumLegs) return false;
  return legs_[leg].solveBody(worldToBody(foot_world), out);
}

void HexapodKinematics::setFootWorld(uint8_t leg, Vec3 foot_world) {
  if (leg >= kNumLegs) return;
  foot_world_[leg] = foot_world;
}

// --- Setup ---

SolveResult HexapodKinematics::standNeutral() {
  body_pos_ = {0.0f, 0.0f, 0.0f};
  body_yaw_ = 0.0f;
  for (uint8_t i = 0; i < kNumLegs; i++) {
    // neutral_foot_body_ is in body frame; with body at world origin these
    // positions coincide with the world-frame planted positions.
    foot_world_[i] = neutral_foot_body_[i];
  }
  return setBodyPose(body_pos_, body_yaw_);
}

bool HexapodKinematics::setStandHeight(float height_mm) {
  // Per-leg validation pass: compute the required knee angle and confirm it
  // sits inside the servo travel limits. We commit nothing until every leg
  // has been verified — keeps the neutral reference consistent across legs.
  Vec3 candidate[kNumLegs];
  for (uint8_t i = 0; i < kNumLegs; i++) {
    const LegConfig& c = legs_[i].config();
    if (height_mm <= 0.0f || height_mm >= c.L2) return false;
    float knee_deg = asinf(height_mm / c.L2) * kRadToDeg;
    if (knee_deg < c.knee_min_deg || knee_deg > c.knee_max_deg) return false;
    candidate[i] = legs_[i].forwardBody({0.0f, knee_deg});
  }
  for (uint8_t i = 0; i < kNumLegs; i++) neutral_foot_body_[i] = candidate[i];
  return true;
}

// --- Queries ---

Vec3 HexapodKinematics::getFootWorld(uint8_t leg) const {
  if (leg >= kNumLegs) return {};
  return foot_world_[leg];
}

Vec3 HexapodKinematics::neutralFootBody(uint8_t leg) const {
  if (leg >= kNumLegs) return {};
  return neutral_foot_body_[leg];
}

bool HexapodKinematics::isReachable(uint8_t leg, Vec3 foot_world) const {
  if (leg >= kNumLegs) return false;
  return legs_[leg].isReachable(worldToBody(foot_world));
}

}  // namespace Kinematics
