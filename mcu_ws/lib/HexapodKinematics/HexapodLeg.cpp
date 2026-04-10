/**
 * @file HexapodLeg.cpp
 * @date 4/3/2026
 * @brief HexapodLeg implementation — 2-DOF IK/FK for hip-yaw + knee-pitch leg.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include "HexapodLeg.h"

namespace Kinematics {

static constexpr float kDegToRad = 0.017453292519943f;  // π / 180
static constexpr float kRadToDeg = 57.295779513082f;    // 180 / π

// --- Construction ---

HexapodLeg::HexapodLeg(const LegConfig& cfg)
    : cfg_(cfg),
      mount_cos_(cosf(cfg.mount_angle_deg * kDegToRad)),
      mount_sin_(sinf(cfg.mount_angle_deg * kDegToRad)) {}

// --- Private helpers ---

Vec3 HexapodLeg::toLocal(Vec3 foot) const {
  float dx = foot.x - cfg_.mount_pos.x;
  float dy = foot.y - cfg_.mount_pos.y;
  // Rotate by -mount_angle so the mount direction aligns with local +X.
  return {mount_cos_ * dx + mount_sin_ * dy, -mount_sin_ * dx + mount_cos_ * dy,
          foot.z - cfg_.mount_pos.z};
}

// --- Inverse kinematics ---

bool HexapodLeg::solveLocal(Vec3 foot, LegAngles& out) const {
  // Horizontal reach from hip.
  float r = sqrtf(foot.x * foot.x + foot.y * foot.y);

  // Hip yaw: direction the leg must point to reach (x, y).
  float hip_deg = atan2f(foot.y, foot.x) * kRadToDeg;

  // Knee pitch: couples reach and height.
  //   reach  = L1 + L2·cos(θ_knee)  →  r - L1 = L2·cos(θ_knee)
  //   height = −L2·sin(θ_knee)       →  −z     = L2·sin(θ_knee)
  float knee_deg = atan2f(-foot.z, r - cfg_.L1) * kRadToDeg;

  if (hip_deg < cfg_.hip_min_deg || hip_deg > cfg_.hip_max_deg) return false;
  if (knee_deg < cfg_.knee_min_deg || knee_deg > cfg_.knee_max_deg)
    return false;

  out = {hip_deg, knee_deg};
  return true;
}

bool HexapodLeg::solveBody(Vec3 foot, LegAngles& out) const {
  return solveLocal(toLocal(foot), out);
}

// --- Forward kinematics ---

Vec3 HexapodLeg::forwardBody(LegAngles angles) const {
  float hip_rad = angles.hip * kDegToRad;
  float knee_rad = angles.knee * kDegToRad;

  // Foot position in leg-local frame.
  float r = cfg_.L1 + cfg_.L2 * cosf(knee_rad);
  Vec3 local = {r * cosf(hip_rad), r * sinf(hip_rad),
                -cfg_.L2 * sinf(knee_rad)};

  // Rotate by +mount_angle and translate by mount_pos to get body frame.
  return {cfg_.mount_pos.x + mount_cos_ * local.x - mount_sin_ * local.y,
          cfg_.mount_pos.y + mount_sin_ * local.x + mount_cos_ * local.y,
          cfg_.mount_pos.z + local.z};
}

// --- Queries ---

bool HexapodLeg::isReachable(Vec3 foot) const {
  Vec3 local = toLocal(foot);
  float r = sqrtf(local.x * local.x + local.y * local.y);

  // The foot must lie on the sphere of radius L2 centred at the knee joint.
  // The knee is always at horizontal distance L1 from the hip (femur
  // horizontal).
  float dist = sqrtf((r - cfg_.L1) * (r - cfg_.L1) + local.z * local.z);
  if (fabsf(dist - cfg_.L2) > cfg_.reach_tolerance) return false;

  // Reuse solveLocal for the servo limit check.
  LegAngles tmp;
  return solveLocal(local, tmp);
}

Vec3 HexapodLeg::neutralPos() const {
  // Hip points straight out (0°), knee at the configured standing angle.
  return forwardBody({0.0f, cfg_.neutral_knee_deg});
}

}  // namespace Kinematics
