/**
 * @file RobotPersistence.h
 * @brief Shared NVS persistence for the hexapod's servo + gait + body-height
 *        calibration. Extracted so every sketch (main, test_sub_movement,
 *        test_sub_servo) reads and writes the same keys and binary layout.
 *
 * NVS namespace: "srvtest" (kept for backward compatibility with the in-field
 * calibration saved by the existing test_sub_servo / test_sub_movement flow).
 *
 * Typical use:
 * @code
 * Persistence::RobotPrefs defaults = buildDefaults();   // sketch-local
 * Persistence::RobotPrefs prefs = defaults;
 * bool from_nvs = Persistence::loadAll(prefs, defaults);
 * // ...apply prefs to servos / gait / kinematics...
 * Persistence::saveAll(prefs);
 * @endcode
 */
#pragma once

#include <GaitController.h>
#include <Preferences.h>
#include <ServoSubsystem.h>

namespace Persistence {

// NVS keys — single source of truth shared by all sketches.
inline constexpr char kNs[] = "srvtest";
inline constexpr char kCfgKey[] = "cfg";
inline constexpr char kBudgetKey[] = "budget";
inline constexpr char kStepHKey[] = "g_step_h";
inline constexpr char kCycleKey[] = "g_cycle";
inline constexpr char kScaleKey[] = "g_scale";
inline constexpr char kHeightKey[] = "m_height";

/**
 * Shared calibration bundle. Sketches may persist additional fields alongside
 * these using the exposed key constants and a direct Preferences handle.
 */
struct RobotPrefs {
  Subsystem::ServoConfig servos[13];
  float budget;
  Gait::GaitConfig gait;
  float height_mm;
};

/// Populates @p out from NVS, falling back to @p defaults for any missing key.
/// Returns true when a valid servo-config blob was present — i.e. the board
/// has been calibrated at least once. Gait/height keys fall back silently.
bool loadAll(RobotPrefs& out, const RobotPrefs& defaults);

/// Persists the full bundle atomically-ish under the Preferences handle.
bool saveAll(const RobotPrefs& in);

// Focused helpers — useful when a single field changes at runtime and we want
// to avoid rewriting the 13-entry servo blob.
bool saveGait(const Gait::GaitConfig& gc);
bool saveHeight(float height_mm);
bool saveBudget(float budget);
bool saveServos(const Subsystem::ServoConfig (&servos)[13], float budget);

/// Wipe every key in the namespace. Next boot will see defaults.
bool clearAll();

}  // namespace Persistence
