#include "RobotPersistence.h"

namespace Persistence {

bool loadAll(RobotPrefs& out, const RobotPrefs& defaults) {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/true)) {
    out = defaults;
    return false;
  }

  // Servo blob is the "have-calibration" gate. If the byte count doesn't match
  // what we expect (older firmware wrote a different layout, etc.), treat the
  // whole bundle as uncalibrated and fall back to defaults.
  bool have_cfg = p.isKey(kCfgKey);
  if (have_cfg) {
    size_t n = p.getBytes(kCfgKey, out.servos, sizeof(out.servos));
    if (n != sizeof(out.servos)) {
      for (int i = 0; i < 13; i++) out.servos[i] = defaults.servos[i];
      have_cfg = false;
    }
  } else {
    for (int i = 0; i < 13; i++) out.servos[i] = defaults.servos[i];
  }

  out.budget =
      have_cfg ? p.getFloat(kBudgetKey, defaults.budget) : defaults.budget;

  // Gait tuning is per-field so migrating to a new default set is painless.
  out.gait = defaults.gait;
  out.gait.step_height_mm = p.getFloat(kStepHKey, defaults.gait.step_height_mm);
  out.gait.cycle_time_s = p.getFloat(kCycleKey, defaults.gait.cycle_time_s);
  out.gait.step_scale = p.getFloat(kScaleKey, defaults.gait.step_scale);

  out.height_mm = p.getFloat(kHeightKey, defaults.height_mm);

  p.end();
  return have_cfg;
}

bool saveAll(const RobotPrefs& in) {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/false)) return false;
  p.putBytes(kCfgKey, in.servos, sizeof(in.servos));
  p.putFloat(kBudgetKey, in.budget);
  p.putFloat(kStepHKey, in.gait.step_height_mm);
  p.putFloat(kCycleKey, in.gait.cycle_time_s);
  p.putFloat(kScaleKey, in.gait.step_scale);
  p.putFloat(kHeightKey, in.height_mm);
  p.end();
  return true;
}

bool saveGait(const Gait::GaitConfig& gc) {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/false)) return false;
  p.putFloat(kStepHKey, gc.step_height_mm);
  p.putFloat(kCycleKey, gc.cycle_time_s);
  p.putFloat(kScaleKey, gc.step_scale);
  p.end();
  return true;
}

bool saveHeight(float height_mm) {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/false)) return false;
  p.putFloat(kHeightKey, height_mm);
  p.end();
  return true;
}

bool saveBudget(float budget) {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/false)) return false;
  p.putFloat(kBudgetKey, budget);
  p.end();
  return true;
}

bool saveServos(const Subsystem::ServoConfig (&servos)[13], float budget) {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/false)) return false;
  p.putBytes(kCfgKey, servos, sizeof(servos));
  p.putFloat(kBudgetKey, budget);
  p.end();
  return true;
}

bool clearAll() {
  Preferences p;
  if (!p.begin(kNs, /*readOnly=*/false)) return false;
  p.clear();
  p.end();
  return true;
}

}  // namespace Persistence
