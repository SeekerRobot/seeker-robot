/**
 * @file test_sub_movement.cpp
 * @date 4/16/2026
 * @brief Serial + BLE console for hexapod movement.
 *
 * Non-micro-ROS REPL that drives the full locomotion stack (ServoSubsystem →
 * HexapodKinematics → GaitController) over two transports simultaneously:
 * USB Serial and BLE Nordic UART. A command from either transport produces a
 * response on BOTH — intentional: both share one printAll() sink.
 *
 * Reuses the NVS namespace "srvtest" (same as test_sub_servo) so servo
 * calibration carries over. Gait parameters and body height are persisted
 * in the same namespace under extra keys (ignored by test_sub_servo's load).
 *
 * Typical workflow:
 *   > walk 0.05 0 0          (forward at 50 mm/s)
 *   > stop
 *   > height 50              (body 50 mm above ground, only in IDLE)
 *   > save                   (persist all tunings)
 *
 * Type 'help' for the full command list.
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BleDebugSubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <GaitController.h>
#include <HexapodConfig.h>
#include <HexapodKinematics.h>
#include <Preferences.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>

#include <cstdarg>

// ---------------------------------------------------------------------------
// Defaults (mirror test_sub_servo)
// ---------------------------------------------------------------------------
static constexpr uint16_t kDefaultMinPwm     = 120;
static constexpr uint16_t kDefaultMaxPwm     = 590;
static constexpr float    kDefaultVelocity   = 720.0f;
static constexpr float    kDefaultAccel      = 1000.0f;
static constexpr float    kDefaultBudget     = 10000.0f;
static constexpr float    kDefaultFreqHz     = 50.0f;
static constexpr float    kDefaultTotalAngle = 180.0f;

// Battery calibration — lifted verbatim from test_all/main.cpp.
static constexpr Subsystem::BatteryCalibration kBattCalibration(
    /*raw_lo=*/2432, /*volt_lo=*/11.52f,
    /*raw_hi=*/2648, /*volt_hi=*/12.62f);

// ---------------------------------------------------------------------------
// M-port ↔ kServoConfigs index mapping (lifted from test_sub_servo)
// ---------------------------------------------------------------------------
static constexpr uint8_t kMPortToHexIdx[14] = {
    255,  //  [0]  — unused
    0,    //  M1  → FL Hip
    4,    //  M2  → ML Hip
    8,    //  M3  → RL Hip
    10,   //  M4  → RR Hip
    6,    //  M5  → MR Hip
    2,    //  M6  → FR Hip
    11,   //  M7  → RR Knee
    1,    //  M8  → FL Knee
    5,    //  M9  → ML Knee
    9,    //  M10 → RL Knee
    7,    //  M11 → MR Knee
    3,    //  M12 → FR Knee
    255,  //  M13 — unassigned
};

// Leg index → ServoSubsystem index (in our 13-entry M-port layout).
// Needed because GaitConfig::leg_servo_hip/knee normally reference the
// 12-entry HexapodConfig::kServoConfigs layout, but ServoSubsystem here is
// populated in M-port order (M1..M13 → index 0..12). Values derived from
// kMPortToHexIdx by inversion.
static constexpr uint8_t kLegServoHip[6]  = {0, 5, 1, 4, 2, 3};   // M1, M6, M2, M5, M3, M4
static constexpr uint8_t kLegServoKnee[6] = {7, 11, 8, 10, 9, 6}; // M8, M12, M9, M11, M10, M7

// ---------------------------------------------------------------------------
// NVS Preferences
// ---------------------------------------------------------------------------
static constexpr char kPrefsNs[]         = "srvtest";
static constexpr char kPrefsCfgKey[]     = "cfg";
static constexpr char kPrefsBudgetKey[]  = "budget";
static constexpr char kPrefsStepHKey[]   = "g_step_h";
static constexpr char kPrefsCycleKey[]   = "g_cycle";
static constexpr char kPrefsScaleKey[]   = "g_scale";
static constexpr char kPrefsHeightKey[]  = "m_height";

static Preferences prefs;

// ---------------------------------------------------------------------------
// Subsystems
// ---------------------------------------------------------------------------
static Subsystem::ServoConfig servo_configs[13];
static Threads::Mutex i2c_mutex;

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::BleDebugSetup ble_setup("SeekerMovement");

static Subsystem::BatterySetup battery_setup(Config::batt, kBattCalibration,
                                             /*num_samples=*/16);

static Subsystem::ServoSubsystem*     servos     = nullptr;
static Kinematics::HexapodKinematics* kinematics = nullptr;
static Gait::GaitController*          gait       = nullptr;
static Subsystem::BatterySubsystem*   battery    = nullptr;

// Current desired body height (mm). Applied whenever gait is IDLE.
static float body_height_mm = 0.0f;

// ---------------------------------------------------------------------------
// Line buffers — one per source so interleaved bytes cannot corrupt commands.
// ---------------------------------------------------------------------------
static constexpr size_t kLineBufSize = 128;
static char     serial_line_buf[kLineBufSize];
static uint8_t  serial_line_pos = 0;
static char     ble_line_buf[kLineBufSize];

// Token buffer
static constexpr uint8_t kMaxTokens = 6;
static char*    tokens[kMaxTokens];
static uint8_t  num_tokens = 0;

// ---------------------------------------------------------------------------
// Dual-transport printing
// ---------------------------------------------------------------------------
static void printAll(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  Serial.print(buf);
  Subsystem::BleDebugSubsystem::writeIfReady(buf);
}

static void printOk(const char* msg)  { printAll("OK: %s\r\n", msg); }
static void printErr(const char* msg) { printAll("ERR: %s\r\n", msg); }

// ---------------------------------------------------------------------------
// Tokenizer / parsers (mirror test_sub_servo)
// ---------------------------------------------------------------------------
static void tokenize(char* line) {
  num_tokens = 0;
  char* p = line;
  while (*p && num_tokens < kMaxTokens) {
    while (*p == ' ' || *p == '\t') p++;
    if (!*p) break;
    tokens[num_tokens++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = '\0';
  }
}

static bool requireArgs(uint8_t n) {
  if (num_tokens < n) {
    printAll("ERR: expected %u arg(s), got %u. Type 'help'.\r\n",
             n - 1, num_tokens - 1);
    return false;
  }
  return true;
}

static bool parseIndex(const char* tok, uint8_t& out) {
  char* end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0') {
    printErr("invalid M-port — must be integer 1-13");
    return false;
  }
  if (v < 1 || v > 13) {
    printAll("ERR: M-port %ld out of range [1..13]\r\n", v);
    return false;
  }
  out = static_cast<uint8_t>(v - 1);
  return true;
}

static bool parseFloat(const char* tok, float& out) {
  char* end;
  float v = strtof(tok, &end);
  if (end == tok || *end != '\0') {
    printErr("invalid number");
    return false;
  }
  out = v;
  return true;
}

static bool parseU16(const char* tok, uint16_t& out) {
  char* end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0' || v < 0 || v > 4095) {
    printErr("invalid PWM value — must be 0..4095");
    return false;
  }
  out = static_cast<uint16_t>(v);
  return true;
}

// ---------------------------------------------------------------------------
// Help
// ---------------------------------------------------------------------------
// printAll has a 256-byte stack buffer, so long help text would be truncated.
// Split into chunks, each well under 256 bytes.
static void printHelp() {
  printAll("\r\n===== Movement Console =====\r\n"
           "-- servo (carry-over from test_sub_servo) --\r\n"
           "help | ?                  Show this help\r\n"
           "status [M]                Show all M-ports or one\r\n"
           "attach <M>                Attach servo on M\r\n"
           "attachall                 Attach M1-M12 (skip M13)\r\n"
           "detach <M>                Detach servo on M\r\n");
  printAll("angle <M> <deg>           Set target angle\r\n"
           "vel <M> <deg/s>           Set max velocity\r\n"
           "accel <M> <deg/s^2>       Set max acceleration\r\n"
           "invert <M> <0|1>          Set inverted flag\r\n"
           "minangle <M> <deg>        Set min angle limit\r\n"
           "maxangle <M> <deg>        Set max angle limit\r\n");
  printAll("minpwm <M> <val>          Set min PWM (0-4095)\r\n"
           "maxpwm <M> <val>          Set max PWM (0-4095)\r\n"
           "freq <hz>                 Set PWM frequency\r\n"
           "budget <deg/s>            Set total rate budget\r\n"
           "neutral                   Move attached servos to neutral pose\r\n");
  printAll("hips <deg>                Set all attached hips\r\n"
           "knees <deg>               Set all attached knees\r\n"
           "flat                      Lay flat (hips=0, knees=0)\r\n"
           "standing                  Stand (kinematics neutral pose)\r\n"
           "arm | disarm              OE on / off\r\n");
  printAll("-- movement --\r\n"
           "height <mm>               Set body height (IDLE only)\r\n"
           "walk                      Start tripod gait\r\n"
           "idle                      Clean stop (legs finish landing)\r\n"
           "stop                      Immediate snap to neutral\r\n"
           "forward <m/s>             Walk forward\r\n"
           "back <m/s>                Walk backward\r\n");
  printAll("strafe <m/s>              Sideways (positive = left)\r\n"
           "turn <deg/s>              Yaw rate\r\n"
           "move <vx> <vy> <wz>       Full velocity (m/s, m/s, rad/s)\r\n"
           "gait_step <mm>            Set swing-arc height\r\n"
           "gait_cycle <s>            Set tripod cycle time\r\n"
           "gait_scale <x>            Set step-reach multiplier\r\n"
           "gait_status               Print gait state + tuning\r\n");
  printAll("-- persistence --\r\n"
           "save                      Save all tunings to NVS\r\n"
           "clearprefs                Clear NVS (defaults next boot)\r\n"
           "============================\r\n");
}

// ---------------------------------------------------------------------------
// Servo status (mirror test_sub_servo)
// ---------------------------------------------------------------------------
static void printServoStatus(uint8_t i) {
  const auto& cfg = servos->getConfig(i);
  uint8_t mport = i + 1;
  printAll(
      "  [M%2u] ch=%2u %s%s | angle: %.1f -> %.1f (vel %.1f) | "
      "limits [%.1f, %.1f] | pwm [%u, %u]%s | vel_max=%.1f accel=%.1f "
      "total=%.0f\r\n",
      mport, cfg.channel, servos->isAttached(i) ? "ATT" : "DET",
      servos->isInitialized(i) ? "" : " UNINIT", servos->getCurrentAngle(i),
      servos->getTargetAngle(i), servos->getVelocity(i), cfg.min_angle,
      cfg.max_angle, cfg.min_pwm, cfg.max_pwm, cfg.inverted ? " INV" : "",
      cfg.max_velocity, cfg.max_accel, cfg.total_angle_deg);
}

static const char* stateName(Gait::GaitState s);

static void cmdStatus() {
  printAll("Armed: %s | Budget: %.0f deg/s\r\n",
           servos->isArmed() ? "YES" : "NO", servos->getTotalRateBudget());
  if (battery) {
    printAll("Battery: %.2f V (raw %u)\r\n",
             battery->getVoltage(), battery->getRawAdc());
  }
  Gait::VelocityCommand vel = gait->getVelocity();
  printAll("Movement: %s | vx=%+.3f vy=%+.3f wz=%+.3f | height=%.1f mm\r\n",
           stateName(gait->getState()), vel.vx, vel.vy, vel.wz, body_height_mm);
  if (num_tokens >= 2) {
    uint8_t idx;
    if (parseIndex(tokens[1], idx)) printServoStatus(idx);
  } else {
    for (uint8_t i = 0; i < 13; i++) printServoStatus(i);
  }
}

static void cmdAttach() {
  if (!requireArgs(2)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  servos->attach(idx);
  printAll("OK: M%u attached (ch%u)\r\n", idx + 1, servos->getConfig(idx).channel);
}

static void cmdAttachAll() {
  for (uint8_t m = 1; m <= 12; m++) {
    uint8_t idx = m - 1;
    servos->attach(idx);
    printAll("OK: M%u attached (ch%u)\r\n", m, servos->getConfig(idx).channel);
  }
}

static void cmdDetach() {
  if (!requireArgs(2)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  servos->detach(idx);
  printAll("OK: M%u detached (ch%u)\r\n", idx + 1, servos->getConfig(idx).channel);
}

static void cmdAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  servos->setAngle(idx, deg);
  printAll("OK: M%u target = %.2f deg\r\n", idx + 1, deg);
}

static void cmdVel() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float v;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], v)) return;
  if (v <= 0.0f) { printErr("velocity must be > 0"); return; }
  servos->setMaxVelocity(idx, v);
  printAll("OK: M%u max_velocity = %.2f deg/s\r\n", idx + 1, v);
}

static void cmdAccel() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float a;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], a)) return;
  if (a <= 0.0f) { printErr("acceleration must be > 0"); return; }
  servos->setMaxAccel(idx, a);
  printAll("OK: M%u max_accel = %.2f deg/s^2\r\n", idx + 1, a);
}

static void cmdInvert() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  long v = strtol(tokens[2], nullptr, 10);
  if (v != 0 && v != 1) { printErr("invert must be 0 or 1"); return; }
  servos->setInverted(idx, v == 1);
  printAll("OK: M%u inverted = %s\r\n", idx + 1, v ? "true" : "false");
}

static void cmdMinAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  const auto& cfg = servos->getConfig(idx);
  if (deg >= cfg.max_angle) { printErr("min_angle must be < max_angle"); return; }
  servos->setAngleLimits(idx, deg, cfg.max_angle);
  printAll("OK: M%u min_angle = %.2f\r\n", idx + 1, deg);
}

static void cmdMaxAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  const auto& cfg = servos->getConfig(idx);
  if (deg <= cfg.min_angle) { printErr("max_angle must be > min_angle"); return; }
  servos->setAngleLimits(idx, cfg.min_angle, deg);
  printAll("OK: M%u max_angle = %.2f\r\n", idx + 1, deg);
}

static void cmdMinPwm() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  uint16_t val;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseU16(tokens[2], val)) return;
  const auto& cfg = servos->getConfig(idx);
  if (val >= cfg.max_pwm) { printErr("min_pwm must be < max_pwm"); return; }
  servos->setPwmLimits(idx, val, cfg.max_pwm);
  printAll("OK: M%u min_pwm = %u\r\n", idx + 1, val);
}

static void cmdMaxPwm() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  uint16_t val;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseU16(tokens[2], val)) return;
  const auto& cfg = servos->getConfig(idx);
  if (val <= cfg.min_pwm) { printErr("max_pwm must be > min_pwm"); return; }
  servos->setPwmLimits(idx, cfg.min_pwm, val);
  printAll("OK: M%u max_pwm = %u\r\n", idx + 1, val);
}

static void cmdFreq() {
  if (!requireArgs(2)) return;
  float hz;
  if (!parseFloat(tokens[1], hz)) return;
  if (hz < 24.0f || hz > 1526.0f) { printErr("frequency must be 24..1526 Hz"); return; }
  servos->setPwmFrequency(hz);
  printAll("OK: PWM frequency = %.1f Hz\r\n", hz);
}

static void cmdBudget() {
  if (!requireArgs(2)) return;
  float b;
  if (!parseFloat(tokens[1], b)) return;
  if (b <= 0.0f) { printErr("budget must be > 0"); return; }
  servos->setTotalRateBudget(b);
  printAll("OK: total rate budget = %.1f deg/s\r\n", b);
}

static void cmdNeutral() {
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    uint8_t hi = kMPortToHexIdx[m];
    float target;
    if (hi == 255)            target = 0.0f;
    else if (hi % 2 == 0)     target = 0.0f;
    else                      target = HexapodConfig::kNeutralKnee;
    servos->setAngle(idx, target);
  }
  printAll("OK: neutral (hips=0, knees=%.1f)\r\n", HexapodConfig::kNeutralKnee);
}

static void cmdHips() {
  if (!requireArgs(2)) return;
  float deg;
  if (!parseFloat(tokens[1], deg)) return;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi == 255 || hi % 2 != 0) continue;
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    servos->setAngle(idx, deg);
  }
  printAll("OK: all hips -> %.2f deg\r\n", deg);
}

static void cmdKnees() {
  if (!requireArgs(2)) return;
  float deg;
  if (!parseFloat(tokens[1], deg)) return;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi == 255 || hi % 2 == 0) continue;
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    servos->setAngle(idx, deg);
  }
  printAll("OK: all knees -> %.2f deg\r\n", deg);
}

static void cmdFlat() {
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    servos->setAngle(idx, 0.0f);
  }
  printAll("OK: flat (all attached -> 0.0 deg)\r\n");
}

static void cmdStanding() {
  if (gait->getState() != Gait::GaitState::IDLE) {
    printErr("gait is walking — issue 'stop' first");
    return;
  }
  Kinematics::SolveResult r = kinematics->standNeutral();
  uint8_t bad = 0;
  for (uint8_t i = 0; i < Gait::kNumLegs; i++) {
    if (!r.valid[i]) { bad++; continue; }
    servos->setAngle(kLegServoHip[i],  r.legs[i].hip);
    servos->setAngle(kLegServoKnee[i], r.legs[i].knee);
  }
  if (bad) printAll("WARN: %u legs unreachable at neutral\r\n", bad);
  printAll("OK: standing (kinematics neutral pose)\r\n");
}

static void cmdArm() {
  if (servos->arm()) printOk("armed — OE enabled");
  else               printErr("arm failed — ensure attached servos have had setAngle() called");
}

static void cmdDisarm() {
  servos->disarm();
  printOk("disarmed — OE disabled");
}

// ---------------------------------------------------------------------------
// Movement
// ---------------------------------------------------------------------------
static bool requireIdle() {
  if (gait->getState() != Gait::GaitState::IDLE) {
    printErr("gait must be IDLE — issue 'stop' first");
    return false;
  }
  return true;
}

// Updates each leg's neutral foot position for the requested stand height,
// then snaps all legs to that pose. The new height also becomes the anchor
// used by the gait layer for step targets, so `walk` keeps the set height.
static void cmdHeight() {
  if (!requireArgs(2)) return;
  float h;
  if (!parseFloat(tokens[1], h)) return;
  if (!requireIdle()) return;
  if (!kinematics->setStandHeight(h)) {
    printAll("ERR: height %.1f mm unreachable for at least one leg\r\n", h);
    return;
  }
  Kinematics::SolveResult r = kinematics->standNeutral();
  uint8_t bad = 0;
  for (uint8_t i = 0; i < Gait::kNumLegs; i++) {
    if (!r.valid[i]) { bad++; continue; }
    servos->setAngle(kLegServoHip[i],  r.legs[i].hip);
    servos->setAngle(kLegServoKnee[i], r.legs[i].knee);
  }
  if (bad) {
    printAll("WARN: %u legs unreachable at height %.1f mm — pose partially applied\r\n",
             bad, h);
  }
  body_height_mm = h;
  printAll("OK: height = %.1f mm\r\n", h);
}

static void cmdWalk() {
  gait->enable();
  printOk("walking (use 'forward'/'turn'/'move' to set velocity)");
}

static void cmdStop() {
  gait->stop();
  printOk("stopped — snapped to neutral");
}

static void cmdIdle() {
  gait->disable();
  printOk("disable requested — STOPPING (legs will finish landing)");
}

static void cmdForward() {
  if (!requireArgs(2)) return;
  float v;
  if (!parseFloat(tokens[1], v)) return;
  gait->setVelocity(v, 0.0f, 0.0f);
  printAll("OK: vx=%+.3f m/s\r\n", v);
}

static void cmdBack() {
  if (!requireArgs(2)) return;
  float v;
  if (!parseFloat(tokens[1], v)) return;
  gait->setVelocity(-v, 0.0f, 0.0f);
  printAll("OK: vx=%+.3f m/s (back)\r\n", -v);
}

static void cmdStrafe() {
  if (!requireArgs(2)) return;
  float v;
  if (!parseFloat(tokens[1], v)) return;
  gait->setVelocity(0.0f, v, 0.0f);
  printAll("OK: vy=%+.3f m/s\r\n", v);
}

static void cmdTurn() {
  if (!requireArgs(2)) return;
  float deg_s;
  if (!parseFloat(tokens[1], deg_s)) return;
  float wz = deg_s * 0.017453292519943f;
  gait->setVelocity(0.0f, 0.0f, wz);
  printAll("OK: wz=%+.3f rad/s (%.1f deg/s)\r\n", wz, deg_s);
}

static void cmdMove() {
  if (!requireArgs(4)) return;
  float vx, vy, wz;
  if (!parseFloat(tokens[1], vx)) return;
  if (!parseFloat(tokens[2], vy)) return;
  if (!parseFloat(tokens[3], wz)) return;
  gait->setVelocity(vx, vy, wz);
  printAll("OK: vx=%+.3f vy=%+.3f wz=%+.3f\r\n", vx, vy, wz);
}

static void cmdGaitStep() {
  if (!requireArgs(2)) return;
  float mm;
  if (!parseFloat(tokens[1], mm)) return;
  if (mm <= 0.0f) { printErr("step height must be > 0"); return; }
  gait->setStepHeight(mm);
  printAll("OK: gait step_height = %.2f mm\r\n", mm);
}

static void cmdGaitCycle() {
  if (!requireArgs(2)) return;
  float s;
  if (!parseFloat(tokens[1], s)) return;
  if (s <= 0.05f) { printErr("cycle time must be > 0.05 s"); return; }
  gait->setCycleTime(s);
  printAll("OK: gait cycle_time = %.3f s\r\n", s);
}

static void cmdGaitScale() {
  if (!requireArgs(2)) return;
  float x;
  if (!parseFloat(tokens[1], x)) return;
  if (x <= 0.0f) { printErr("scale must be > 0"); return; }
  gait->setStepScale(x);
  printAll("OK: gait step_scale = %.3f\r\n", x);
}

static const char* stateName(Gait::GaitState s) {
  switch (s) {
    case Gait::GaitState::IDLE:     return "IDLE";
    case Gait::GaitState::WALKING:  return "WALKING";
    case Gait::GaitState::STOPPING: return "STOPPING";
    default:                        return "UNKNOWN";
  }
}

static void cmdGaitStatus() {
  Gait::GaitState s = gait->getState();
  Gait::VelocityCommand vel = gait->getVelocity();
  Gait::GaitConfig gc = gait->getGaitConfig();
  printAll("State: %s\r\n", stateName(s));
  printAll("Cmd:   vx=%+.3f  vy=%+.3f  wz=%+.3f\r\n", vel.vx, vel.vy, vel.wz);
  printAll("Tune:  step_height=%.2f mm  cycle=%.3f s  scale=%.3f\r\n",
           gc.step_height_mm, gc.cycle_time_s, gc.step_scale);
  printAll("Body:  height=%.1f mm\r\n", body_height_mm);
}

// ---------------------------------------------------------------------------
// Persistence — namespace "srvtest" shared with test_sub_servo
// ---------------------------------------------------------------------------
static void cmdSave() {
  Subsystem::ServoConfig saved[13];
  for (uint8_t i = 0; i < 13; i++) saved[i] = servos->getConfig(i);

  Gait::GaitConfig gc = gait->getGaitConfig();

  prefs.begin(kPrefsNs, /*readOnly=*/false);
  prefs.putBytes(kPrefsCfgKey, saved, sizeof(saved));
  prefs.putFloat(kPrefsBudgetKey, servos->getTotalRateBudget());
  prefs.putFloat(kPrefsStepHKey,  gc.step_height_mm);
  prefs.putFloat(kPrefsCycleKey,  gc.cycle_time_s);
  prefs.putFloat(kPrefsScaleKey,  gc.step_scale);
  prefs.putFloat(kPrefsHeightKey, body_height_mm);
  prefs.end();
  printOk("config + gait + height saved to NVS");
}

static void cmdClearPrefs() {
  prefs.begin(kPrefsNs, /*readOnly=*/false);
  prefs.clear();
  prefs.end();
  printOk("NVS cleared — defaults will load on next boot");
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------
static void dispatch(char* line) {
  tokenize(line);
  if (num_tokens == 0) return;

  // Lowercase the verb for case-insensitive matching
  for (char* p = tokens[0]; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p += 32;
  }
  const char* cmd = tokens[0];

  if      (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)          printHelp();
  else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "s") == 0)        cmdStatus();
  else if (strcmp(cmd, "attach") == 0)                                 cmdAttach();
  else if (strcmp(cmd, "attachall") == 0)                              cmdAttachAll();
  else if (strcmp(cmd, "detach") == 0)                                 cmdDetach();
  else if (strcmp(cmd, "angle") == 0)                                  cmdAngle();
  else if (strcmp(cmd, "vel") == 0)                                    cmdVel();
  else if (strcmp(cmd, "accel") == 0)                                  cmdAccel();
  else if (strcmp(cmd, "invert") == 0)                                 cmdInvert();
  else if (strcmp(cmd, "minangle") == 0)                               cmdMinAngle();
  else if (strcmp(cmd, "maxangle") == 0)                               cmdMaxAngle();
  else if (strcmp(cmd, "minpwm") == 0)                                 cmdMinPwm();
  else if (strcmp(cmd, "maxpwm") == 0)                                 cmdMaxPwm();
  else if (strcmp(cmd, "freq") == 0)                                   cmdFreq();
  else if (strcmp(cmd, "budget") == 0)                                 cmdBudget();
  else if (strcmp(cmd, "neutral") == 0)                                cmdNeutral();
  else if (strcmp(cmd, "hips") == 0)                                   cmdHips();
  else if (strcmp(cmd, "knees") == 0)                                  cmdKnees();
  else if (strcmp(cmd, "flat") == 0)                                   cmdFlat();
  else if (strcmp(cmd, "standing") == 0 || strcmp(cmd, "stand") == 0)  cmdStanding();
  else if (strcmp(cmd, "arm") == 0)                                    cmdArm();
  else if (strcmp(cmd, "disarm") == 0)                                 cmdDisarm();
  else if (strcmp(cmd, "height") == 0)                                 cmdHeight();
  else if (strcmp(cmd, "walk") == 0)                                   cmdWalk();
  else if (strcmp(cmd, "stop") == 0)                                   cmdStop();
  else if (strcmp(cmd, "idle") == 0)                                   cmdIdle();
  else if (strcmp(cmd, "forward") == 0 || strcmp(cmd, "fwd") == 0)     cmdForward();
  else if (strcmp(cmd, "back") == 0   || strcmp(cmd, "bwd") == 0)      cmdBack();
  else if (strcmp(cmd, "strafe") == 0)                                 cmdStrafe();
  else if (strcmp(cmd, "turn") == 0)                                   cmdTurn();
  else if (strcmp(cmd, "move") == 0)                                   cmdMove();
  else if (strcmp(cmd, "gait_step") == 0)                              cmdGaitStep();
  else if (strcmp(cmd, "gait_cycle") == 0)                             cmdGaitCycle();
  else if (strcmp(cmd, "gait_scale") == 0)                             cmdGaitScale();
  else if (strcmp(cmd, "gait_status") == 0)                            cmdGaitStatus();
  else if (strcmp(cmd, "save") == 0)                                   cmdSave();
  else if (strcmp(cmd, "clearprefs") == 0)                             cmdClearPrefs();
  else                                                                 printAll("ERR: unknown command '%s'. Type 'help'.\r\n", cmd);
}

// ---------------------------------------------------------------------------
// Input drivers — drain Serial byte-by-byte, poll BLE for assembled lines
// ---------------------------------------------------------------------------
static void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_line_pos > 0) {
        serial_line_buf[serial_line_pos] = '\0';
        printAll("> %s\r\n", serial_line_buf);
        dispatch(serial_line_buf);
        serial_line_pos = 0;
      }
    } else if (serial_line_pos < kLineBufSize - 1) {
      serial_line_buf[serial_line_pos++] = c;
    }
  }
}

static void processBle() {
  if (Subsystem::BleDebugSubsystem::tryGetLine(ble_line_buf, kLineBufSize)) {
    printAll("> %s\r\n", ble_line_buf);
    dispatch(ble_line_buf);
  }
}

// ---------------------------------------------------------------------------
// Config helpers
// ---------------------------------------------------------------------------
static void buildDefaults(float& budget_out, Gait::GaitConfig& gc_out,
                          float& height_out) {
  budget_out = kDefaultBudget;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi != 255) {
      servo_configs[m - 1] = HexapodConfig::kServoConfigs[hi];
      const float total = servo_configs[m - 1].total_angle_deg;
      const float scale = (kDefaultMaxPwm - kDefaultMinPwm) / total;
      if (servo_configs[m - 1].min_angle < 0.0f) {
        const float center = (kDefaultMinPwm + kDefaultMaxPwm) / 2.0f;
        servo_configs[m - 1].min_pwm =
            (uint16_t)roundf(center + servo_configs[m - 1].min_angle * scale);
        servo_configs[m - 1].max_pwm =
            (uint16_t)roundf(center + servo_configs[m - 1].max_angle * scale);
      } else {
        servo_configs[m - 1].min_pwm =
            (uint16_t)roundf(kDefaultMinPwm + servo_configs[m - 1].min_angle * scale);
        servo_configs[m - 1].max_pwm =
            (uint16_t)roundf(kDefaultMinPwm + servo_configs[m - 1].max_angle * scale);
      }
    } else {
      servo_configs[m - 1] = {
          .channel         = Config::mPort(m),
          .min_angle       = 0.0f,
          .max_angle       = kDefaultTotalAngle,
          .min_pwm         = kDefaultMinPwm,
          .max_pwm         = kDefaultMaxPwm,
          .inverted        = false,
          .max_velocity    = kDefaultVelocity,
          .max_accel       = kDefaultAccel,
          .total_angle_deg = kDefaultTotalAngle,
      };
    }
  }

  gc_out = HexapodConfig::kGaitConfig;
  // Override leg_servo_*[] to index into the 13-entry M-port layout.
  for (uint8_t i = 0; i < 6; i++) {
    gc_out.leg_servo_hip[i]  = kLegServoHip[i];
    gc_out.leg_servo_knee[i] = kLegServoKnee[i];
  }

  height_out = 0.0f;
}

/// Returns true if the servo config blob was present (valid calibration).
/// All gait/height keys are optional and fall back silently to defaults.
static bool loadFromPrefs(float& budget_out, Gait::GaitConfig& gc_out,
                          float& height_out) {
  prefs.begin(kPrefsNs, /*readOnly=*/true);

  bool have_cfg = prefs.isKey(kPrefsCfgKey);
  if (have_cfg) {
    size_t n = prefs.getBytes(kPrefsCfgKey, servo_configs, sizeof(servo_configs));
    if (n != sizeof(servo_configs)) have_cfg = false;
  }
  if (have_cfg) {
    budget_out = prefs.getFloat(kPrefsBudgetKey, kDefaultBudget);
  }

  gc_out.step_height_mm = prefs.getFloat(kPrefsStepHKey, gc_out.step_height_mm);
  gc_out.cycle_time_s   = prefs.getFloat(kPrefsCycleKey, gc_out.cycle_time_s);
  gc_out.step_scale     = prefs.getFloat(kPrefsScaleKey, gc_out.step_scale);
  height_out            = prefs.getFloat(kPrefsHeightKey, 0.0f);

  prefs.end();
  return have_cfg;
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(1000);

  // Build defaults, then overlay any NVS-saved values.
  float budget = kDefaultBudget;
  Gait::GaitConfig gc;
  buildDefaults(budget, gc, body_height_mm);
  bool cfg_from_prefs = loadFromPrefs(budget, gc, body_height_mm);

  // Heartbeat
  blink.beginThreadedPinned(2048, 1, 500, 1);

  // BLE console transport — core 0 alongside NimBLE host.
  auto& ble = Subsystem::BleDebugSubsystem::getInstance(ble_setup);
  bool ble_ok = ble.init();
  if (!ble_ok) {
    Serial.println("[Main] BleDebugSubsystem init failed — Serial-only mode");
  } else {
    ble.beginThreadedPinned(8192, 2, 50, 0);
  }

  // Battery — ADC sampler on Config::batt, 50 ms cadence, core 1.
  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  if (!batt.init()) {
    Serial.println("[Main] Battery init failed — status will omit voltage");
  } else {
    batt.beginThreadedPinned(4096, 2, 50, 1);
    battery = &batt;
  }

  // Servo subsystem
  static Subsystem::ServoSetup servo_setup(
      Wire, Config::pca_addr, Config::servo_en,
      servo_configs, 13, budget, kDefaultFreqHz);
  auto& srv = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  // Pre-attach all 12 hexapod servos so gait standNeutral/arm can succeed.
  for (uint8_t m = 1; m <= 12; m++) srv.attach(m - 1);
  if (!srv.init()) {
    Serial.println("ERR: Servo subsystem init failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  servos = &srv;
  srv.beginThreadedPinned(4096, 5, 10, 1);

  // Kinematics
  static Kinematics::HexapodKinematics kin(HexapodConfig::kLegConfigs);
  kinematics = &kin;

  // Gait
  static Gait::GaitSetup gait_setup(gc, kinematics, &srv);
  static Gait::GaitController gait_ctrl(gait_setup);
  gait = &gait_ctrl;
  if (!gait->init()) {
    Serial.println("ERR: Gait init failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  gait->beginThreadedPinned(8192, 4, 10, 1);

  // Restore a saved body height after gait begin() has had time to run
  // standNeutral() and arm. setStandHeight re-targets each leg's neutral;
  // standNeutral then solves IK and we push the resulting angles.
  if (body_height_mm > 0.01f && body_height_mm < HexapodConfig::kL2) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (kinematics->setStandHeight(body_height_mm)) {
      Kinematics::SolveResult r = kinematics->standNeutral();
      for (uint8_t i = 0; i < Gait::kNumLegs; i++) {
        if (!r.valid[i]) continue;
        servos->setAngle(kLegServoHip[i],  r.legs[i].hip);
        servos->setAngle(kLegServoKnee[i], r.legs[i].knee);
      }
    }
  }

  // Banner — goes to both transports (BLE writes buffer until a client
  // connects, so this is useful later too).
  printAll("\r\n=== Movement Console ===\r\n");
  printAll("Transports: Serial @921600 | BLE \"%s\" %s\r\n",
           ble_setup.deviceName, ble_ok ? "(advertising)" : "(OFFLINE)");
  printAll("Config: %s\r\n", cfg_from_prefs ? "LOADED FROM NVS (srvtest)" : "defaults");
  printAll("Gait: step=%.1f mm  cycle=%.2f s  scale=%.2f\r\n",
           gc.step_height_mm, gc.cycle_time_s, gc.step_scale);
  printAll("Body height: %.1f mm\r\n", body_height_mm);
  printHelp();
}

void loop() {
  processSerial();
  processBle();
  delay(2);
}
