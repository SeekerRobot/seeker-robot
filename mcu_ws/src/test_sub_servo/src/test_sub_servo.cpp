/**
 * @file test_sub_servo.cpp
 * @date 3/27/2026
 * @brief Interactive serial test for ServoSubsystem + PCA9685 driver.
 *
 * Commands use M-port numbers (1–13), matching the silkscreen labels on the
 * PCB. "attach 6" always targets the servo plugged into the M6 header.
 *
 * M-ports with hexapod servo assignments load their angle limits, inversion
 * flag, velocity, and acceleration from HexapodConfig. M8 (no hexapod servo)
 * and the PWM range for all ports use the defaults below.
 *
 * All servos start detached. Type 'help' for available commands.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <HexapodConfig.h>
#include <Preferences.h>
#include <ServoSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>

// ---------------------------------------------------------------------------
// Defaults (restored from original test sketch)
// ---------------------------------------------------------------------------
static constexpr uint16_t kDefaultMinPwm = 120;    // ~500 µs @ 50 Hz
static constexpr uint16_t kDefaultMaxPwm = 590;    // ~2500 µs @ 50 Hz
static constexpr float kDefaultVelocity = 720.0f;  // deg/s
static constexpr float kDefaultAccel = 1000.0f;    // deg/s²
static constexpr float kDefaultBudget = 10000.0f;  // total deg/s budget
static constexpr float kDefaultFreqHz = 50.0f;
static constexpr float kDefaultTotalAngle = 180.0f;  // physical travel (°)

// ---------------------------------------------------------------------------
// M-port → kServoConfigs[] reverse lookup
// Index = M-port label (1–13). Value = index into HexapodConfig::kServoConfigs.
// 255 = no hexapod servo assigned to this port.
// ---------------------------------------------------------------------------
static constexpr uint8_t kMPortToHexIdx[14] = {
    255,  // [0]  — no M0
    0,    // [1]  M1  → FL Hip  (kServoConfigs[0])
    4,    // [2]  M2  → ML Hip  (kServoConfigs[4])
    8,    // [3]  M3  → RL Hip  (kServoConfigs[8])
    10,   // [4]  M4  → RR Hip  (kServoConfigs[10])
    6,    // [5]  M5  → MR Hip  (kServoConfigs[6])
    2,    // [6]  M6  → FR Hip  (kServoConfigs[2])
    11,   // [7]  M7  → RR Knee (kServoConfigs[11])
    1,    // [8]  M8  → FL Knee (kServoConfigs[1])
    5,    // [9]  M9  → ML Knee (kServoConfigs[5])
    9,    // [10] M10 → RL Knee (kServoConfigs[9])
    7,    // [11] M11 → MR Knee (kServoConfigs[7])
    3,    // [12] M12 → FR Knee (kServoConfigs[3])
    255,  // [13] M13 — unassigned
};

// ---------------------------------------------------------------------------
// NVS Preferences
// ---------------------------------------------------------------------------
static constexpr char kPrefsNs[] = "srvtest";
static constexpr char kPrefsCfgKey[] = "cfg";
static constexpr char kPrefsBudgetKey[] = "budget";

static Preferences prefs;

// ---------------------------------------------------------------------------
// Objects
// ---------------------------------------------------------------------------
static Subsystem::ServoConfig servo_configs[13];  // indices 0–12 = M1–M13
static Threads::Mutex i2c_mutex;
static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

// Forward-declare — setup needs the pointer
static Subsystem::ServoSubsystem* servos = nullptr;

// ---------------------------------------------------------------------------
// Serial parser
// ---------------------------------------------------------------------------
static constexpr size_t kLineBufSize = 128;
static char line_buf[kLineBufSize];
static uint8_t line_pos = 0;

// Token buffer — max 4 tokens per command
static constexpr uint8_t kMaxTokens = 4;
static char* tokens[kMaxTokens];
static uint8_t num_tokens = 0;

static void printOk(const char* msg) { Serial.printf("OK: %s\r\n", msg); }
static void printErr(const char* msg) { Serial.printf("ERR: %s\r\n", msg); }

/// @brief Tokenize line_buf in-place. Populates tokens[] and num_tokens.
static void tokenize() {
  num_tokens = 0;
  char* p = line_buf;
  while (*p && num_tokens < kMaxTokens) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') break;
    tokens[num_tokens++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = '\0';
  }
}

/// @brief Parse a token as M-port number (1–13). Returns internal index (0–12).
static bool parseIndex(const char* tok, uint8_t& out) {
  char* end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0') {
    printErr("invalid M-port — must be integer 1-13");
    return false;
  }
  if (v < 1 || v > 13) {
    Serial.printf("ERR: M-port %ld out of range [1..13]\r\n", v);
    return false;
  }
  out = static_cast<uint8_t>(v - 1);  // internal index 0–12
  return true;
}

/// @brief Parse a token as float.
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

/// @brief Parse a token as uint16_t.
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

/// @brief Require exactly n tokens (including the command itself).
static bool requireArgs(uint8_t n) {
  if (num_tokens < n) {
    Serial.printf("ERR: expected %u args, got %u. Type 'help'.\r\n", n - 1,
                  num_tokens - 1);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void printHelp() {
  Serial.println(
      "\r\n"
      "===== Servo Test Commands =====\r\n"
      "<M> = M-port number (1-13)\r\n"
      "help                    Show this help\r\n"
      "status                  Show all M-ports\r\n"
      "status <M>              Show M-port M\r\n"
      "attach <M>              Attach servo on M\r\n"
      "attachall               Attach all servos (M1-M12, skips M13)\r\n"
      "detach <M>              Detach servo on M\r\n"
      "angle <M> <deg>         Set target angle\r\n"
      "vel <M> <deg/s>         Set max velocity\r\n"
      "accel <M> <deg/s^2>     Set max acceleration\r\n"
      "invert <M> <0|1>        Set inverted flag\r\n"
      "minangle <M> <deg>      Set min angle limit\r\n"
      "maxangle <M> <deg>      Set max angle limit\r\n"
      "minpwm <M> <val>        Set min PWM (0-4095)\r\n"
      "maxpwm <M> <val>        Set max PWM (0-4095)\r\n"
      "freq <hz>               Set PWM frequency\r\n"
      "budget <deg/s>          Set total rate budget\r\n"
      "neutral                 Move all attached servos to neutral pose\r\n"
      "hips <deg>              Set all attached hips to angle\r\n"
      "knees <deg>             Set all attached knees to angle\r\n"
      "arm                     Enable OE (outputs on)\r\n"
      "disarm                  Disable OE (outputs off)\r\n"
      "save                    Save current config to NVS\r\n"
      "clearprefs              Clear NVS (defaults load on next boot)\r\n"
      "===============================\r\n");
}

static void printServoStatus(uint8_t i) {
  const auto& cfg = servos->getConfig(i);
  uint8_t mport = i + 1;
  Serial.printf(
      "  [M%2u] ch=%2u %s%s | angle: %.1f -> %.1f (vel %.1f) | "
      "limits [%.1f, %.1f] | pwm [%u, %u]%s | vel_max=%.1f accel=%.1f "
      "total=%.0f\r\n",
      mport, cfg.channel, servos->isAttached(i) ? "ATT" : "DET",
      servos->isInitialized(i) ? "" : " UNINIT", servos->getCurrentAngle(i),
      servos->getTargetAngle(i), servos->getVelocity(i), cfg.min_angle,
      cfg.max_angle, cfg.min_pwm, cfg.max_pwm, cfg.inverted ? " INV" : "",
      cfg.max_velocity, cfg.max_accel, cfg.total_angle_deg);
}

static void cmdStatus() {
  Serial.printf("Armed: %s | Budget: %.0f deg/s\r\n",
                servos->isArmed() ? "YES" : "NO", servos->getTotalRateBudget());
  if (num_tokens >= 2) {
    uint8_t idx;
    if (parseIndex(tokens[1], idx)) printServoStatus(idx);
  } else {
    for (uint8_t i = 0; i < 13; i++) {
      printServoStatus(i);
    }
  }
}

static void cmdAttach() {
  if (!requireArgs(2)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  servos->attach(idx);
  Serial.printf("OK: M%u attached (ch%u)\r\n", idx + 1,
                servos->getConfig(idx).channel);
}

static void cmdAttachAll() {
  for (uint8_t m = 1; m <= 12; m++) {
    uint8_t idx = m - 1;
    servos->attach(idx);
    Serial.printf("OK: M%u attached (ch%u)\r\n", m,
                  servos->getConfig(idx).channel);
  }
}

static void cmdDetach() {
  if (!requireArgs(2)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  servos->detach(idx);
  Serial.printf("OK: M%u detached (ch%u)\r\n", idx + 1,
                servos->getConfig(idx).channel);
}

static void cmdAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  servos->setAngle(idx, deg);
  Serial.printf("OK: M%u target = %.2f deg\r\n", idx + 1, deg);
}

static void cmdVel() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float v;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], v)) return;
  if (v <= 0.0f) {
    printErr("velocity must be > 0");
    return;
  }
  servos->setMaxVelocity(idx, v);
  Serial.printf("OK: M%u max_velocity = %.2f deg/s\r\n", idx + 1, v);
}

static void cmdAccel() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float a;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], a)) return;
  if (a <= 0.0f) {
    printErr("acceleration must be > 0");
    return;
  }
  servos->setMaxAccel(idx, a);
  Serial.printf("OK: M%u max_accel = %.2f deg/s^2\r\n", idx + 1, a);
}

static void cmdInvert() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  long v = strtol(tokens[2], nullptr, 10);
  if (v != 0 && v != 1) {
    printErr("invert must be 0 or 1");
    return;
  }
  servos->setInverted(idx, v == 1);
  Serial.printf("OK: M%u inverted = %s\r\n", idx + 1, v ? "true" : "false");
}

static void cmdMinAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  const auto& cfg = servos->getConfig(idx);
  if (deg >= cfg.max_angle) {
    printErr("min_angle must be < max_angle");
    return;
  }
  servos->setAngleLimits(idx, deg, cfg.max_angle);
  Serial.printf("OK: M%u min_angle = %.2f\r\n", idx + 1, deg);
}

static void cmdMaxAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  const auto& cfg = servos->getConfig(idx);
  if (deg <= cfg.min_angle) {
    printErr("max_angle must be > min_angle");
    return;
  }
  servos->setAngleLimits(idx, cfg.min_angle, deg);
  Serial.printf("OK: M%u max_angle = %.2f\r\n", idx + 1, deg);
}

static void cmdMinPwm() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  uint16_t val;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseU16(tokens[2], val)) return;
  const auto& cfg = servos->getConfig(idx);
  if (val >= cfg.max_pwm) {
    printErr("min_pwm must be < max_pwm");
    return;
  }
  servos->setPwmLimits(idx, val, cfg.max_pwm);
  Serial.printf("OK: M%u min_pwm = %u\r\n", idx + 1, val);
}

static void cmdMaxPwm() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  uint16_t val;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseU16(tokens[2], val)) return;
  const auto& cfg = servos->getConfig(idx);
  if (val <= cfg.min_pwm) {
    printErr("max_pwm must be > min_pwm");
    return;
  }
  servos->setPwmLimits(idx, cfg.min_pwm, val);
  Serial.printf("OK: M%u max_pwm = %u\r\n", idx + 1, val);
}

static void cmdFreq() {
  if (!requireArgs(2)) return;
  float hz;
  if (!parseFloat(tokens[1], hz)) return;
  if (hz < 24.0f || hz > 1526.0f) {
    printErr("frequency must be 24..1526 Hz");
    return;
  }
  servos->setPwmFrequency(hz);
  Serial.printf("OK: PWM frequency = %.1f Hz\r\n", hz);
}

static void cmdBudget() {
  if (!requireArgs(2)) return;
  float b;
  if (!parseFloat(tokens[1], b)) return;
  if (b <= 0.0f) {
    printErr("budget must be > 0");
    return;
  }
  servos->setTotalRateBudget(b);
  Serial.printf("OK: total rate budget = %.1f deg/s\r\n", b);
}

static void cmdNeutral() {
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    uint8_t hi = kMPortToHexIdx[m];
    float target;
    if (hi == 255) {
      target = 0.0f;
    } else if (hi % 2 == 0) {
      target = 0.0f;  // hip neutral: femur straight out
    } else {
      target = HexapodConfig::kNeutralKnee;  // knee neutral: standing height
    }
    servos->setAngle(idx, target);
  }
  Serial.printf("OK: all servos -> neutral (hips=0 deg, knees=%.1f deg)\r\n",
                HexapodConfig::kNeutralKnee);
}

static void cmdHips() {
  if (!requireArgs(2)) return;
  float deg;
  if (!parseFloat(tokens[1], deg)) return;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi == 255) continue;
    if (hi % 2 != 0) continue;  // odd indices are knees
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    servos->setAngle(idx, deg);
  }
  Serial.printf("OK: all hips -> %.2f deg\r\n", deg);
}

static void cmdKnees() {
  if (!requireArgs(2)) return;
  float deg;
  if (!parseFloat(tokens[1], deg)) return;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi == 255) continue;
    if (hi % 2 == 0) continue;  // even indices are hips
    uint8_t idx = m - 1;
    if (!servos->isAttached(idx)) continue;
    servos->setAngle(idx, deg);
  }
  Serial.printf("OK: all knees -> %.2f deg\r\n", deg);
}

static void cmdArm() {
  if (servos->arm()) {
    printOk("armed — OE enabled");
  } else {
    printErr("arm failed — ensure all attached servos have setAngle() called");
  }
}

static void cmdDisarm() {
  servos->disarm();
  printOk("disarmed — OE disabled");
}

static void cmdSave() {
  Subsystem::ServoConfig saved[13];
  for (uint8_t i = 0; i < 13; i++) saved[i] = servos->getConfig(i);

  prefs.begin(kPrefsNs, /*readOnly=*/false);
  prefs.putBytes(kPrefsCfgKey, saved, sizeof(saved));
  prefs.putFloat(kPrefsBudgetKey, servos->getTotalRateBudget());
  prefs.end();
  printOk("config saved to NVS — will load on next boot");
}

static void cmdClearPrefs() {
  prefs.begin(kPrefsNs, /*readOnly=*/false);
  prefs.clear();
  prefs.end();
  printOk("NVS cleared — defaults will load on next boot");
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------
static void processCommand() {
  tokenize();
  if (num_tokens == 0) return;

  const char* cmd = tokens[0];

  // Convert command to lowercase for case-insensitive matching
  for (char* p = tokens[0]; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p += 32;
  }

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    printHelp();
  else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "s") == 0)
    cmdStatus();
  else if (strcmp(cmd, "attach") == 0)
    cmdAttach();
  else if (strcmp(cmd, "attachall") == 0)
    cmdAttachAll();
  else if (strcmp(cmd, "detach") == 0)
    cmdDetach();
  else if (strcmp(cmd, "angle") == 0)
    cmdAngle();
  else if (strcmp(cmd, "vel") == 0)
    cmdVel();
  else if (strcmp(cmd, "accel") == 0)
    cmdAccel();
  else if (strcmp(cmd, "invert") == 0)
    cmdInvert();
  else if (strcmp(cmd, "minangle") == 0)
    cmdMinAngle();
  else if (strcmp(cmd, "maxangle") == 0)
    cmdMaxAngle();
  else if (strcmp(cmd, "minpwm") == 0)
    cmdMinPwm();
  else if (strcmp(cmd, "maxpwm") == 0)
    cmdMaxPwm();
  else if (strcmp(cmd, "freq") == 0)
    cmdFreq();
  else if (strcmp(cmd, "budget") == 0)
    cmdBudget();
  else if (strcmp(cmd, "neutral") == 0)
    cmdNeutral();
  else if (strcmp(cmd, "hips") == 0)
    cmdHips();
  else if (strcmp(cmd, "knees") == 0)
    cmdKnees();
  else if (strcmp(cmd, "arm") == 0)
    cmdArm();
  else if (strcmp(cmd, "disarm") == 0)
    cmdDisarm();
  else if (strcmp(cmd, "save") == 0)
    cmdSave();
  else if (strcmp(cmd, "clearprefs") == 0)
    cmdClearPrefs();
  else
    Serial.printf("ERR: unknown command '%s'. Type 'help'.\r\n", cmd);
}

/// @brief Read serial bytes, dispatch complete lines.
static void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_pos > 0) {
        line_buf[line_pos] = '\0';
        Serial.printf("> %s\r\n", line_buf);  // echo
        processCommand();
        line_pos = 0;
      }
    } else if (line_pos < kLineBufSize - 1) {
      line_buf[line_pos++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Config helpers — called from setup()
// ---------------------------------------------------------------------------

/// Populate servo_configs[] from HexapodConfig defaults and set budget_out.
static void buildDefaults(float& budget_out) {
  budget_out = kDefaultBudget;
  // Build per-M-port servo configs.
  // Hexapod ports copy angle limits, inversion, vel/accel from
  // HexapodConfig::kServoConfigs. PWM is derived from the physical range
  // (kDefaultMinPwm–kDefaultMaxPwm spans total_angle_deg).
  //
  // Hips (min_angle < 0): symmetric about 0° → kinematic 0° = servo center.
  //   pwm = center ± angle × scale
  //
  // Knees (min_angle >= 0): one-sided → kinematic 0° = servo minimum.
  //   pwm = kDefaultMinPwm + angle × scale
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
        servo_configs[m - 1].min_pwm = (uint16_t)roundf(
            kDefaultMinPwm + servo_configs[m - 1].min_angle * scale);
        servo_configs[m - 1].max_pwm = (uint16_t)roundf(
            kDefaultMinPwm + servo_configs[m - 1].max_angle * scale);
      }
    } else {
      // M13: no hexapod servo — generic full-range default
      servo_configs[m - 1] = {
          .channel = Config::mPort(m),
          .min_angle = 0.0f,
          .max_angle = kDefaultTotalAngle,
          .min_pwm = kDefaultMinPwm,
          .max_pwm = kDefaultMaxPwm,
          .inverted = false,
          .max_velocity = kDefaultVelocity,
          .max_accel = kDefaultAccel,
          .total_angle_deg = kDefaultTotalAngle,
      };
    }
  }
}

/// Load servo_configs[] and budget from NVS. Returns true if a valid saved
/// config was found; servo_configs[] is overwritten only on success.
static bool loadFromPrefs(float& budget_out) {
  prefs.begin(kPrefsNs, /*readOnly=*/true);
  bool found = prefs.isKey(kPrefsCfgKey);
  if (found) {
    size_t n =
        prefs.getBytes(kPrefsCfgKey, servo_configs, sizeof(servo_configs));
    if (n != sizeof(servo_configs)) found = false;  // corrupt blob — ignore
  }
  if (found) {
    budget_out = prefs.getFloat(kPrefsBudgetKey, kDefaultBudget);
  }
  prefs.end();
  return found;
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(1000);  // let serial settle

  float budget = kDefaultBudget;
  buildDefaults(budget);  // always build formula-derived defaults
  bool from_prefs =
      loadFromPrefs(budget);  // overrides if a valid NVS blob exists

  // Heartbeat blink
  blink.beginThreadedPinned(2048, 1, 500, 1);

  // Servo subsystem
  static Subsystem::ServoSetup servo_setup(Wire, Config::pca_addr,
                                           Config::servo_en, servo_configs, 13,
                                           budget, kDefaultFreqHz);
  auto& srv = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  if (!srv.init()) {
    Debug::printf(Debug::Level::ERROR, "[MAIN] Servo init failed");
    Serial.println("ERR: Servo subsystem init failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  servos = &srv;
  srv.beginThreadedPinned(4096, 3, 20, 1);  // 50 Hz update

  Serial.println("\r\n=== Servo Subsystem Test ===");
  Serial.println("M-port assignments (from HexapodConfig):");
  Serial.println(
      "  M1=FL_Hip  M2=ML_Hip  M3=RL_Hip  M4=RR_Hip  M5=MR_Hip  M6=FR_Hip");
  Serial.println(
      "  M7=RR_Knee M8=FL_Knee M9=ML_Knee M10=RL_Knee M11=MR_Knee M12=FR_Knee");
  Serial.println("  M13=<unassigned>");
  Serial.printf("Config: %s\r\n", from_prefs ? "LOADED FROM NVS" : "defaults");
  Serial.println("All servos DETACHED. OE DISABLED.");
  Serial.println("Typical workflow: attach 6 -> angle 6 0 -> arm");
  printHelp();
}

void loop() { processSerial(); }
