/**
 * @file test_sub_servo.cpp
 * @date 3/27/2026
 * @brief Interactive serial test for ServoSubsystem + PCA9685 driver.
 * All servos start detached. Type 'help' for available commands.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
static constexpr uint8_t kNumServos = 16;
static constexpr float kDefaultMinAngle = 0.0f;
static constexpr float kDefaultMaxAngle = 180.0f;
static constexpr uint16_t kDefaultMinPwm = 120;    // ~500 us @ 50 Hz
static constexpr uint16_t kDefaultMaxPwm = 590;    // ~2500 us @ 50 Hz
static constexpr float kDefaultVelocity = 720.0f;  // deg/s
static constexpr float kDefaultAccel = 1000.0f;    // deg/s^2
static constexpr float kDefaultBudget = 1000.0f;   // total deg/s
static constexpr float kDefaultFreqHz = 50.0f;

// ---------------------------------------------------------------------------
// Objects
// ---------------------------------------------------------------------------
static Subsystem::ServoConfig servo_configs[kNumServos];
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

/// @brief Parse a token as uint8_t servo index with bounds check.
static bool parseIndex(const char* tok, uint8_t& out) {
  char* end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0') {
    printErr("invalid index — must be integer");
    return false;
  }
  if (v < 0 || v >= servos->getNumServos()) {
    Serial.printf("ERR: index %ld out of range [0..%u)\r\n", v,
                  servos->getNumServos());
    return false;
  }
  out = static_cast<uint8_t>(v);
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
      "help                    Show this help\r\n"
      "status                  Show all servos\r\n"
      "status <i>              Show servo i\r\n"
      "attach <i>              Attach servo\r\n"
      "detach <i>              Detach servo\r\n"
      "angle <i> <deg>         Set target angle\r\n"
      "vel <i> <deg/s>         Set max velocity\r\n"
      "accel <i> <deg/s^2>     Set max acceleration\r\n"
      "invert <i> <0|1>        Set inverted flag\r\n"
      "minangle <i> <deg>      Set min angle limit\r\n"
      "maxangle <i> <deg>      Set max angle limit\r\n"
      "minpwm <i> <val>        Set min PWM (0-4095)\r\n"
      "maxpwm <i> <val>        Set max PWM (0-4095)\r\n"
      "freq <hz>               Set PWM frequency\r\n"
      "budget <deg/s>          Set total rate budget\r\n"
      "arm                     Enable OE (outputs on)\r\n"
      "disarm                  Disable OE (outputs off)\r\n"
      "===============================\r\n");
}

static void printServoStatus(uint8_t i) {
  const auto& cfg = servos->getConfig(i);
  Serial.printf(
      "  [%2u] ch=%2u %s%s | angle: %.1f -> %.1f (vel %.1f) | "
      "limits [%.1f, %.1f] | pwm [%u, %u]%s | vel_max=%.1f accel=%.1f\r\n",
      i, cfg.channel, servos->isAttached(i) ? "ATT" : "DET",
      servos->isInitialized(i) ? "" : " UNINIT", servos->getCurrentAngle(i),
      servos->getTargetAngle(i), servos->getVelocity(i), cfg.min_angle,
      cfg.max_angle, cfg.min_pwm, cfg.max_pwm, cfg.inverted ? " INV" : "",
      cfg.max_velocity, cfg.max_accel);
}

static void cmdStatus() {
  Serial.printf("Armed: %s | Budget: %.0f deg/s\r\n",
                servos->isArmed() ? "YES" : "NO", servos->getTotalRateBudget());
  if (num_tokens >= 2) {
    uint8_t idx;
    if (parseIndex(tokens[1], idx)) printServoStatus(idx);
  } else {
    for (uint8_t i = 0; i < servos->getNumServos(); i++) {
      printServoStatus(i);
    }
  }
}

static void cmdAttach() {
  if (!requireArgs(2)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  servos->attach(idx);
  Serial.printf("OK: servo %u attached (ch%u)\r\n", idx,
                servos->getConfig(idx).channel);
}

static void cmdDetach() {
  if (!requireArgs(2)) return;
  uint8_t idx;
  if (!parseIndex(tokens[1], idx)) return;
  servos->detach(idx);
  Serial.printf("OK: servo %u detached (ch%u)\r\n", idx,
                servos->getConfig(idx).channel);
}

static void cmdAngle() {
  if (!requireArgs(3)) return;
  uint8_t idx;
  float deg;
  if (!parseIndex(tokens[1], idx)) return;
  if (!parseFloat(tokens[2], deg)) return;
  servos->setAngle(idx, deg);
  Serial.printf("OK: servo %u target = %.2f deg\r\n", idx, deg);
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
  Serial.printf("OK: servo %u max_velocity = %.2f deg/s\r\n", idx, v);
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
  Serial.printf("OK: servo %u max_accel = %.2f deg/s^2\r\n", idx, a);
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
  Serial.printf("OK: servo %u inverted = %s\r\n", idx, v ? "true" : "false");
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
  Serial.printf("OK: servo %u min_angle = %.2f\r\n", idx, deg);
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
  Serial.printf("OK: servo %u max_angle = %.2f\r\n", idx, deg);
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
  Serial.printf("OK: servo %u min_pwm = %u\r\n", idx, val);
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
  Serial.printf("OK: servo %u max_pwm = %u\r\n", idx, val);
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
  else if (strcmp(cmd, "arm") == 0)
    cmdArm();
  else if (strcmp(cmd, "disarm") == 0)
    cmdDisarm();
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
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(1000);  // let serial settle

  // Populate default servo configs — all 16 channels, all detached
  for (uint8_t i = 0; i < kNumServos; i++) {
    servo_configs[i] = {
        .channel = i,
        .min_angle = kDefaultMinAngle,
        .max_angle = kDefaultMaxAngle,
        .min_pwm = kDefaultMinPwm,
        .max_pwm = kDefaultMaxPwm,
        .inverted = false,
        .max_velocity = kDefaultVelocity,
        .max_accel = kDefaultAccel,
    };
  }

  // Heartbeat blink
  blink.beginThreadedPinned(2048, 1, 500, 1);

  // Servo subsystem
  static Subsystem::ServoSetup servo_setup(
      Wire, Config::pca_addr, Config::servo_en, servo_configs, kNumServos,
      kDefaultBudget, kDefaultFreqHz);
  auto& srv = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  if (!srv.init()) {
    Debug::printf(Debug::Level::ERROR, "[MAIN] Servo init failed");
    Serial.println("ERR: Servo subsystem init failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  servos = &srv;
  srv.beginThreadedPinned(4096, 3, 20, 1);  // 50 Hz update

  Serial.println("\r\n=== Servo Subsystem Test ===");
  Serial.println("All servos DETACHED. OE DISABLED.");
  Serial.println("Typical workflow: attach 0 -> angle 0 90 -> arm");
  printHelp();
}

void loop() { processSerial(); }
