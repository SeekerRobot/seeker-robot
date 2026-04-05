/**
 * @file test_sub_gait.cpp
 * @date 4/3/2026
 * @brief Serial-driven integration test for GaitController + HexapodKinematics
 *        + ServoSubsystem. No WiFi or micro-ROS — all commands over serial.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * Validates the full MCU-side locomotion stack without any ROS2 infrastructure.
 * GaitController and ServoSubsystem each run on their own FreeRTOS tasks;
 * this loop only handles serial input.
 *
 * All robot-specific configuration (servo calibration, link lengths, leg
 * geometry, gait tuning) lives in HexapodConfig.h. Edit that file, not this one.
 *
 * Expected workflow:
 *   1. Flash and open serial at 921600 baud.
 *   2. Type 'help' to see available commands.
 *   3. 'start' then 'vel 0.05 0' — walk forward at 50 mm/s.
 *   4. 'stop' for a clean finish (all feet land before IDLE).
 *   5. 'halt' for an immediate snap back to neutral.
 *
 * Verify: Group A legs (FL=0, MR=3, RL=4) and Group B legs (FR=1, ML=2, RR=5)
 *         alternate in tripod fashion. Use 'status' to inspect phase values.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <GaitController.h>
#include <HexapodConfig.h>
#include <HexapodKinematics.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>

// ---------------------------------------------------------------------------
// Shared resources
// ---------------------------------------------------------------------------
static Subsystem::ServoConfig servo_configs[HexapodConfig::kNumServos];
static Threads::Mutex         i2c_mutex;

static Classes::BaseSetup      blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

// Pointers set in setup() after singleton / object construction
static Kinematics::HexapodKinematics* kinematics = nullptr;
static Gait::GaitController*          gait        = nullptr;

// ---------------------------------------------------------------------------
// Serial parser (mirrors test_sub_servo pattern)
// ---------------------------------------------------------------------------
static constexpr size_t  kLineBufSize = 128;
static constexpr uint8_t kMaxTokens   = 4;
static char    line_buf[kLineBufSize];
static uint8_t line_pos = 0;
static char*   tokens[kMaxTokens];
static uint8_t num_tokens = 0;

static void printOk(const char* msg)  { Serial.printf("OK:  %s\r\n", msg); }
static void printErr(const char* msg) { Serial.printf("ERR: %s\r\n", msg); }

static void tokenize() {
  num_tokens = 0;
  char* p = line_buf;
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
    Serial.printf("ERR: expected %u arg(s), got %u. Type 'help'.\r\n",
                  n - 1, num_tokens - 1);
    return false;
  }
  return true;
}

static bool parseFloat(const char* tok, float& out) {
  char* end;
  float v = strtof(tok, &end);
  if (end == tok || *end != '\0') { printErr("invalid number"); return false; }
  out = v;
  return true;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void printHelp() {
  Serial.println(
      "\r\n"
      "===== Gait Test Commands =====\r\n"
      "help                Show this help\r\n"
      "start               enable()  — begin walking\r\n"
      "stop                disable() — clean stop (legs finish landing)\r\n"
      "halt                stop()    — immediate snap to neutral\r\n"
      "vel <vx> <wz>       Set forward vel (m/s) and yaw rate (rad/s)\r\n"
      "status              Print gait state, velocity, leg phases\r\n"
      "neutral             halt then standNeutral\r\n"
      "==============================\r\n");
}

static const char* stateName(Gait::GaitState s) {
  switch (s) {
    case Gait::GaitState::IDLE:     return "IDLE";
    case Gait::GaitState::WALKING:  return "WALKING";
    case Gait::GaitState::STOPPING: return "STOPPING";
    default:                        return "UNKNOWN";
  }
}

static const char* kLegNames[Gait::kNumLegs] = {
    "FL", "FR", "ML", "MR", "RL", "RR"};

static void cmdStatus() {
  Gait::GaitState       s   = gait->getState();
  Gait::VelocityCommand vel = gait->getVelocity();

  Serial.printf("State: %s\r\n", stateName(s));
  Serial.printf("Cmd:   vx=%+.3f m/s  vy=%+.3f m/s  wz=%+.3f rad/s\r\n",
                vel.vx, vel.vy, vel.wz);
  Serial.println("Legs:");
  for (uint8_t i = 0; i < Gait::kNumLegs; i++) {
    const Gait::LegGaitState& ls = gait->legState(i);
    bool flying = (ls.phase_t >= 0.5f);
    Serial.printf(
        "  [%u] %s  phase=%4.2f  %s%s\r\n",
        i, kLegNames[i], ls.phase_t,
        flying  ? "FLYING" : "STANCE",
        ls.frozen ? " (frozen)" : "");
  }
}

static void cmdStart() {
  gait->enable();
  printOk("enable() called — gait WALKING");
}

static void cmdStop() {
  gait->disable();
  printOk("disable() called — gait STOPPING (clean landing)");
}

static void cmdHalt() {
  gait->stop();
  printOk("stop() called — snapped to neutral");
}

static void cmdVel() {
  if (!requireArgs(3)) return;
  float vx, wz;
  if (!parseFloat(tokens[1], vx)) return;
  if (!parseFloat(tokens[2], wz)) return;
  gait->setVelocity(vx, 0.f, wz);
  Serial.printf("OK:  vx=%+.3f m/s  wz=%+.3f rad/s\r\n", vx, wz);
}

static void cmdNeutral() {
  gait->stop();
  printOk("halted and snapped to neutral");
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------
static void processCommand() {
  tokenize();
  if (num_tokens == 0) return;

  // Lowercase the command token
  for (char* p = tokens[0]; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p += 32;
  }

  const char* cmd = tokens[0];
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) printHelp();
  else if (strcmp(cmd, "start")   == 0) cmdStart();
  else if (strcmp(cmd, "stop")    == 0) cmdStop();
  else if (strcmp(cmd, "halt")    == 0) cmdHalt();
  else if (strcmp(cmd, "vel")     == 0) cmdVel();
  else if (strcmp(cmd, "status")  == 0 || strcmp(cmd, "s") == 0) cmdStatus();
  else if (strcmp(cmd, "neutral") == 0) cmdNeutral();
  else Serial.printf("ERR: unknown command '%s'. Type 'help'.\r\n", cmd);
}

static void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_pos > 0) {
        line_buf[line_pos] = '\0';
        Serial.printf("> %s\r\n", line_buf);
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
  delay(500);

  blink.beginThreadedPinned(2048, 1, 500, 1);

  // --- Servo configs — from HexapodConfig.h ---
  for (uint8_t i = 0; i < HexapodConfig::kNumServos; i++) {
    servo_configs[i] = HexapodConfig::kServoConfigs[i];
  }

  // --- ServoSubsystem ---
  static Subsystem::ServoSetup servo_setup(
      Wire, Config::pca_addr, Config::servo_en,
      servo_configs, HexapodConfig::kNumServos,
      HexapodConfig::kServoBudget, HexapodConfig::kPwmFreqHz);

  auto& servos = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  for (uint8_t i = 0; i < HexapodConfig::kNumServos; i++) servos.attach(i);
  if (!servos.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Servo init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 5 | 10 ms update | 4096 words
  servos.beginThreadedPinned(4096, 5, 10, 1);

  // --- HexapodKinematics ---
  static Kinematics::HexapodKinematics kin(HexapodConfig::kLegConfigs);
  kinematics = &kin;

  // --- GaitController ---
  static Gait::GaitSetup gait_setup(HexapodConfig::kGaitConfig, kinematics, &servos);
  static Gait::GaitController gait_ctrl(gait_setup);
  gait = &gait_ctrl;

  if (!gait->init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gait init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 4 | 10 ms update | 8192 words
  gait->beginThreadedPinned(8192, 4, 10, 1);

  Serial.println("\r\n=== Gait Subsystem Test ===");
  Serial.println("GaitController started. Servos armed in begin().");
  printHelp();
}

void loop() { processSerial(); }
