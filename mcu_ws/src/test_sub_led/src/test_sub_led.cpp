/**
 * @file test_sub_led.cpp
 * @author Aldem Pido, Claude Code
 * @date 4/2/2026
 * @brief Interactive serial test for LedSubsystem (SK6812 + WS2812B chain).
 * Type 'help' for available commands.
 * Disclaimer: This file was written mostly with Claude Code.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <LedSubsystem.h>
#include <RobotConfig.h>

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
static constexpr uint8_t kNumLeds = 5;

// ---------------------------------------------------------------------------
// Objects
// ---------------------------------------------------------------------------
static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::LedSetup led_setup(kNumLeds);
static Subsystem::LedSubsystem<Config::rgb_data> leds(led_setup);

// ---------------------------------------------------------------------------
// Serial parser
// ---------------------------------------------------------------------------
static constexpr size_t kLineBufSize = 128;
static char line_buf[kLineBufSize];
static uint8_t line_pos = 0;

static constexpr uint8_t kMaxTokens = 6;
static char* tokens[kMaxTokens];
static uint8_t num_tokens = 0;

static void printOk(const char* msg) { Serial.printf("OK: %s\r\n", msg); }
static void printErr(const char* msg) { Serial.printf("ERR: %s\r\n", msg); }

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

static bool parseU8(const char* tok, uint8_t& out) {
  char* end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0' || v < 0 || v > 255) {
    printErr("value must be 0–255");
    return false;
  }
  out = static_cast<uint8_t>(v);
  return true;
}

static bool parseU16(const char* tok, uint16_t& out) {
  char* end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0' || v < 0 || v > 65535) {
    printErr("value must be 0–65535");
    return false;
  }
  out = static_cast<uint16_t>(v);
  return true;
}

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
      "===== LED Test Commands ==============================\r\n"
      "help                           Show this help\r\n"
      "clear                          All LEDs off\r\n"
      "solid <r> <g> <b>              All LEDs one color\r\n"
      "set <i> <r> <g> <b>            Set individual LED (SOLID mode)\r\n"
      "brightness <0-255>             Set global brightness\r\n"
      "pulse <r> <g> <b> [speed]      All breathe one color\r\n"
      "chase <r> <g> <b> [speed]      Moving dot, static color\r\n"
      "rainbow_pulse [speed]          All pulse cycling hue\r\n"
      "chase_rainbow [speed]          Moving dot, cycling hue\r\n"
      "rainbow_spread [speed]         Each LED different hue, all cycle\r\n"
      "info                           Show LED chain info\r\n"
      "=====================================================\r\n"
      "speed default = 128 (1=slowest, 4096=fastest)\r\n");
}

static void cmdClear() {
  leds.clear();
  printOk("cleared");
}

static void cmdSolid() {
  if (!requireArgs(4)) return;
  uint8_t r, g, b;
  if (!parseU8(tokens[1], r)) return;
  if (!parseU8(tokens[2], g)) return;
  if (!parseU8(tokens[3], b)) return;
  leds.setAll(CRGB(r, g, b));
  Serial.printf("OK: all LEDs -> (%u, %u, %u)\r\n", r, g, b);
}

static void cmdSet() {
  if (!requireArgs(5)) return;
  uint8_t idx, r, g, b;
  if (!parseU8(tokens[1], idx)) return;
  if (idx >= leds.getTotalLeds()) {
    Serial.printf("ERR: index %u out of range [0..%u)\r\n", idx,
                  leds.getTotalLeds());
    return;
  }
  if (!parseU8(tokens[2], r)) return;
  if (!parseU8(tokens[3], g)) return;
  if (!parseU8(tokens[4], b)) return;
  leds.setLed(idx, CRGB(r, g, b));
  // Switch to SOLID so individual LED color is visible
  leds.setEffect(Subsystem::LedMode::SOLID);
  Serial.printf("OK: LED %u -> (%u, %u, %u) [SOLID]\r\n", idx, r, g, b);
}

static void cmdBrightness() {
  if (!requireArgs(2)) return;
  uint8_t bri;
  if (!parseU8(tokens[1], bri)) return;
  leds.setBrightness(bri);
  Serial.printf("OK: brightness = %u\r\n", bri);
}

static void cmdPulse() {
  if (!requireArgs(4)) return;
  uint8_t r, g, b;
  uint16_t speed = 128;
  if (!parseU8(tokens[1], r)) return;
  if (!parseU8(tokens[2], g)) return;
  if (!parseU8(tokens[3], b)) return;
  if (num_tokens >= 5) {
    if (!parseU16(tokens[4], speed)) return;
  }
  leds.setEffect(Subsystem::LedMode::PULSE, CRGB(r, g, b), speed);
  Serial.printf("OK: PULSE (%u, %u, %u) speed=%u\r\n", r, g, b, speed);
}

static void cmdChase() {
  if (!requireArgs(4)) return;
  uint8_t r, g, b;
  uint16_t speed = 128;
  if (!parseU8(tokens[1], r)) return;
  if (!parseU8(tokens[2], g)) return;
  if (!parseU8(tokens[3], b)) return;
  if (num_tokens >= 5) {
    if (!parseU16(tokens[4], speed)) return;
  }
  leds.setEffect(Subsystem::LedMode::CHASE, CRGB(r, g, b), speed);
  Serial.printf("OK: CHASE (%u, %u, %u) speed=%u\r\n", r, g, b, speed);
}

static void cmdRainbowPulse() {
  uint16_t speed = 128;
  if (num_tokens >= 2) {
    if (!parseU16(tokens[1], speed)) return;
  }
  leds.setEffect(Subsystem::LedMode::RAINBOW_PULSE, CRGB::White, speed);
  Serial.printf("OK: RAINBOW_PULSE speed=%u\r\n", speed);
}

static void cmdChaseRainbow() {
  uint16_t speed = 128;
  if (num_tokens >= 2) {
    if (!parseU16(tokens[1], speed)) return;
  }
  leds.setEffect(Subsystem::LedMode::CHASE_RAINBOW, CRGB::White, speed);
  Serial.printf("OK: CHASE_RAINBOW speed=%u\r\n", speed);
}

static void cmdRainbowSpread() {
  uint16_t speed = 128;
  if (num_tokens >= 2) {
    if (!parseU16(tokens[1], speed)) return;
  }
  leds.setEffect(Subsystem::LedMode::RAINBOW_SPREAD, CRGB::White, speed);
  Serial.printf("OK: RAINBOW_SPREAD speed=%u\r\n", speed);
}

static void cmdInfo() {
  Serial.printf("Chain: %u SK6812 LEDs | data pin: %d\r\n", leds.getTotalLeds(),
                Config::rgb_data);
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

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    printHelp();
  else if (strcmp(cmd, "clear") == 0)
    cmdClear();
  else if (strcmp(cmd, "solid") == 0)
    cmdSolid();
  else if (strcmp(cmd, "set") == 0)
    cmdSet();
  else if (strcmp(cmd, "brightness") == 0)
    cmdBrightness();
  else if (strcmp(cmd, "pulse") == 0)
    cmdPulse();
  else if (strcmp(cmd, "chase") == 0)
    cmdChase();
  else if (strcmp(cmd, "rainbow_pulse") == 0)
    cmdRainbowPulse();
  else if (strcmp(cmd, "chase_rainbow") == 0)
    cmdChaseRainbow();
  else if (strcmp(cmd, "rainbow_spread") == 0)
    cmdRainbowSpread();
  else if (strcmp(cmd, "info") == 0)
    cmdInfo();
  else
    Serial.printf("ERR: unknown command '%s'. Type 'help'.\r\n", cmd);
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
  delay(1000);

  blink.beginThreadedPinned(2048, 1, 500, 1);
  leds.beginThreadedPinned(2048, 2, 20, 1);  // ~50 Hz

  Serial.println("\r\n=== LED Subsystem Test ===");
  cmdInfo();
  printHelp();
}

void loop() { processSerial(); }
