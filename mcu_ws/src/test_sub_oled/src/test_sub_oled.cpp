/**
 * @file test_sub_oled.cpp
 * @author Claude Code
 * @date 4/10/2026
 * @brief Interactive serial test for OledSubsystem (SSD1306 128x64 I2C).
 * Type 'help' for available commands.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <OledSubsystem.h>
#include <RobotConfig.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Objects
// ---------------------------------------------------------------------------
static Threads::Mutex i2c_mutex;

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::OledSetup oled_setup(i2c_mutex);
static auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);

// ---------------------------------------------------------------------------
// Serial parser (same pattern as test_sub_led)
// ---------------------------------------------------------------------------
static constexpr size_t kLineBufSize = 128;
static char line_buf[kLineBufSize];
static uint8_t line_pos = 0;

static constexpr uint8_t kMaxTokens = 8;
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
    printErr("value must be 0-255");
    return false;
  }
  out = static_cast<uint8_t>(v);
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
      "===== OLED Test Commands =============================\r\n"
      "help                           Show this help\r\n"
      "frame <key>                    Set frame (blank/boot/status)\r\n"
      "text <slot> <x> <y> <msg..>    Set overlay text (slot 0-3)\r\n"
      "cleartext [slot]               Clear overlay(s)\r\n"
      "clear                          Blank everything\r\n"
      "contrast <0-255>               Set display brightness\r\n"
      "invert                         Invert display pixels\r\n"
      "normal                         Normal display pixels\r\n"
      "info                           Show current state\r\n"
      "=====================================================\r\n"
      "Overlay y should be page-aligned (0,8,16,24,32,40,48,56)\r\n"
      "Font is 6x8, max ~21 chars per line\r\n");
}

static void cmdFrame() {
  if (!requireArgs(2)) return;
  oled.setFrame(tokens[1]);
  Serial.printf("OK: frame = '%s'\r\n", tokens[1]);
}

static void cmdText() {
  // text <slot> <x> <y> <message words...>
  if (!requireArgs(5)) return;
  uint8_t slot, x, y;
  if (!parseU8(tokens[1], slot)) return;
  if (slot >= Subsystem::OledSubsystem::kMaxOverlays) {
    Serial.printf("ERR: slot must be 0-%u\r\n",
                  Subsystem::OledSubsystem::kMaxOverlays - 1);
    return;
  }
  if (!parseU8(tokens[2], x)) return;
  if (!parseU8(tokens[3], y)) return;

  // Reassemble remaining tokens into a single string
  static char text_buf[Subsystem::OledSubsystem::kMaxTextLen + 1];
  text_buf[0] = '\0';
  size_t pos = 0;
  for (uint8_t i = 4; i < num_tokens && pos < sizeof(text_buf) - 1; i++) {
    if (i > 4 && pos < sizeof(text_buf) - 1) text_buf[pos++] = ' ';
    size_t len = strlen(tokens[i]);
    size_t copy =
        (pos + len < sizeof(text_buf)) ? len : sizeof(text_buf) - 1 - pos;
    memcpy(text_buf + pos, tokens[i], copy);
    pos += copy;
  }
  text_buf[pos] = '\0';

  oled.setOverlay(slot, x, y, text_buf);
  Serial.printf("OK: slot %u -> \"%s\" at (%u, %u)\r\n", slot, text_buf, x, y);
}

static void cmdClearText() {
  if (num_tokens >= 2) {
    uint8_t slot;
    if (!parseU8(tokens[1], slot)) return;
    oled.clearOverlay(slot);
    Serial.printf("OK: cleared slot %u\r\n", slot);
  } else {
    oled.clearAllOverlays();
    printOk("cleared all overlays");
  }
}

static void cmdClear() {
  oled.clear();
  printOk("display cleared");
}

static void cmdContrast() {
  if (!requireArgs(2)) return;
  uint8_t val;
  if (!parseU8(tokens[1], val)) return;
  oled.setContrast(val);
  Serial.printf("OK: contrast = %u\r\n", val);
}

static void cmdInvert() {
  oled.setInverted(true);
  printOk("inverted");
}

static void cmdNormal() {
  oled.setInverted(false);
  printOk("normal");
}

static void cmdInfo() {
  const char* frame = oled.getCurrentFrame();
  Serial.printf("Frame: %s\r\n", frame ? frame : "(none)");
  Serial.printf("Display: %ux%u | I2C addr: 0x%02X\r\n",
                Subsystem::OledSubsystem::kWidth,
                Subsystem::OledSubsystem::kHeight, Config::oled_addr);
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
  else if (strcmp(cmd, "frame") == 0)
    cmdFrame();
  else if (strcmp(cmd, "text") == 0)
    cmdText();
  else if (strcmp(cmd, "cleartext") == 0)
    cmdClearText();
  else if (strcmp(cmd, "clear") == 0)
    cmdClear();
  else if (strcmp(cmd, "contrast") == 0)
    cmdContrast();
  else if (strcmp(cmd, "invert") == 0)
    cmdInvert();
  else if (strcmp(cmd, "normal") == 0)
    cmdNormal();
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

  // Initialize I2C bus
  Wire.begin(Config::sda, Config::scl);
  Wire.setClock(400000);

  // Initialize OLED display
  if (!oled.init()) {
    Serial.println("FATAL: OLED init failed");
    while (true) vTaskDelay(portMAX_DELAY);
  }

  // Start threaded subsystems
  blink.beginThreadedPinned(2048, 1, 500, 1);
  oled.beginThreadedPinned(4096, 2, 100, 1);  // ~10 Hz, Core 1

  // Show boot frame with version text
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "OLED Test v1.0");
  oled.setOverlay(1, 4, 52, "Type 'help'");

  Serial.println("\r\n=== OLED Subsystem Test ===");
  cmdInfo();
  printHelp();
}

void loop() { processSerial(); }
