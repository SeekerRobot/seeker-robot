/**
 * @file test_sub_battery.cpp
 * @author Aldem Pido
 * @date 4/2/2026
 * @brief Interactive serial test for BatterySubsystem.
 *
 * Continuously streams voltage + raw ADC readings at 1 Hz and exposes a
 * small command set for calibration and diagnostics.
 *
 * Commands (type 'help' in the serial monitor at 921600 baud):
 *   read              Print one voltage + raw ADC reading immediately
 *   stream <0|1>      Enable (1) or disable (0) 1 Hz streaming
 *   cal               Print current calibration points
 *   info              Print subsystem configuration
 *   help              Show this help
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BlinkSubsystem.h>
#include <RobotConfig.h>

// ---------------------------------------------------------------------------
// Battery calibration & setup
// ---------------------------------------------------------------------------
// Adjust these two data points to match your voltage-divider circuit.
// Measure the actual battery voltage with a multimeter and record the
// corresponding raw ADC value reported by 'read'.
//
// Example divider: Vbatt → 100kΩ → ADC pin → 100kΩ → GND
//   Gives Vadc = Vbatt / 2, so a 3.7 V lipo reads ~1.85 V → ~2300 raw.
static constexpr Subsystem::BatteryCalibration kCalibration(
    /*raw_lo=*/1862, /*volt_lo=*/3.0f,  // low point  (e.g. discharged lipo)
    /*raw_hi=*/2480, /*volt_hi=*/4.2f   // high point (e.g. fully charged)
);
static constexpr uint8_t kNumSamples = 16;  // ADC samples averaged per reading

static Subsystem::BatterySetup battery_setup(Config::batt, kCalibration,
                                             kNumSamples);

// ---------------------------------------------------------------------------
// Objects
// ---------------------------------------------------------------------------
static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

// ---------------------------------------------------------------------------
// Serial parser
// ---------------------------------------------------------------------------
static constexpr size_t kLineBufSize = 64;
static char line_buf[kLineBufSize];
static uint8_t line_pos = 0;

static constexpr uint8_t kMaxTokens = 4;
static char* tokens[kMaxTokens];
static uint8_t num_tokens = 0;

static bool stream_enabled = true;

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

static void printOk(const char* msg) { Serial.printf("OK: %s\r\n", msg); }
static void printErr(const char* msg) { Serial.printf("ERR: %s\r\n", msg); }

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void printHelp() {
  Serial.println(
      "\r\n"
      "===== Battery Subsystem Test =========================\r\n"
      "read                   Print one reading (voltage + raw ADC)\r\n"
      "stream <0|1>           Disable/enable 1 Hz auto-print (default: on)\r\n"
      "cal                    Print calibration points\r\n"
      "info                   Print subsystem info\r\n"
      "help / ?               Show this help\r\n"
      "=====================================================\r\n");
}

static void cmdRead() {
  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  Serial.printf("voltage=%.3f V  raw=%u\r\n", batt.getVoltage(),
                batt.getRawAdc());
}

static void cmdStream() {
  if (num_tokens < 2) {
    printErr("usage: stream <0|1>");
    return;
  }
  char* end;
  long v = strtol(tokens[1], &end, 10);
  if (end == tokens[1] || (v != 0 && v != 1)) {
    printErr("value must be 0 or 1");
    return;
  }
  stream_enabled = (v == 1);
  Serial.printf("OK: streaming %s\r\n",
                stream_enabled ? "enabled" : "disabled");
}

static void cmdCal() {
  const auto& c = battery_setup.calibration_;
  Serial.printf("Calibration: [raw %u → %.3f V]  [raw %u → %.3f V]\r\n",
                c.raw_lo, c.volt_lo, c.raw_hi, c.volt_hi);
}

static void cmdInfo() {
  Serial.printf("BatterySubsystem — pin %d  samples %u  id '%s'\r\n",
                battery_setup.adc_pin_, battery_setup.num_samples_,
                battery_setup.getId());
  cmdCal();
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------
static void processCommand() {
  tokenize();
  if (num_tokens == 0) return;

  for (char* p = tokens[0]; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p += 32;
  }
  const char* cmd = tokens[0];

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    printHelp();
  else if (strcmp(cmd, "read") == 0)
    cmdRead();
  else if (strcmp(cmd, "stream") == 0)
    cmdStream();
  else if (strcmp(cmd, "cal") == 0)
    cmdCal();
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

  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  if (!batt.init()) {
    Serial.println("ERR: BatterySubsystem init failed");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Sample every 100 ms (10 Hz); getVoltage() is always fresh.
  // 4096 B stack: analogRead (IDF ADC oneshot driver) + Debug::printf with
  // %.3f (dtoa_r ~700 B) + 256-byte buf + recursive_mutex = tight in 2048 B.
  batt.beginThreadedPinned(4096, 2, 100, 1);

  Serial.println("\r\n=== Battery Subsystem Test ===");
  cmdInfo();
  printHelp();
}

void loop() {
  processSerial();

  static uint32_t last_stream_ms = 0;
  if (stream_enabled && (millis() - last_stream_ms >= 1000)) {
    last_stream_ms = millis();
    cmdRead();
  }
}
