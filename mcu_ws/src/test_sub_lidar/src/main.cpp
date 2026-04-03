/**
 * @file main.cpp
 * @author Aldem Pido
 * @date 4/2/2026
 * @brief Serial-only hardware test for LidarSubsystem (LD14P).
 *
 * Initialises LidarSubsystem and streams scan summaries over USB Serial.
 * No micro-ROS or WiFi — pure hardware verification.
 *
 * Wire the LD14P to Config::tx (MCU→sensor) and Config::rx (sensor→MCU):
 *   ESP32DEV:     rx=GPIO16  tx=GPIO17
 *   ESP32S3SENSE: rx=D7      tx=D6
 *
 * Commands (921600 baud, send with newline):
 *   scan           Print latest complete scan point-by-point
 *   stream <0|1>   Enable/disable 1 Hz summary (default: on)
 *   freq           Print current scan frequency reported by sensor
 *   info           Print subsystem configuration
 *   help / ?       Show this help
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <LidarSubsystem.h>
#include <RobotConfig.h>

// ---------------------------------------------------------------------------
// LiDAR setup
// ---------------------------------------------------------------------------
// Config::rx / Config::tx are the UART2-capable pins on both board variants.
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);

// ---------------------------------------------------------------------------
// Blink
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
    if (!*p) break;
    tokens[num_tokens++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = '\0';
  }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void printHelp() {
  Serial.println(
      "\r\n"
      "===== LiDAR Subsystem Test ==========================\r\n"
      "scan             Print latest complete scan\r\n"
      "stream <0|1>     Toggle 1 Hz summary (default: on)\r\n"
      "freq             Print current scan frequency (Hz)\r\n"
      "info             Print subsystem configuration\r\n"
      "help / ?         Show this help\r\n"
      "====================================================\r\n");
}

static void cmdScan() {
  // Static to avoid placing 8.6 KB on the Arduino main-task stack.
  static Subsystem::LidarScanData scan;
  scan = Subsystem::LidarSubsystem::getInstance(lidar_setup).getScanData();
  if (!scan.valid) {
    Serial.println("ERR: no complete scan yet — wait for first revolution");
    return;
  }
  Serial.printf("Scan #%lu  points=%u\r\n", (unsigned long)scan.scan_count,
                scan.count);
  for (uint16_t i = 0; i < scan.count; i++) {
    Serial.printf("  [%3u] angle=%6.2f deg  dist=%7.1f mm  quality=%.0f\r\n", i,
                  scan.angles_deg[i], scan.distances_mm[i], scan.qualities[i]);
  }
}

static void cmdStream() {
  if (num_tokens < 2) {
    Serial.println("ERR: usage: stream <0|1>");
    return;
  }
  char* end;
  long v = strtol(tokens[1], &end, 10);
  if (end == tokens[1] || (v != 0 && v != 1)) {
    Serial.println("ERR: value must be 0 or 1");
    return;
  }
  stream_enabled = (v == 1);
  Serial.printf("OK: streaming %s\r\n",
                stream_enabled ? "enabled" : "disabled");
}

static void cmdFreq() {
  float hz = Subsystem::LidarSubsystem::getInstance(lidar_setup)
                 .getCurrentScanFreqHz();
  Serial.printf("Scan freq: %.2f Hz\r\n", hz);
}

static void cmdInfo() {
  Serial.printf(
      "LidarSubsystem — rx=%d  tx=%d  rx_buf=%lu B  target=%.1f Hz\r\n",
      lidar_setup.rx_pin_, lidar_setup.tx_pin_,
      (unsigned long)lidar_setup.rx_buf_size_, lidar_setup.scan_freq_hz_);
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
  else if (strcmp(cmd, "scan") == 0)
    cmdScan();
  else if (strcmp(cmd, "stream") == 0)
    cmdStream();
  else if (strcmp(cmd, "freq") == 0)
    cmdFreq();
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
// 1 Hz stream summary: scan number, point count, dist range, freq
// ---------------------------------------------------------------------------
static void printStreamSummary() {
  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  // Static to avoid placing 8.6 KB on the Arduino main-task stack.
  static Subsystem::LidarScanData scan;
  scan = lidar.getScanData();
  if (!scan.valid) {
    Serial.println("[Lidar] waiting for first complete scan...");
    return;
  }
  float dmin = 1.0e9f, dmax = 0.0f;
  for (uint16_t i = 0; i < scan.count; i++) {
    if (scan.distances_mm[i] > 0.0f) {
      if (scan.distances_mm[i] < dmin) dmin = scan.distances_mm[i];
      if (scan.distances_mm[i] > dmax) dmax = scan.distances_mm[i];
    }
  }
  Serial.printf(
      "[Lidar] scan #%lu  pts=%u  dist=[%.0f..%.0f] mm  freq=%.2f Hz\r\n",
      (unsigned long)scan.scan_count, scan.count, dmin, dmax,
      lidar.getCurrentScanFreqHz());
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(1000);

  blink.beginThreadedPinned(2048, 1, 500, 1);

  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) {
    Serial.println("ERR: LidarSubsystem init failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 0 | priority 4 | 1 ms update | 6144 word (24 KB) stack
  lidar.beginThreadedPinned(6144, 4, 1, 0);

  Serial.println("\r\n=== LiDAR Subsystem Test ===");
  cmdInfo();
  printHelp();
}

void loop() {
  processSerial();

  static uint32_t last_stream_ms = 0;
  if (stream_enabled && (millis() - last_stream_ms >= 1000)) {
    last_stream_ms = millis();
    printStreamSummary();
  }
}
