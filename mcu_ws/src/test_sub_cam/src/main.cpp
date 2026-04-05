/**
 * @file main.cpp
 * @author Tal Avital
 * @date 4/5/2026
 * @brief WiFi + MJPEG stream hardware test for CameraSubsystem (OV2640).
 *
 * Connects to WiFi (DHCP), initialises CameraSubsystem, and serves an MJPEG
 * stream at http://<IP>/stream on port 80.  No micro-ROS — pure hardware
 * verification.
 *
 * Steps:
 *   1. Flash the sketch.
 *   2. Open a serial monitor at 921600 baud.
 *   3. Wait for "Stream ready" and note the printed URL.
 *   4. Open the URL in a browser to view the live feed.
 *
 * Commands (send with newline):
 *   status        Print camera and stream state
 *   ip            Print current WiFi IP address
 *   help / ?      Show this help
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CameraSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <RobotConfig.h>
#include <camera_pins.h>

// ---------------------------------------------------------------------------
// WiFi setup (DHCP — clients connect to IP shown on serial)
// ---------------------------------------------------------------------------
static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, IPAddress(STATIC_IP), IPAddress(GATEWAY),
    IPAddress(SUBNET));

// ---------------------------------------------------------------------------
// Camera setup
// ---------------------------------------------------------------------------
static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80);

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

static void printHelp() {
  Serial.println(
      "\r\n"
      "===== Camera Subsystem Test =========================\r\n"
      "status           Print camera and stream state\r\n"
      "ip               Print current WiFi IP address\r\n"
      "help / ?         Show this help\r\n"
      "====================================================\r\n");
}

static void cmdStatus() {
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  Serial.printf("[Camera] ready=%s  stream=%s  port=80\r\n",
                cam.isCameraReady() ? "yes" : "no",
                cam.isServerRunning() ? "up" : "down");
}

static void cmdIp() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (wifi.isConnected()) {
    Serial.printf("[WiFi] IP: %s\r\n", wifi.getLocalIP().toString().c_str());
  } else {
    Serial.println("[WiFi] not connected");
  }
}

static void processCommand(const char* cmd) {
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    printHelp();
  else if (strcmp(cmd, "status") == 0)
    cmdStatus();
  else if (strcmp(cmd, "ip") == 0)
    cmdIp();
  else
    Serial.printf("ERR: unknown command '%s'. Type 'help'.\r\n", cmd);
}

static void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_pos > 0) {
        line_buf[line_pos] = '\0';
        // lowercase
        for (uint8_t i = 0; i < line_pos; i++) {
          if (line_buf[i] >= 'A' && line_buf[i] <= 'Z') line_buf[i] += 32;
        }
        Serial.printf("> %s\r\n", line_buf);
        processCommand(line_buf);
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

  // Connect WiFi (blocking until connected or subsystem reaches FAILED state)
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  wifi.init();
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  Serial.println("[WiFi] Connecting...");
  while (!wifi.isConnected() && !wifi.hasFailed()) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  if (wifi.hasFailed()) {
    Serial.println("ERR: WiFi connection failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  Serial.printf("[WiFi] Connected — IP: %s\r\n",
                wifi.getLocalIP().toString().c_str());

  // Start camera subsystem (Core 1, priority 2, 5 s update interval)
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  if (!cam.init()) {
    Serial.println("ERR: CameraSubsystem init failed — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  cam.beginThreadedPinned(4096, 2, 5000, 1);

  // Wait briefly for begin() to fire inside the task
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.println("\r\n=== Camera Subsystem Test ===");
  if (cam.isCameraReady()) {
    Serial.printf("Stream ready: http://%s/stream\r\n",
                  wifi.getLocalIP().toString().c_str());
  } else {
    Serial.println("WARN: camera not ready — check wiring and OV2640 sensor");
  }
  printHelp();
}

void loop() { processSerial(); }
