/**
 * @file test_bridge_oled.cpp
 * @author Claude Code
 * @date 4/11/2026
 * @brief Test: WiFi + OLED HTTP streaming (no micro-ROS required).
 *
 * The ESP32 connects to WiFi then opens a persistent HTTP connection to the
 * host at AGENT_IP:8384/lcd_out, reads 1024-byte framebuffers, and renders
 * them on the SSD1306 128x64 OLED.  The micro-ROS agent does NOT need to be
 * running — OLED data flows over plain HTTP, independent of DDS.
 *
 * Verify on host (pick one):
 *   ros2 run seeker_display oled_sine_node
 *   ros2 run seeker_media   mp4_player_node
 *
 * Expected serial output once WiFi is up:
 *   [OLED] LCD stream connected
 *   (display updates ~10 Hz)
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <OledSubsystem.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <hal_thread.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;
static IPAddress agent_ip(AGENT_IP);

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Threads::Mutex i2c_mutex;

static Subsystem::OledSetup oled_setup(i2c_mutex, agent_ip, 8384);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

void setup() {
  Serial.begin(921600);
  delay(500);

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  blink.beginThreadedPinned(2048, 1, 500, 1);
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  // --- I2C bus for OLED ---
  Wire.begin(Config::sda, Config::scl);
  Wire.setClock(400000);

  // --- OLED display ---
  // begin() spawns the lcdFetchTask that connects to AGENT_IP:8384/lcd_out.
  auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);
  if (!oled.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] OLED init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 2 | 100 ms update (10 Hz render loop)
  oled.beginThreadedPinned(4096, 2, 100, 1);
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "bridge_oled");
  oled.setOverlay(1, 4, 52, "connecting...");
  Debug::printf(Debug::Level::INFO,
                "[Main] OLED started (HTTP client -> %s:8384)",
                agent_ip.toString().c_str());
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);

  if (wifi.isConnected()) {
    Debug::printf(Debug::Level::INFO,
                  "[Loop] wifi=CONNECTED  ip=%-15s  rssi=%d dBm",
                  wifi.getLocalIP().toString().c_str(), wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%s",
                  wifi.getState() == Subsystem::WifiState::CONNECTING
                      ? "CONNECTING"
                      : "DISCONNECTED");
  }
  delay(2000);
}
