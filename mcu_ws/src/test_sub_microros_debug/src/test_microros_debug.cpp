/**
 * @file test_microros_debug.cpp
 * @author Aldem Pido
 * @date 4/3/2026
 * @brief Integration test for the micro-ROS debug log transport.
 *
 * Verifies that Debug::printf output is forwarded to the micro-ROS agent via
 * the MicroRosBridge debug publisher (BRIDGE_ENABLE_DEBUG=1).
 *
 * Build flags (set in platformio.ini):
 *   -DDEBUG_TRANSPORT_SERIAL      — local echo over USB serial
 *   -DDEBUG_TRANSPORT_MICROROS    — routes Debug::printf → mcu/log
 *   -DBRIDGE_ENABLE_HEARTBEAT=1   — confirms agent is alive
 *   -DBRIDGE_ENABLE_DEBUG=1       — enables the log publisher
 *
 * Verify on host:
 *   ros2 topic echo /mcu/log        # formatted strings with [I]/[W]/[E] prefix
 *   ros2 topic hz   /mcu/heartbeat  # ~1 Hz confirms connection
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <MicroRosBridge.h>
#include <RobotConfig.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "debug_test_node");
static Subsystem::MicrorosManager manager(manager_setup);

static const char* wifiStateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED:
      return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:
      return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:
      return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING:
      return "RECONNECTING";
    case Subsystem::WifiState::FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);

  blink.beginThreadedPinned(2048, 1, 500, 1);

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  // Bridge: heartbeat + debug log only — no subsystem pointers needed.
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  manager.registerParticipant(&bridge);
  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  manager.beginThreadedPinned(8192, 4, 10, 1);
}

static uint8_t s_counter = 0;

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (wifi.isConnected()) {
    Debug::printf(Debug::Level::INFO,
                  "[Loop] wifi=%-12s  microros=%-18s  ip=%-15s  rssi=%d dBm",
                  wifiStateStr(wifi.getState()), manager.getStateStr(),
                  wifi.getLocalIP().toString().c_str(), wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%-12s  microros=%s",
                  wifiStateStr(wifi.getState()), manager.getStateStr());
  }

  // One message per level every 2 s to exercise the full pipeline.
  Debug::printf(Debug::Level::DEBUG, "[Test] counter=%u  level=DEBUG",
                s_counter);
  Debug::printf(Debug::Level::INFO, "[Test] counter=%u  level=INFO", s_counter);
  Debug::printf(Debug::Level::WARN, "[Test] counter=%u  level=WARN", s_counter);
  Debug::printf(Debug::Level::ERROR, "[Test] counter=%u  level=ERROR",
                s_counter);
  s_counter++;

  delay(2000);
}
