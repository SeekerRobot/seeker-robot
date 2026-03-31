/**
 * @file test_sub_heartbeat.cpp
 * @brief Integration test: ESP32WifiSubsystem + MicrorosManager +
 *        HeartbeatParticipant. Connects to WiFi, then publishes an incrementing
 *        counter on mcu/heartbeat at 1 Hz once the micro-ROS agent is reached.
 *
 * Expected Serial output at ~2 s intervals:
 *   [Loop] wifi=CONNECTED  microros=CONNECTED  ip=192.168.x.x  rssi=-XX dBm
 *
 * Verify on the host:
 *   ros2 topic echo /mcu/heartbeat
 * @author Aldem Pido
 * @date 3/31/26
 */
#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <HeartbeatParticipant.h>
#include <microros_manager_robot.h>

// ---------------------------------------------------------------------------
// Network config — injected by platformio.ini from network_config.ini
// ---------------------------------------------------------------------------
static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

// ---------------------------------------------------------------------------
// Subsystem instances
// ---------------------------------------------------------------------------
static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "heartbeat_node");

static Subsystem::HeartbeatParticipantSetup hb_setup("mcu/heartbeat", 1000);
static Subsystem::HeartbeatParticipant heartbeat(hb_setup);
static Subsystem::MicrorosManager manager(manager_setup);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static const char* wifiStateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED:  return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:    return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:     return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING:  return "RECONNECTING";
    case Subsystem::WifiState::FAILED:        return "FAILED";
    default:                                  return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(500);

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 3 | 100 ms update | 4 KB stack
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  // --- Micro-ROS manager ---
  manager.registerParticipant(&heartbeat);

  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  // Core 1 | priority 5 | 10 ms update | 8 KB stack
  manager.beginThreadedPinned(8192, 5, 10, 1);
  Debug::printf(Debug::Level::INFO, "[Main] Micro-ROS manager started");
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);

  if (wifi.isConnected()) {
    Debug::printf(Debug::Level::INFO,
                  "[Loop] wifi=%-12s  microros=%-18s  ip=%-15s  rssi=%d dBm",
                  wifiStateStr(wifi.getState()),
                  manager.getStateStr(),
                  wifi.getLocalIP().toString().c_str(),
                  wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%-12s  microros=%s",
                  wifiStateStr(wifi.getState()),
                  manager.getStateStr());
  }

  delay(2000);
}
