/**
 * @file test_sub_wifi.cpp
 * @brief ESP32WifiSubsystem test — connects to AP from network_config.ini
 *        and prints connection state + network info to Serial.
 * @author Aldem Pido
 * @date 3/31/26
 */
#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "WifiSubsystem", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

static const char* stateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED: return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:   return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:    return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING: return "RECONNECTING";
    case Subsystem::WifiState::FAILED:       return "FAILED";
    default:                                 return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] ESP32WifiSubsystem init FAILED");
    return;
  }

  // Core 1, priority 3, 100 ms update interval, 4 KB stack
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi subsystem started, connecting to \"%s\"",
                WIFI_SSID);
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  Subsystem::WifiState state = wifi.getState();

  if (wifi.isConnected()) {
    String ssid = wifi.getSSID();
    String ip = wifi.getLocalIP().toString();
    Debug::printf(Debug::Level::INFO,
                  "[Loop] state=%-12s  SSID=%-20s  IP=%-15s  RSSI=%d dBm",
                  stateStr(state), ssid.c_str(), ip.c_str(), wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] state=%s", stateStr(state));
  }

  delay(2000);
}
