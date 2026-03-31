/**
 * @file ESP32WifiSubsystem.h
 * @brief ESP32 WiFi connection management subsystem for micro-ROS
 */
#pragma once

#ifdef ESP32

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <ThreadedSubsystem.h>
#include <WiFi.h>

namespace Subsystem {

enum class WifiState : uint8_t {
  DISCONNECTED = 0,
  CONNECTING = 1,
  CONNECTED = 2,
  RECONNECTING = 3,
  FAILED = 4
};

class ESP32WifiSubsystemSetup : public Classes::BaseSetup {
 public:
  ESP32WifiSubsystemSetup() = delete;

  ESP32WifiSubsystemSetup(const char* _id, const char* ssid,
                          const char* password,
                          uint32_t connection_timeout_ms = 10000,
                          uint32_t reconnect_interval_ms = 5000,
                          uint8_t max_retries = 5)
      : Classes::BaseSetup(_id),
        ssid_(ssid),
        password_(password),
        connection_timeout_ms_(connection_timeout_ms),
        reconnect_interval_ms_(reconnect_interval_ms),
        max_retries_(max_retries) {}

  ESP32WifiSubsystemSetup(const char* _id, const char* ssid,
                          const char* password, IPAddress local_ip,
                          IPAddress gateway, IPAddress subnet,
                          uint32_t connection_timeout_ms = 10000,
                          uint32_t reconnect_interval_ms = 5000,
                          uint8_t max_retries = 5)
      : Classes::BaseSetup(_id),
        ssid_(ssid),
        password_(password),
        connection_timeout_ms_(connection_timeout_ms),
        reconnect_interval_ms_(reconnect_interval_ms),
        max_retries_(max_retries),
        local_ip_(local_ip),
        gateway_(gateway),
        subnet_(subnet),
        use_static_ip_(true) {}

  const char* ssid_;
  const char* password_;
  uint32_t connection_timeout_ms_;
  uint32_t reconnect_interval_ms_;
  uint8_t max_retries_;

  IPAddress local_ip_{INADDR_NONE};
  IPAddress gateway_{INADDR_NONE};
  IPAddress subnet_{INADDR_NONE};
  bool use_static_ip_ = false;
};

class ESP32WifiSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  ESP32WifiSubsystem(const ESP32WifiSubsystem&) = delete;
  ESP32WifiSubsystem& operator=(const ESP32WifiSubsystem&) = delete;

  static ESP32WifiSubsystem& getInstance(const ESP32WifiSubsystemSetup& setup) {
    static ESP32WifiSubsystem instance(setup);
    return instance;
  }

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  WifiState getState() const { return state_; }
  bool isConnected() const { return state_ == WifiState::CONNECTED; }
  bool isConnecting() const {
    return state_ == WifiState::CONNECTING || state_ == WifiState::RECONNECTING;
  }
  bool hasFailed() const { return state_ == WifiState::FAILED; }

  IPAddress getLocalIP() const { return WiFi.localIP(); }
  int32_t getRSSI() const { return WiFi.RSSI(); }
  String getSSID() const { return WiFi.SSID(); }

  void disconnect();
  void reconnect();
  void clearFailedState();

 private:
  explicit ESP32WifiSubsystem(const ESP32WifiSubsystemSetup& setup)
      : Subsystem::ThreadedSubsystem(setup), setup_(setup) {
    s_instance_ = this;
  }

  void transitionTo(WifiState new_state);
  void attemptConnection();
  void handleConnected();
  void handleDisconnected();
  void powerCycleRadio();

  static void onWifiEvent(WiFiEvent_t event);
  static ESP32WifiSubsystem* s_instance_;

  static constexpr uint8_t kRadioResetInterval = 3;

  const ESP32WifiSubsystemSetup setup_;

  WifiState state_ = WifiState::DISCONNECTED;
  uint8_t retry_count_ = 0;
  uint8_t consecutive_failures_ = 0;
  uint32_t state_entry_time_ms_ = 0;
  uint32_t last_attempt_time_ms_ = 0;
  uint32_t last_health_check_ms_ = 0;

  volatile bool event_connected_ = false;
  volatile bool event_disconnected_ = false;
  volatile bool event_got_ip_ = false;
};

}  // namespace Subsystem

#endif  // ESP32
