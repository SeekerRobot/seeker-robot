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

#ifndef ENABLE_ARDUINO_OTA
#define ENABLE_ARDUINO_OTA 0
#endif

#if ENABLE_ARDUINO_OTA
#include <ArduinoOTA.h>
#endif

namespace Subsystem {

/// @brief WiFi connection states driven by the internal state machine.
enum class WifiState : uint8_t {
  DISCONNECTED = 0,
  CONNECTING = 1,
  CONNECTED = 2,
  RECONNECTING = 3,
  FAILED = 4  ///< Max retries exceeded. Call clearFailedState() to retry.
};

/// @brief Setup configuration for ESP32WifiSubsystem.
class ESP32WifiSubsystemSetup : public Classes::BaseSetup {
 public:
  ESP32WifiSubsystemSetup() = delete;

  /// @brief DHCP mode — ESP32 receives an IP from the router.
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

  /// @brief Static IP mode — required for micro-ROS so the agent can reach the
  ///        ESP32 at a known address.
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
  uint32_t connection_timeout_ms_;  ///< Timeout per connection attempt (ms).
  uint32_t reconnect_interval_ms_;  ///< Delay between retry attempts (ms).
  uint8_t max_retries_;  ///< Max failures before FAILED state. 0 = infinite.

  IPAddress local_ip_{INADDR_NONE};
  IPAddress gateway_{INADDR_NONE};
  IPAddress subnet_{INADDR_NONE};
  bool use_static_ip_ = false;
};

/**
 * @brief Manages the WiFi connection lifecycle for micro-ROS connectivity.
 *
 * Runs as a FreeRTOS task via ThreadedSubsystem. Handles initial connection,
 * automatic reconnection on dropout, and periodic health checks. WiFi events
 * are received via a static callback and consumed in update().
 *
 * Usage:
 * @code
 * static ESP32WifiSubsystemSetup setup("wifi", WIFI_SSID, WIFI_PASSWORD,
 *                                      local_ip, gateway, subnet);
 * auto& wifi = ESP32WifiSubsystem::getInstance(setup);
 * wifi.init();
 * wifi.beginThreadedPinned(4096, 3, 100, 1);
 * @endcode
 */
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

  /// @brief Returns the current state machine state.
  WifiState getState() const { return state_; }
  bool isConnected() const { return state_ == WifiState::CONNECTED; }
  bool isConnecting() const {
    return state_ == WifiState::CONNECTING || state_ == WifiState::RECONNECTING;
  }
  bool hasFailed() const { return state_ == WifiState::FAILED; }

  /// @brief Valid only when isConnected() is true.
  IPAddress getLocalIP() const { return WiFi.localIP(); }
  /// @brief Valid only when isConnected() is true.
  int32_t getRSSI() const { return WiFi.RSSI(); }
  /// @brief Valid only when isConnected() is true.
  String getSSID() const { return WiFi.SSID(); }

  /// @brief Drops the connection and transitions to DISCONNECTED.
  void disconnect();
  /// @brief Resets retry counters and reattempts connection from any state.
  void reconnect();
  /// @brief Clears the FAILED state so connection attempts can resume.
  void clearFailedState();

 private:
  explicit ESP32WifiSubsystem(const ESP32WifiSubsystemSetup& setup)
      : Subsystem::ThreadedSubsystem(setup), setup_(setup) {
    s_instance_ = this;
  }

  void transitionTo(WifiState new_state);
  void attemptConnection();
  void applyStaticIP();
  void handleConnected();
  void handleDisconnected();
  /// @brief Full radio power cycle (OFF → delay → STA) to recover from stuck
  ///        PHY or stale association state after repeated failures.
  void powerCycleRadio();

#if ENABLE_ARDUINO_OTA
  void setupOTA();
  bool ota_started_ = false;
#endif

  static void onWifiEvent(WiFiEvent_t event);
  static ESP32WifiSubsystem* s_instance_;

  /// @brief Trigger a radio power cycle after this many consecutive failures.
  static constexpr uint8_t kRadioResetInterval = 3;
  static constexpr uint32_t kHealthCheckIntervalMs = 5000;

  const ESP32WifiSubsystemSetup setup_;

  WifiState state_ = WifiState::DISCONNECTED;
  uint8_t retry_count_ = 0;
  uint8_t consecutive_failures_ = 0;
  uint32_t last_attempt_time_ms_ = 0;
  uint32_t last_health_check_ms_ = 0;

  /// @brief Flags set by the WiFi event callback; consumed in update().
  volatile bool event_disconnected_ = false;
  volatile bool event_got_ip_ = false;
};

}  // namespace Subsystem

#endif  // ESP32
