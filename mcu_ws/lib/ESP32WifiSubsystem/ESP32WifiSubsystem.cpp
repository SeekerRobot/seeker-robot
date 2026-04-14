/**
 * @file ESP32WifiSubsystem.cpp
 * @brief Implementation of ESP32 WiFi connection management subsystem
 */

#ifdef ESP32

#include "ESP32WifiSubsystem.h"

namespace Subsystem {

ESP32WifiSubsystem* ESP32WifiSubsystem::s_instance_ = nullptr;

void ESP32WifiSubsystem::onWifiEvent(WiFiEvent_t event) {
  if (!s_instance_) return;

  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_instance_->event_disconnected_ = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      s_instance_->event_got_ip_ = true;
      break;
    default:
      break;
  }
}

bool ESP32WifiSubsystem::init() {
  if (setup_.ssid_ == nullptr) return false;

  state_ = WifiState::DISCONNECTED;
  retry_count_ = 0;
  consecutive_failures_ = 0;
  event_disconnected_ = false;
  event_got_ip_ = false;

  initSuccess_ = true;
  return true;
}

void ESP32WifiSubsystem::begin() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);

  transitionTo(WifiState::CONNECTING);
  attemptConnection();
}

void ESP32WifiSubsystem::update() {
  if (!initSuccess_) return;

  uint32_t now = millis();

  if (event_got_ip_) {
    event_got_ip_ = false;
    event_disconnected_ = false;
    handleConnected();
  }

  if (event_disconnected_) {
    event_disconnected_ = false;
    handleDisconnected();
  }

  switch (state_) {
    case WifiState::DISCONNECTED:
      break;

    case WifiState::CONNECTING:
    case WifiState::RECONNECTING: {
      uint32_t time_since_attempt = now - last_attempt_time_ms_;
      if (time_since_attempt >= setup_.connection_timeout_ms_) {
        WiFi.disconnect(true);
        retry_count_++;
        consecutive_failures_++;

        if (setup_.max_retries_ > 0 && retry_count_ >= setup_.max_retries_) {
          transitionTo(WifiState::FAILED);
        } else {
          if (consecutive_failures_ > 0 &&
              consecutive_failures_ % kRadioResetInterval == 0) {
            powerCycleRadio();
          }
          last_attempt_time_ms_ = now;
        }
      }

      if (retry_count_ > 0 &&
          (now - last_attempt_time_ms_) >= setup_.reconnect_interval_ms_) {
        attemptConnection();
      }
      break;
    }

    case WifiState::CONNECTED:
#if ENABLE_ARDUINO_OTA
      ArduinoOTA.handle();
#endif
      if (now - last_health_check_ms_ >= kHealthCheckIntervalMs) {
        last_health_check_ms_ = now;
        if (WiFi.status() != WL_CONNECTED) {
          handleDisconnected();
        }
      }
      break;

    case WifiState::FAILED:
      break;
  }
}

void ESP32WifiSubsystem::pause() {
  WiFi.disconnect(true);
  transitionTo(WifiState::DISCONNECTED);
}

void ESP32WifiSubsystem::reset() {
  WiFi.disconnect(true);
  retry_count_ = 0;
  consecutive_failures_ = 0;
  event_disconnected_ = false;
  event_got_ip_ = false;
#if ENABLE_ARDUINO_OTA
  ota_started_ = false;
#endif
  transitionTo(WifiState::DISCONNECTED);
}

const char* ESP32WifiSubsystem::getInfo() { return setup_.getId(); }

void ESP32WifiSubsystem::disconnect() {
  WiFi.disconnect(true);
  transitionTo(WifiState::DISCONNECTED);
}

void ESP32WifiSubsystem::reconnect() {
  if (state_ == WifiState::FAILED) clearFailedState();
  retry_count_ = 0;
  consecutive_failures_ = 0;
  transitionTo(WifiState::RECONNECTING);
  attemptConnection();
}

void ESP32WifiSubsystem::clearFailedState() {
  if (state_ == WifiState::FAILED) {
    retry_count_ = 0;
    consecutive_failures_ = 0;
    transitionTo(WifiState::DISCONNECTED);
  }
}

void ESP32WifiSubsystem::transitionTo(WifiState new_state) {
  if (state_ != new_state) {
    state_ = new_state;
  }
}

void ESP32WifiSubsystem::applyStaticIP() {
  if (setup_.use_static_ip_) {
    WiFi.config(setup_.local_ip_, setup_.gateway_, setup_.subnet_,
                setup_.gateway_);
  }
}

void ESP32WifiSubsystem::attemptConnection() {
  WiFi.disconnect(true);
  applyStaticIP();
  WiFi.begin(setup_.ssid_, setup_.password_);
  last_attempt_time_ms_ = millis();
}

void ESP32WifiSubsystem::handleConnected() {
  retry_count_ = 0;
  consecutive_failures_ = 0;
  last_health_check_ms_ = millis();
  transitionTo(WifiState::CONNECTED);
#if ENABLE_ARDUINO_OTA
  setupOTA();
#endif
}

void ESP32WifiSubsystem::handleDisconnected() {
#if ENABLE_ARDUINO_OTA
  ota_started_ = false;
#endif
  if (state_ == WifiState::CONNECTED) {
    retry_count_ = 0;
    transitionTo(WifiState::RECONNECTING);
    attemptConnection();
  }
}

void ESP32WifiSubsystem::powerCycleRadio() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(500));
  WiFi.mode(WIFI_STA);
  applyStaticIP();
}

#if ENABLE_ARDUINO_OTA
void ESP32WifiSubsystem::setupOTA() {
  if (ota_started_) return;
  ArduinoOTA.setHostname(setup_.getId());
  ArduinoOTA.begin();
  ota_started_ = true;
}
#endif

}  // namespace Subsystem

#endif  // ESP32
