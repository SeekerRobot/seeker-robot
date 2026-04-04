/**
 * @file test_raw_cam.cpp
 * @brief ESP32CAM test — displays a camera stream on an HTTP web server
 * @author Tal Avital
 * @date 3/31/26
 */
#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>

#include "camera_pins.h"
#include "camera_web_server.h"

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("WifiSubsystem", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

void setup() {
  Serial.begin(921600);
  delay(500);

  // Configure camera
  // Check PSRAM before init
  if (psramFound()) {
    camera_config.frame_size = FRAMESIZE_VGA;
    camera_config.jpeg_quality = 10;
    camera_config.fb_count = 2;
  } else {
    camera_config.frame_size = FRAMESIZE_QQVGA;
    camera_config.fb_count = 1;
  }

  // Init
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    return;
  }

  // Connect to WIFI
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  // Start camera web server
  startCameraWebServer();
}

void loop() { delay(10000); }
