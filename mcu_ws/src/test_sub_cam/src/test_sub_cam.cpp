/**
 * @file test_sub_cam.cpp
 * @author Tal Avital
 * @date 4/4/26
 * @brief Camera subsystem test — streams MJPEG over HTTP using
 *        CameraSubsystem, ESP32WifiSubsystem, and BlinkSubsystem.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CameraSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <camera_pins.h>

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "WifiSubsystem", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

static Subsystem::CameraSetup cam_setup(camera_config);

void setup() {
  Serial.begin(921600);
  delay(500);

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED");
    return;
  }

  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  if (!cam.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Camera init FAILED");
    return;
  }

  blink.beginThreadedPinned(2048, 1, 500, 1);
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  cam.beginThreadedPinned(4096, 2, 5000, 1);

  Debug::printf(Debug::Level::INFO, "[Main] All subsystems started");
}

void loop() {}
