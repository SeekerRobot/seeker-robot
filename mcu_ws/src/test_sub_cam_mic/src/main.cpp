/**
 * @file test_sub_cam_mic/src/main.cpp
 * @author Tal Avital
 * @date 4/7/2026
 * @brief CameraSubsystem + MicSubsystem side-by-side test.
 *
 *   GET http://<static_ip>:80/cam    — MJPEG video stream
 *   GET http://<static_ip>:81/audio  — raw 16-bit PCM, 16kHz, mono
 *
 * Each subsystem runs its own httpd instance. ctrl_port must be unique per
 * instance (camera=32768 default, mic=32769 default) — this is what allows
 * two httpd servers to coexist on the same device.
 */

#include <Arduino.h>
#include <CameraSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <MicSubsystem.h>
#include <RobotConfig.h>
#include <camera_pins.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("WifiSubsystem", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80);

static Subsystem::MicSetup mic_setup(I2S_NUM_0,
                                     /*sample_rate=*/16000,
                                     /*clk_pin=*/Config::pdm_clk,
                                     /*data_pin=*/Config::pdm_data,
                                     /*gain=*/4,
                                     /*http_port=*/81);

void setup() {
  Serial.begin(921600);
  delay(500);

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  cam.beginThreadedPinned(8192, 2, 5000, 1);

  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  mic.beginThreadedPinned(4096, 2, 5000, 0);
}

void loop() { delay(10000); }
