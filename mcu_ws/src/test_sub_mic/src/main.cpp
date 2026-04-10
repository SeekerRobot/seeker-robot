/**
 * @file test_sub_mic/src/main.cpp
 * @author Tal Avital
 * @date 4/7/2026
 * @brief MicSubsystem test — streams PDM mic audio over HTTP via the subsystem
 *        abstraction.
 *
 * curl -s http://<static_ip>:81/audio | aplay -r 16000 -f S16_LE -c 1
 */

#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <MicSubsystem.h>
#include <RobotConfig.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("WifiSubsystem", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

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

  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  mic.beginThreadedPinned(4096, 2, 5000, 1);
}

void loop() { delay(10000); }
