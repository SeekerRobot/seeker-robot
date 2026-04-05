#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>

#include <I2S.h>

#include "mic_web_server.h"

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("WifiSubsystem", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

void setup() {
  Serial.begin(921600);

  // start I2S at 16 kHz with 16-bits per sample
  I2S.setAllPins(-1, 42, 41, -1, -1);
  if (!I2S.begin(PDM_MONO_MODE, 16000, 16)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  // Connect to WIFI
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  // Start camera web server
  startCameraWebServer();

}

void loop() {
  // read a sample
  int sample = I2S.read();

  if (sample && sample != -1 && sample != 1) {
    Serial.println(sample);
  }
}