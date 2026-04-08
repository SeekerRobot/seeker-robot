/**
 * @file test_raw_mic/src/main.cpp
 * @author Tal Avital
 * @date 4/5/2026
 * @brief ESP32 Sense Mic raw I2S test — streams PDM audio over HTTP without
 *        the MicSubsystem abstraction. Useful for verifying hardware directly.
 *
 * Open http://<static_ip>/stream in a browser and click "Start Listening".
 */

#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <driver/i2s.h>

#include "mic_web_server.h"

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("WifiSubsystem", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

void setup() {
  Serial.begin(921600);

  // Init I2S for onboard PDM mic (PDM_CLK=42, PDM_DATA=41)
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false,
  };
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_PIN_NO_CHANGE,
      .ws_io_num = 42,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = 41,
  };
  if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) != ESP_OK ||
      i2s_set_pin(I2S_NUM_0, &pin_config) != ESP_OK) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  startMicWebServer();
}

void loop() { delay(1000); }
