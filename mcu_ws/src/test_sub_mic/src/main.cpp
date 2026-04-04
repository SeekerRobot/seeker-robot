/**
 * @file test_sub_mic/src/main.cpp
 * @author Tal Avital
 * @date 4/4/2026
 * @brief ESP32 Sense mic submodule
 */

// Code from: https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/

#include <ESP_I2S.h>
I2SClass I2S;

void setup() {
  // Baud rate found from other files
  Serial.begin(921600);

  // Set up clock (42) and data pins (41)
  I2S.setPinsPdmRx(42, 41);

  // Start I2S at 16 kHz with 16 bits per sample
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH,
                 I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  void loop() {
    // Read a sample
    int sample = I2S.read();

    if (sample && sample != -1 && sample != 1) {
      Serial.println(sample);
    }
  }
}