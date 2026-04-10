/**
 * @file test_ble_debug.cpp
 * @author Aldem Pido
 * @date 4/1/26
 * @brief BLE Nordic UART Serial debugging test.
 */
#include <Arduino.h>
#include <BleDebugSubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>

static Subsystem::BleDebugSetup ble_setup("SeekerDebug");

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

void setup() {
  Serial.begin(921600);
  delay(500);

  auto& ble = Subsystem::BleDebugSubsystem::getInstance(ble_setup);
  if (!ble.init()) {
    Serial.printf("[Main] BleDebugSubsystem init FAILED\n");
  }

  blink.beginThreadedPinned(2048, 1, 500, 1);
  // Core 0: co-located with NimBLE's host task to reduce cross-core overhead.
  ble.beginThreadedPinned(8192, 2, 50, 0);
}

void loop() {
  static uint32_t counter = 0;
  Debug::printf(Debug::Level::INFO, "[Loop] tick %u", counter++);
  delay(1000);
}
