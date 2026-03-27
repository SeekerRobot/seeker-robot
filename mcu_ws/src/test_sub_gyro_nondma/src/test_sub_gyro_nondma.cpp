/**
 * @file test_sub_gyro_nondma.cpp
 * @author Aldem Pido
 * @date 3/26/26
 * @brief Simplest possible test to determine if target is alive.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <GyroSubsystem.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <hal_thread.h>

Threads::Mutex i2c_mutex;
static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);
static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);

void setup() {
  Serial.begin(921600);
  blink.beginThreadedPinned(2048, 1, 500, 1);
  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  if (!gyro.init()) {
    Debug::printf(Debug::Level::ERROR, "[MAIN] Gyro init failed");
    return;
  }
  gyro.beginThreadedPinned(4096, 5, 0, 1);
}

void loop() {}
