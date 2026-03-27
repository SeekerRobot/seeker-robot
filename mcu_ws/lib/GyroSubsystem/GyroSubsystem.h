/**
 * @file GyroSubsystem.h
 * @author Aldem Piod
 * @date 3/26/2026
 * @brief Defines the BNO085 Driver Subsystem for sesame.
 * Meant to be ran at high priority.
 */
#pragma once
#include <Adafruit_BNO08x.h>
#include <CustomDebug.h>
#include <ThreadedSubsystem.h>
#include <Wire.h>
#include <elapsedMillis.h>
#include <hal_thread.h>
#include <sh2.h>

namespace Subsystem {

struct ImuData {
  sh2_RotationVector_t gameRotationVector;
  sh2_Accelerometer_t linearAcceleration;
  sh2_Accelerometer_t gravity;
  sh2_Gyroscope_t gyroscope;
  sh2_StabilityClassifier_t stabilityClassifier;
};
class GyroSetup : public Classes::BaseSetup {
 public:
  ~GyroSetup() = default;
  GyroSetup() = delete;

  /// @brief Creates a GyroSetup object for GyroSubsystem.
  /// @param wire TwoWire Wire bus.
  /// @param addr BNO085 Address.
  /// @param int_pin Interrupt pin used (required)
  GyroSetup(TwoWire& wire, uint8_t addr, int8_t int_pin)
      : Classes::BaseSetup("GyroSubsystem"),
        wire_(wire),
        addr_(addr),
        int_pin_(int_pin) {}

  TwoWire& wire_;
  uint8_t addr_;
  int8_t int_pin_;
};

class GyroSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  // Delete copy and assignment operator to force singleton
  GyroSubsystem(const GyroSubsystem&) = delete;
  GyroSubsystem& operator=(const GyroSubsystem&) = delete;

  // Create static method to get singleton
  static GyroSubsystem& getInstance(const GyroSetup& setup,
                                    Threads::Mutex& i2c_mutex) {
    static GyroSubsystem instance(setup, i2c_mutex);
    return instance;
  }

  // Logic functions
  bool init() override;
  void begin() override { return; }
  void update() override;
  void pause() override { return; }
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

 private:
  // Constructor moved to private to prevent multiple creations
  explicit GyroSubsystem(const GyroSetup& setup, Threads::Mutex& i2c_mutex)
      : ThreadedSubsystem(setup),
        setup_(setup),
        i2c_mutex_(i2c_mutex),
        int_semaphore_(xSemaphoreCreateBinary()) {}

  const GyroSetup setup_;
  Threads::Mutex& i2c_mutex_;

  static constexpr unsigned long kLogIntervalMs = 500;

  Adafruit_BNO08x bno08x_;
  ImuData imu_data_ = {};
  SemaphoreHandle_t int_semaphore_;
  elapsedMillis since_last_log_;

  static void IRAM_ATTR intISR(void* arg);

  /// @brief Set the BNO085 reports.
  void setReports();
  /// @brief Log IMU data.
  void logImuData();
};
};  // namespace Subsystem