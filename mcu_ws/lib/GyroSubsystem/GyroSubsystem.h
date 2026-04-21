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

  /// @brief Returns a thread-safe snapshot of the latest IMU data.
  ///        Safe to call from any task (e.g., the microROS manager task).
  ImuData getImuData() const;

  /// @brief Count of intISR invocations since boot. Lets external code
  ///        confirm the INT line is actually firing without touching
  ///        update() — handy after the pioarduino/IDF migration.
  uint32_t getIsrCount() const { return isr_count_; }

  /// @brief Current logic level on the BNO INT pin. BNO drives LOW when a
  ///        report is queued; returns HIGH when the bus has been drained.
  int readIntPin() const { return digitalRead(setup_.int_pin_); }

  /// @brief Diagnostic: poll the BNO once without waiting on the semaphore.
  ///        Returns the number of sensor events drained this call. If this
  ///        returns >0 while getIsrCount() stays at 0, the chip is producing
  ///        data but the INT line isn't reaching the MCU pin.
  int pollOnce();

 private:
  // Constructor moved to private to prevent multiple creations
  explicit GyroSubsystem(const GyroSetup& setup, Threads::Mutex& i2c_mutex)
      : ThreadedSubsystem(setup),
        setup_(setup),
        i2c_mutex_(i2c_mutex),
        int_semaphore_(xSemaphoreCreateBinary()) {}

  const GyroSetup setup_;
  Threads::Mutex& i2c_mutex_;
  /// Protects imu_data_ between the gyro task (writer) and any reader task.
  mutable Threads::Mutex data_mutex_;

  static constexpr unsigned long kLogIntervalMs = 1000;

  Adafruit_BNO08x bno08x_;
  ImuData imu_data_ = {};
  SemaphoreHandle_t int_semaphore_;
  elapsedMillis since_last_log_;
  volatile uint32_t isr_count_ = 0;

  static void IRAM_ATTR intISR(void* arg);

  /// @brief Set the BNO085 reports.
  void setReports();
  /// @brief Write the mounting-orientation quaternion to the BNO085 FRS so
  ///        all sensor outputs are in ROS REP 103 (X=fwd, Y=left, Z=up).
  ///        Must be called once after begin_I2C() and reset().
  ///        The value persists in flash through soft resets.
  void setReorientation();
  /// @brief Zero the yaw to the robot's current heading while leaving
  ///        pitch/roll gravity-referenced. Safe to call any time after init().
  void tareYaw();
  /// @brief Log IMU data.
  void logImuData();
};
};  // namespace Subsystem