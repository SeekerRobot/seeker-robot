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

// Boards where the BNO INT line isn't wired (e.g., ESP32-CAM satellite with
// all GPIOs spoken for by the camera) build with -DGYRO_USE_INT=0. update()
// then polls the chip on the ThreadedSubsystem cadence — callers MUST pass a
// non-zero updateDelayMs (~5 ms for 200 Hz headroom) to beginThreadedPinned.
#ifndef GYRO_USE_INT
#define GYRO_USE_INT 1
#endif

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
  /// @param sda_pin Optional SDA pin. Pass -1 (default) to let Wire use the
  ///                board's default SDA. Required on boards whose default
  ///                Wire pins collide with other peripherals (ESP32-CAM
  ///                defaults to GPIO21 = camera Y5, so pass Config::sda
  ///                there).
  /// @param scl_pin Optional SCL pin. Same semantics as sda_pin.
  GyroSetup(TwoWire& wire, uint8_t addr, int8_t int_pin, int8_t sda_pin = -1,
            int8_t scl_pin = -1)
      : Classes::BaseSetup("GyroSubsystem"),
        wire_(wire),
        addr_(addr),
        int_pin_(int_pin),
        sda_pin_(sda_pin),
        scl_pin_(scl_pin) {}

  TwoWire& wire_;
  uint8_t addr_;
  int8_t int_pin_;
  int8_t sda_pin_;
  int8_t scl_pin_;
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
  ///        Always 0 when built with GYRO_USE_INT=0.
  uint32_t getIsrCount() const {
#if GYRO_USE_INT
    return isr_count_;
#else
    return 0;
#endif
  }

  /// @brief Current logic level on the BNO INT pin. BNO drives LOW when a
  ///        report is queued; returns HIGH when the bus has been drained.
  ///        Returns -1 when built with GYRO_USE_INT=0 (pin not in use).
  int readIntPin() const {
#if GYRO_USE_INT
    return digitalRead(setup_.int_pin_);
#else
    return -1;
#endif
  }

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
        i2c_mutex_(i2c_mutex)
#if GYRO_USE_INT
        ,
        int_semaphore_(xSemaphoreCreateBinary())
#endif
  {
  }

  const GyroSetup setup_;
  Threads::Mutex& i2c_mutex_;
  /// Protects imu_data_ between the gyro task (writer) and any reader task.
  mutable Threads::Mutex data_mutex_;

  static constexpr unsigned long kLogIntervalMs = 1000;

  Adafruit_BNO08x bno08x_;
  ImuData imu_data_ = {};
#if GYRO_USE_INT
  SemaphoreHandle_t int_semaphore_;
  volatile uint32_t isr_count_ = 0;
#endif
  elapsedMillis since_last_log_;

#if GYRO_USE_INT
  static void IRAM_ATTR intISR(void* arg);
#endif

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