#include "GyroSubsystem.h"

namespace Subsystem {
bool GyroSubsystem::init() {
  Threads::Scope lock(i2c_mutex_);
  if (!setup_.wire_.begin()) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Wire.begin() failure");
    return false;
  }
  if (!setup_.wire_.setClock(400000)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Wire.setClock() failure");
    return false;
  }
  if (!bno08x_.begin_I2C(setup_.addr_, &setup_.wire_)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Failed to find BNO08x chip");
    return false;
  }
  for (int n = 0; n < bno08x_.prodIds.numEntries; n++) {
    Debug::printf(Debug::Level::INFO, "[BNO085] Part %u: Version :%u.%u.%u Build %u",
                  bno08x_.prodIds.entry[n].swPartNumber,
                  bno08x_.prodIds.entry[n].swVersionMajor,
                  bno08x_.prodIds.entry[n].swVersionMinor,
                  bno08x_.prodIds.entry[n].swVersionPatch,
                  bno08x_.prodIds.entry[n].swBuildNumber);
  }
  reset();
  setReports();
  pinMode(setup_.int_pin_, INPUT_PULLUP);
  attachInterruptArg(digitalPinToInterrupt(setup_.int_pin_), intISR, this, FALLING);
  Debug::printf(Debug::Level::INFO, "[BNO085] Init success");
  return true;
}

void IRAM_ATTR GyroSubsystem::intISR(void* arg) {
  auto* self = static_cast<GyroSubsystem*>(arg);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(self->int_semaphore_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GyroSubsystem::update() {
  xSemaphoreTake(int_semaphore_, portMAX_DELAY);
  Threads::Scope lock(i2c_mutex_);
  if (bno08x_.wasReset()) {
    Debug::printf(Debug::Level::INFO, "[BNO085] Sensor was reset");
    setReports();
  }

  sh2_SensorValue_t sensorValue;
  while (bno08x_.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        imu_data_.gameRotationVector = sensorValue.un.gameRotationVector;
        break;
      case SH2_LINEAR_ACCELERATION:
        imu_data_.linearAcceleration = sensorValue.un.linearAcceleration;
        break;
      case SH2_GRAVITY:
        imu_data_.gravity = sensorValue.un.gravity;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imu_data_.gyroscope = sensorValue.un.gyroscope;
        break;
      case SH2_STABILITY_CLASSIFIER:
        imu_data_.stabilityClassifier = sensorValue.un.stabilityClassifier;
        break;
    }
  }
  if (since_last_log_ >= kLogIntervalMs) {
    since_last_log_ = 0;
    logImuData();
  }
}

void GyroSubsystem::reset() {
  Threads::Scope lock(i2c_mutex_);
  Debug::printf(Debug::Level::INFO, "[BNO085] Software resetting");
  sh2_reinitialize();
  delay(500);  // wait for bus to stabilize
}

void GyroSubsystem::logImuData() {
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] GRV: i=%.3f j=%.3f k=%.3f r=%.3f",
                imu_data_.gameRotationVector.i,
                imu_data_.gameRotationVector.j,
                imu_data_.gameRotationVector.k,
                imu_data_.gameRotationVector.real);
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] LinAccel: x=%.3f y=%.3f z=%.3f",
                imu_data_.linearAcceleration.x,
                imu_data_.linearAcceleration.y,
                imu_data_.linearAcceleration.z);
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] Gravity: x=%.3f y=%.3f z=%.3f",
                imu_data_.gravity.x,
                imu_data_.gravity.y,
                imu_data_.gravity.z);
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] Gyro: x=%.3f y=%.3f z=%.3f",
                imu_data_.gyroscope.x,
                imu_data_.gyroscope.y,
                imu_data_.gyroscope.z);
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] Stability: %u",
                imu_data_.stabilityClassifier.classification);
}

void GyroSubsystem::setReports() {
  if (!bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Could not enable gyroscope");
  }
  if (!bno08x_.enableReport(SH2_LINEAR_ACCELERATION)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Could not enable linear acceleration");
  }
  if (!bno08x_.enableReport(SH2_GRAVITY)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Could not enable gravity vector");
  }
  if (!bno08x_.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Could not enable game rotation vector");
  }
  if (!bno08x_.enableReport(SH2_STABILITY_CLASSIFIER)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Could not enable stability classifier");
  }
  Debug:printf(Debug::Level::INFO, "[BNO085] Reports set");
}
};  // namespace Subsystem