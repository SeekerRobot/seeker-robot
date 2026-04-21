#include "GyroSubsystem.h"

// BNO085 soft-reset caused cold-boot hangs on our boards — default OFF. Flip
// via -DGYRO_ENABLE_SOFT_RESET=1 if you need to exercise the code path.
#ifndef GYRO_ENABLE_SOFT_RESET
#define GYRO_ENABLE_SOFT_RESET 0
#endif

namespace Subsystem {
bool GyroSubsystem::init() {
  Threads::Scope lock(i2c_mutex_);
  if (!setup_.wire_.begin()) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Wire.begin() failure");
    return false;
  }
  setup_.wire_.setClock(400000);
  if (!bno08x_.begin_I2C(setup_.addr_, &setup_.wire_)) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Failed to find BNO08x chip");
    return false;
  }
  for (int n = 0; n < bno08x_.prodIds.numEntries; n++) {
    Debug::printf(Debug::Level::INFO,
                  "[BNO085] Part %u: Version :%u.%u.%u Build %u",
                  bno08x_.prodIds.entry[n].swPartNumber,
                  bno08x_.prodIds.entry[n].swVersionMajor,
                  bno08x_.prodIds.entry[n].swVersionMinor,
                  bno08x_.prodIds.entry[n].swVersionPatch,
                  bno08x_.prodIds.entry[n].swBuildNumber);
  }
  //reset();
  setReorientation();
  // Attach ISR before enabling reports: BNO085 INT is active-low and level-
  // held until I2C drain, so if setReports() fires the first report before
  // the ISR is wired up, we miss the FALLING edge and never see another one
  // (the line stays LOW). Kick the semaphore once so update()'s first pass
  // drains whatever is already queued and releases INT back to HIGH.
  pinMode(setup_.int_pin_, INPUT_PULLUP);
  attachInterruptArg(digitalPinToInterrupt(setup_.int_pin_), intISR, this,
                     FALLING);
  setReports();
  xSemaphoreGive(int_semaphore_);
  Debug::printf(Debug::Level::INFO, "[BNO085] Init success");
  return true;
}

void IRAM_ATTR GyroSubsystem::intISR(void* arg) {
  auto* self = static_cast<GyroSubsystem*>(arg);
  self->isr_count_++;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(self->int_semaphore_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GyroSubsystem::update() {
  xSemaphoreTake(int_semaphore_, portMAX_DELAY);
  Threads::Scope i2c_lock(i2c_mutex_);

  if (bno08x_.wasReset()) {
    Debug::printf(Debug::Level::INFO, "[BNO085] Sensor was reset");
    setReports();
  }

  // Accumulate sensor events into a local copy so data_mutex_ is held only
  // during the final write, not during I2C communication.
  ImuData local;
  {
    Threads::Scope data_lock(data_mutex_);
    local = imu_data_;
  }

  sh2_SensorValue_t sensorValue;
  while (bno08x_.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        local.gameRotationVector = sensorValue.un.gameRotationVector;
        break;
      case SH2_LINEAR_ACCELERATION:
        local.linearAcceleration = sensorValue.un.linearAcceleration;
        break;
      case SH2_GRAVITY:
        local.gravity = sensorValue.un.gravity;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        local.gyroscope = sensorValue.un.gyroscope;
        break;
      case SH2_STABILITY_CLASSIFIER:
        local.stabilityClassifier = sensorValue.un.stabilityClassifier;
        break;
    }
  }

  {
    Threads::Scope data_lock(data_mutex_);
    imu_data_ = local;
  }

  if (since_last_log_ >= kLogIntervalMs) {
    since_last_log_ = 0;
    logImuData();
  }
}

ImuData GyroSubsystem::getImuData() const {
  Threads::Scope lock(data_mutex_);
  return imu_data_;
}

int GyroSubsystem::pollOnce() {
  Threads::Scope i2c_lock(i2c_mutex_);
  sh2_SensorValue_t sensorValue;
  int drained = 0;
  while (bno08x_.getSensorEvent(&sensorValue)) {
    ++drained;
  }
  return drained;
}

void GyroSubsystem::reset() {
  Threads::Scope lock(i2c_mutex_);
  Debug::printf(Debug::Level::INFO, "[BNO085] Software resetting");
  sh2_reinitialize();
  delay(500);  // wait for bus to stabilize
}

void GyroSubsystem::logImuData() {
  const float qx = imu_data_.gameRotationVector.i;
  const float qy = imu_data_.gameRotationVector.j;
  const float qz = imu_data_.gameRotationVector.k;
  const float qw = imu_data_.gameRotationVector.real;
  // ZYX intrinsic (roll-pitch-yaw), ROS REP 103 convention.
  const float sinp = 2.0f * (qw * qy - qz * qx);
  const float pitch =
      (fabsf(sinp) >= 1.0f) ? copysignf(M_PI_2, sinp) : asinf(sinp);
  const float roll = atan2f(2.0f * (qw * qx + qy * qz),
                            1.0f - 2.0f * (qx * qx + qy * qy));
  const float yaw = atan2f(2.0f * (qw * qz + qx * qy),
                           1.0f - 2.0f * (qy * qy + qz * qz));
  constexpr float kRad2Deg = 180.0f / static_cast<float>(M_PI);
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] RPY: roll=%+7.2f pitch=%+7.2f yaw=%+7.2f deg",
                roll * kRad2Deg, pitch * kRad2Deg, yaw * kRad2Deg);
  Debug::printf(
      Debug::Level::VERBOSE, "[BNO085] GRV: i=%.3f j=%.3f k=%.3f r=%.3f",
      imu_data_.gameRotationVector.i, imu_data_.gameRotationVector.j,
      imu_data_.gameRotationVector.k, imu_data_.gameRotationVector.real);
  Debug::printf(Debug::Level::VERBOSE,
                "[BNO085] LinAccel: x=%.3f y=%.3f z=%.3f",
                imu_data_.linearAcceleration.x, imu_data_.linearAcceleration.y,
                imu_data_.linearAcceleration.z);
  Debug::printf(Debug::Level::VERBOSE, "[BNO085] Gravity: x=%.3f y=%.3f z=%.3f",
                imu_data_.gravity.x, imu_data_.gravity.y, imu_data_.gravity.z);
  Debug::printf(Debug::Level::VERBOSE, "[BNO085] Gyro: x=%.3f y=%.3f z=%.3f",
                imu_data_.gyroscope.x, imu_data_.gyroscope.y,
                imu_data_.gyroscope.z);
  Debug::printf(Debug::Level::VERBOSE, "[BNO085] Stability: %u",
                imu_data_.stabilityClassifier.classification);
}

void GyroSubsystem::setReorientation() {
  Threads::Scope lock(i2c_mutex_);
  // Chip mounting (default BNO085 orientation): physical X = robot right,
  // physical Y = robot forward, physical Z = up.
  //
  // Target output frame: ROS REP 103 (X = forward, Y = left, Z = up).
  //
  // Redefining the device frame as ROS: East = robot forward, North = robot
  // left. Physical chip axes in that device frame:
  //   X = robot right = South  (opposite of East)
  //   Y = robot forward = East
  //   Z = Up
  //
  // From datasheet Figure 4-3, row X=South, Y=East, Z=Up:
  //   Qw = sqrt(2)/2, Qx = 0, Qy = 0, Qz = -sqrt(2)/2  (-90 deg around Z)
  sh2_Quaternion_t orient;
  orient.x = 0.0f;
  orient.y = 0.0f;
  orient.z = -0.7071068f;
  orient.w = 0.7071068f;
  int rc = sh2_setReorientation(&orient);
  if (rc != SH2_OK) {
    Debug::printf(Debug::Level::ERROR,
                  "[BNO085] Failed to set reorientation (%d)", rc);
  } else {
    Debug::printf(Debug::Level::INFO,
                  "[BNO085] Reorientation set: -90 deg around Z -> ROS REP 103 "
                  "(X=fwd, Y=left, Z=up)");
  }
}

void GyroSubsystem::tareYaw() {
  Threads::Scope lock(i2c_mutex_);
  // Tare Z axis only — zeros yaw to the current heading without affecting
  // pitch/roll, which remain gravity-referenced.
  int rc = sh2_setTareNow(SH2_TARE_Z, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
  if (rc != SH2_OK) {
    Debug::printf(Debug::Level::ERROR, "[BNO085] Yaw tare failed (%d)", rc);
  } else {
    Debug::printf(Debug::Level::INFO, "[BNO085] Yaw tared to current heading");
  }
}

void GyroSubsystem::setReports() {
  Debug::printf(Debug::Level::INFO,
                "[BNO085] enableReport GYRO_CAL      -> %d",
                bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED));
  Debug::printf(Debug::Level::INFO,
                "[BNO085] enableReport LIN_ACCEL     -> %d",
                bno08x_.enableReport(SH2_LINEAR_ACCELERATION));
  Debug::printf(Debug::Level::INFO,
                "[BNO085] enableReport GRAVITY       -> %d",
                bno08x_.enableReport(SH2_GRAVITY));
  Debug::printf(Debug::Level::INFO,
                "[BNO085] enableReport GAME_ROT_VEC  -> %d",
                bno08x_.enableReport(SH2_GAME_ROTATION_VECTOR));
  Debug::printf(Debug::Level::INFO,
                "[BNO085] enableReport STABILITY     -> %d",
                bno08x_.enableReport(SH2_STABILITY_CLASSIFIER));
  Debug::printf(Debug::Level::INFO, "[BNO085] Reports set");
}
};  // namespace Subsystem