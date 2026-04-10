/**
 * @file LidarSubsystem.cpp
 * @author Aldem Pido
 * @date 4/2/2026
 */
#include "LidarSubsystem.h"

namespace Subsystem {

LidarSubsystem* LidarSubsystem::instance_ = nullptr;

// ---------------------------------------------------------------------------
// init() — called from Arduino setup() before beginThreadedPinned()
// ---------------------------------------------------------------------------
bool LidarSubsystem::init() {
  instance_ = this;

  // Configure serial with enlarged RX buffer.
  // ESP32 HardwareSerial::begin() overload:
  //   (baud, config, rxPin, txPin, invert, rxBufSize, txBufSize)
  setup_.serial_.begin(lidar_.getSerialBaudRate(), SERIAL_8N1, setup_.rx_pin_,
                       setup_.tx_pin_, /*invert=*/false, setup_.rx_buf_size_);

  lidar_.setSerialReadCallback(serialReadCb);
  lidar_.setSerialWriteCallback(serialWriteCb);
  // scan_completed=true fires on the FIRST point of the NEW revolution,
  // not the last of the completed one. onScanPoint() handles this ordering.
  lidar_.setScanPointCallback(scanPointCb);
  lidar_.setMotorPinCallback(motorPinCb);
  lidar_.setInfoCallback(infoCb);
  lidar_.setErrorCallback(errorCb);

  lidar_.init();

  Debug::printf(Debug::Level::INFO,
                "[Lidar] init ok — baud %lu  rx=%d  tx=%d  target %.1f Hz",
                (unsigned long)lidar_.getSerialBaudRate(), setup_.rx_pin_,
                setup_.tx_pin_, setup_.scan_freq_hz_);

  initialized_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// begin() — called once by the FreeRTOS task before the update() loop.
//           Runs after init(), so the serial port is already configured.
// ---------------------------------------------------------------------------
void LidarSubsystem::begin() {
  if (!initialized_) return;

  LDS::result_t r = lidar_.setScanTargetFreqHz(setup_.scan_freq_hz_);
  if (r != LDS::RESULT_OK) {
    Debug::printf(Debug::Level::WARN,
                  "[Lidar] setScanTargetFreqHz(%.1f) returned %d",
                  setup_.scan_freq_hz_, (int)r);
  }

  r = lidar_.start();
  if (r != LDS::RESULT_OK) {
    Debug::printf(Debug::Level::ERROR, "[Lidar] start() failed: %d", (int)r);
    return;
  }
  Debug::printf(Debug::Level::INFO, "[Lidar] started — model: %s",
                lidar_.getModelName());
}

// ---------------------------------------------------------------------------
// update() — called by FreeRTOS task at ~1 ms cadence.
//            LD14P::loop() drains all bytes in the serial RX buffer.
// ---------------------------------------------------------------------------
void LidarSubsystem::update() {
  if (!initialized_) return;
  lidar_.loop();
}

// ---------------------------------------------------------------------------
// pause() / reset()
// ---------------------------------------------------------------------------
void LidarSubsystem::pause() {
  if (!initialized_) return;
  lidar_.stop();
  Debug::printf(Debug::Level::INFO, "[Lidar] paused (motor stopped)");
}

void LidarSubsystem::reset() {
  Threads::Scope lock(data_mutex_);
  published_ = LidarScanData{};
}

// ---------------------------------------------------------------------------
// getScanData() — thread-safe getter
// ---------------------------------------------------------------------------
void LidarSubsystem::getScanData(LidarScanData& out) const {
  Threads::Scope lock(data_mutex_);
  out = published_;
}

float LidarSubsystem::getCurrentScanFreqHz() {
  // Benign float read from update() task — acceptable for diagnostic telemetry.
  return lidar_.getCurrentScanFreqHz();
}

// ---------------------------------------------------------------------------
// onScanPoint() — dispatched from scanPointCb(); runs in update() task.
// accum_ is exclusively owned by the update() task — no lock needed on it.
// ---------------------------------------------------------------------------
void LidarSubsystem::onScanPoint(float angle_deg, float dist_mm, float quality,
                                 bool scan_completed) {
  if (scan_completed) {
    // scan_completed fires on the FIRST point of the new revolution.
    // Finalise the accumulated scan and swap it into published_.
    accum_.valid = true;
    accum_.scan_count = ++completed_scans_;
    {
      Threads::Scope lock(data_mutex_);
      published_ = accum_;
    }
    accum_ = LidarScanData{};  // reset for next scan
  }

  // Store the current point into the (possibly just-reset) accumulation buffer.
  if (accum_.count < kLidarMaxPoints) {
    accum_.angles_deg[accum_.count] = angle_deg;
    accum_.distances_mm[accum_.count] = dist_mm;
    accum_.qualities[accum_.count] = quality;
    accum_.count++;
  }
}

// ---------------------------------------------------------------------------
// Static callback stubs — forward to the singleton via instance_
// ---------------------------------------------------------------------------
int LidarSubsystem::serialReadCb() {
  if (!instance_) return -1;
  return instance_->setup_.serial_.read();
}

size_t LidarSubsystem::serialWriteCb(const uint8_t* buf, size_t len) {
  if (!instance_) return 0;
  return instance_->setup_.serial_.write(buf, len);
}

void LidarSubsystem::scanPointCb(float angle_deg, float dist_mm, float quality,
                                 bool scan_completed) {
  if (instance_)
    instance_->onScanPoint(angle_deg, dist_mm, quality, scan_completed);
}

void LidarSubsystem::motorPinCb(float value, LDS::lds_pin_t pin) {
  // LD14P motor is controlled via serial commands; no GPIO action required.
  (void)value;
  (void)pin;
}

void LidarSubsystem::infoCb(LDS::info_t code, String info) {
  if (!instance_) return;
  Debug::printf(Debug::Level::INFO, "[Lidar] %s: %s",
                instance_->lidar_.infoCodeToString(code).c_str(), info.c_str());
}

void LidarSubsystem::errorCb(LDS::result_t code, String aux) {
  if (!instance_) return;
  // ERROR_CHECKSUM fires frequently during motor spin-up — demote to DEBUG.
  Debug::Level lvl =
      (code == LDS::ERROR_CHECKSUM) ? Debug::Level::DEBUG : Debug::Level::WARN;
  Debug::printf(lvl, "[Lidar] %s: %s",
                instance_->lidar_.resultCodeToString(code).c_str(),
                aux.c_str());
}

}  // namespace Subsystem
