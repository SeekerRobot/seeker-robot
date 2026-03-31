/**
 * @file BleDebugSubsystem.cpp
 * @author Aldem Pido
 * @date 4/1/26
 */
#include "BleDebugSubsystem.h"

#include <Arduino.h>

namespace Subsystem {

BleDebugSubsystem* BleDebugSubsystem::instance_ = nullptr;

bool BleDebugSubsystem::init() {
  if (!NimBLEDevice::isInitialized()) {
    NimBLEDevice::init(setup_.deviceName);
  }
  NimBLEDevice::setPower(9);  // Max TX power (+9 dBm) for all BLE roles.

  NimBLEServer* pServer = NimBLEDevice::createServer();
  if (!pServer) {
    Serial.printf("[BleDebug] createServer() failed\n");
    return false;
  }
  // deleteCallbacks=false: serverCallbacks_ is a member, not heap-allocated.
  pServer->setCallbacks(&serverCallbacks_, /*deleteCallbacks=*/false);
  pServer->advertiseOnDisconnect(true);

  if (!bleStream_.begin(NimBLEUUID(kServiceUUID), NimBLEUUID(kCharUUID),
                        kTxBufSize, kRxBufSize, /*secure=*/false)) {
    Serial.printf("[BleDebug] bleStream_.begin() failed\n");
    return false;
  }

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(kServiceUUID);
  pAdv->setName(setup_.deviceName);
  pAdv->enableScanResponse(true);
  if (!pAdv->start()) {
    Serial.printf("[BleDebug] advertising start failed\n");
    return false;
  }

  Serial.printf("[BleDebug] Advertising as \"%s\"\n", setup_.deviceName);
  initSuccess_ = true;
  return true;
}

void BleDebugSubsystem::begin() { Serial.printf("[BleDebug] Task started\n"); }

void BleDebugSubsystem::update() {
  // Drain any RX data to prevent ring buffer overflow.
  while (bleStream_.available()) {
    if (bleStream_.read() < 0) break;
  }
}

// static
void BleDebugSubsystem::writeIfReady(const char* buf) {
  if (!instance_ || !instance_->initSuccess_) return;
  if (!instance_->bleStream_.ready()) return;
  instance_->bleStream_.println(buf);
}

void BleDebugSubsystem::ServerCallbacks::onConnect(NimBLEServer* pServer,
                                                   NimBLEConnInfo& connInfo) {
  Serial.printf("[BleDebug] Client connected, handle=%u\n",
                connInfo.getConnHandle());
}

void BleDebugSubsystem::ServerCallbacks::onDisconnect(NimBLEServer* pServer,
                                                      NimBLEConnInfo& connInfo,
                                                      int reason) {
  Serial.printf("[BleDebug] Client disconnected, handle=%u, reason=%d\n",
                connInfo.getConnHandle(), reason);
  // advertiseOnDisconnect(true) handles advertising restart automatically.
}

}  // namespace Subsystem
