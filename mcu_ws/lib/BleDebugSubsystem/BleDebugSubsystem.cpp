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

  msgQueue_ = xQueueCreate(kQueueDepth, sizeof(DebugMsg));
  if (!msgQueue_) {
    Serial.printf("[BleDebug] queue creation failed\n");
    return false;
  }

  Serial.printf("[BleDebug] Advertising as \"%s\"\n", setup_.deviceName);
  initSuccess_ = true;
  return true;
}

void BleDebugSubsystem::begin() { Serial.printf("[BleDebug] Task started\n"); }

void BleDebugSubsystem::update() {
  // Drain the message queue and write to BLE stream if a client is connected.
  // Messages are consumed regardless of ready() so the queue never backs up.
  DebugMsg msg;
  while (xQueueReceive(msgQueue_, &msg, 0) == pdTRUE) {
    if (bleStream_.ready()) {
      // Use printf (single write() call) instead of println (two writes) to
      // avoid a bare \r\n notification appearing as "" on the client.
      bleStream_.printf("%s\n", msg.text);
    }
  }

  // Accumulate RX bytes into line_assembly_; on '\r'/'\n', commit the line
  // into line_ready_ for the main loop to consume via tryGetLine(). Dropping
  // the drain here is safe because completed lines (and oversized lines) both
  // reset the assembly cursor, so the client's ring buffer is still consumed
  // fully even when nobody calls tryGetLine().
  while (bleStream_.available()) {
    int b = bleStream_.read();
    if (b < 0) break;
    char c = static_cast<char>(b);
    if (c == '\r' || c == '\n') {
      if (line_assembly_pos_ > 0) {
        line_assembly_[line_assembly_pos_] = '\0';
        Threads::Scope lock(line_mutex_);
        if (!line_ready_has_) {
          strncpy(line_ready_, line_assembly_, kLineBufSize - 1);
          line_ready_[kLineBufSize - 1] = '\0';
          line_ready_has_ = true;
        }
        // If a line is already pending, drop this one — main loop hasn't
        // caught up. Preferable to blocking the BLE task.
        line_assembly_pos_ = 0;
      }
    } else if (line_assembly_pos_ < kLineBufSize - 1) {
      line_assembly_[line_assembly_pos_++] = c;
    } else {
      // Line too long — reset assembly; the current line is discarded.
      line_assembly_pos_ = 0;
    }
  }
}

// static
void BleDebugSubsystem::writeIfReady(const char* buf) {
  if (!instance_ || !instance_->initSuccess_ || !instance_->msgQueue_) return;
  DebugMsg msg;
  strncpy(msg.text, buf, kMsgLen - 1);
  msg.text[kMsgLen - 1] = '\0';
  // Non-blocking: drop if queue is full rather than stalling the caller.
  xQueueSendToBack(instance_->msgQueue_, &msg, 0);
}

// static
bool BleDebugSubsystem::tryGetLine(char* out, size_t max_len) {
  if (!instance_ || !instance_->initSuccess_ || !out || max_len == 0) {
    return false;
  }
  Threads::Scope lock(instance_->line_mutex_);
  if (!instance_->line_ready_has_) return false;
  strncpy(out, instance_->line_ready_, max_len - 1);
  out[max_len - 1] = '\0';
  instance_->line_ready_has_ = false;
  return true;
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
