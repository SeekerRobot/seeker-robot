/**
 * @file BleDebugSubsystem.h
 * @author Aldem Pido
 * @date 4/1/26
 * @brief BLE Nordic UART debug transport subsystem.
 * NOTE: Must NOT include CustomDebug.h — CustomDebug.h includes this file.
 * Use Serial.printf for internal diagnostics only.
 */
#pragma once

#include <NimBLEDevice.h>
#include <ThreadedSubsystem.h>

namespace Subsystem {

class BleDebugSetup : public Classes::BaseSetup {
 public:
  BleDebugSetup() = delete;
  /// @param deviceName BLE advertising name visible to scanning devices.
  explicit BleDebugSetup(const char* deviceName)
      : Classes::BaseSetup("BleDebugSubsystem"), deviceName(deviceName) {}
  const char* deviceName;
};

class BleDebugSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  // Nordic UART Service UUIDs (NUS-compatible)
  static constexpr const char* kServiceUUID =
      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
  static constexpr const char* kCharUUID =
      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
  static constexpr uint32_t kTxBufSize = 1024;
  static constexpr uint32_t kRxBufSize = 256;

  BleDebugSubsystem(const BleDebugSubsystem&) = delete;
  BleDebugSubsystem& operator=(const BleDebugSubsystem&) = delete;

  static BleDebugSubsystem& getInstance(const BleDebugSetup& setup) {
    static BleDebugSubsystem instance(setup);
    return instance;
  }

  bool        init() override;
  void        begin() override;
  void        update() override;
  void        pause() override {}
  void        reset() override {}
  const char* getInfo() override { return setup_.getId(); }

  /// @brief Write buf + newline over BLE if a client is subscribed.
  /// Called by CustomDebug. Non-blocking; drops silently if not ready.
  static void writeIfReady(const char* buf);

 private:
  explicit BleDebugSubsystem(const BleDebugSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {
    instance_ = this;
  }

  const BleDebugSetup setup_;
  NimBLEStreamServer  bleStream_;

  static BleDebugSubsystem* instance_;

  class ServerCallbacks : public NimBLEServerCallbacks {
   public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo,
                      int reason) override;
  };
  ServerCallbacks serverCallbacks_;
};

}  // namespace Subsystem
