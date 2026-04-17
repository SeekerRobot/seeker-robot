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
#include <freertos/queue.h>
#include <hal_thread.h>

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
  static constexpr uint8_t kQueueDepth = 16;
  static constexpr uint16_t kMsgLen = 256;

  BleDebugSubsystem(const BleDebugSubsystem&) = delete;
  BleDebugSubsystem& operator=(const BleDebugSubsystem&) = delete;

  static BleDebugSubsystem& getInstance(const BleDebugSetup& setup) {
    static BleDebugSubsystem instance(setup);
    return instance;
  }

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override {}
  void reset() override {}
  const char* getInfo() override { return setup_.getId(); }

  /// @brief Enqueue buf for BLE transmission. Non-blocking; drops if queue
  /// full. Actual write happens in update() on the BLE task.
  static void writeIfReady(const char* buf);

  /// @brief Retrieve the next complete line received over BLE RX. Lines are
  /// assembled in update() on the BLE task, split on '\r' or '\n'. Returns
  /// false if no line is ready. On success, copies the NUL-terminated line
  /// (without the terminator) into out and clears the slot.
  static bool tryGetLine(char* out, size_t max_len);

  static constexpr size_t kLineBufSize = 128;

 private:
  explicit BleDebugSubsystem(const BleDebugSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {
    instance_ = this;
  }

  const BleDebugSetup setup_;
  NimBLEStreamServer bleStream_;
  QueueHandle_t msgQueue_ = nullptr;

  // Line-assembly scratch — only touched by the BLE task (in update()).
  char line_assembly_[kLineBufSize] = {};
  size_t line_assembly_pos_ = 0;

  // Ready-slot — a single completed line awaiting consumption by the main
  // loop. Protected by line_mutex_ because producer (BLE task) and consumer
  // (main loop) run on different cores.
  char line_ready_[kLineBufSize] = {};
  bool line_ready_has_ = false;
  Threads::Mutex line_mutex_;

  static BleDebugSubsystem* instance_;

  struct DebugMsg {
    char text[kMsgLen];
  };

  class ServerCallbacks : public NimBLEServerCallbacks {
   public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo,
                      int reason) override;
  };
  ServerCallbacks serverCallbacks_;
};

}  // namespace Subsystem
