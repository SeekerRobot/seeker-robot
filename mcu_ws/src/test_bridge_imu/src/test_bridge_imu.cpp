/**
 * @file test_bridge_imu.cpp
 * @author Aldem Pido
 * @date 3/31/2026
 * @brief Integration test: WiFi + MicrorosManager + MicroRosBridge (IMU
 * publisher).
 *
 * Demonstrates the bridge pattern: GyroSubsystem runs independently on its own
 * FreeRTOS task (interrupt-driven, no ROS knowledge). MicroRosBridge is the
 * sole IMicroRosParticipant — it reads from the gyro via getImuData() and
 * publishes sensor_msgs/Imu at 50 Hz once the micro-ROS agent is reachable.
 *
 * Build flags required (set in this sketch's platformio.ini):
 *   -D BRIDGE_ENABLE_GYRO=1
 *
 * Expected Serial output at ~2 s intervals:
 *   [Loop] wifi=CONNECTED    microros=CONNECTED    ip=192.168.x.x  rssi=-XX dBm
 *
 * Verify on the host:
 *   ros2 topic echo /mcu/imu
 *   ros2 topic hz   /mcu/imu    # should report ~50 Hz
 */
#include <Arduino.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <GyroSubsystem.h>
#include <MicroRosBridge.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

// ---------------------------------------------------------------------------
// Network config — injected by platformio.ini from network_config.ini
// ---------------------------------------------------------------------------
static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

// ---------------------------------------------------------------------------
// Shared I2C mutex (gyro uses the same bus as any future I2C peripherals)
// ---------------------------------------------------------------------------
static Threads::Mutex i2c_mutex;

// ---------------------------------------------------------------------------
// Subsystem setup structs (no runtime dependencies — safe as globals)
// ---------------------------------------------------------------------------
static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "bridge_imu_node");

// MicrorosManager has no runtime-resolved pointers in its setup — safe as
// global.
static Subsystem::MicrorosManager manager(manager_setup);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static const char* wifiStateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED:
      return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:
      return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:
      return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING:
      return "RECONNECTING";
    case Subsystem::WifiState::FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(500);

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 3 | 100 ms update | 4096 words
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  // --- Gyro ---
  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  if (!gyro.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gyro init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 5 | 0 ms update (blocks on interrupt semaphore) | 4096 words
  gyro.beginThreadedPinned(4096, 5, 0, 1);
  Debug::printf(Debug::Level::INFO, "[Main] Gyro started");

  // --- Bridge ---
  // MicroRosBridgeSetup holds a pointer to the gyro singleton, so it must be
  // constructed here — after getInstance() — not at global-static init time.
  // The local statics below live for the duration of the program.
  // Note: fields not set here use their defaults from MicroRosBridgeSetup
  // (imu_topic = "mcu/imu", imu_interval_ms = 20).
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = &gyro;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  // --- micro-ROS manager ---
  manager.registerParticipant(&bridge);

  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  // Core 1 | priority 4 | 10 ms update | 8192 word stack
  manager.beginThreadedPinned(8192, 4, 10, 1);
  Debug::printf(Debug::Level::INFO, "[Main] micro-ROS manager started");
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);

  if (wifi.isConnected()) {
    Debug::printf(Debug::Level::INFO,
                  "[Loop] wifi=%-12s  microros=%-18s  ip=%-15s  rssi=%d dBm",
                  wifiStateStr(wifi.getState()), manager.getStateStr(),
                  wifi.getLocalIP().toString().c_str(), wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%-12s  microros=%s",
                  wifiStateStr(wifi.getState()), manager.getStateStr());
  }

  delay(2000);
}
