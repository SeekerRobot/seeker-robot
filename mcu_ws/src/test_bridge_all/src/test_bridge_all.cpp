/**
 * @file test_bridge_all.cpp
 * @author Aldem Pido
 * @date 4/2/2026
 * @brief Integration test: WiFi + MicrorosManager + MicroRosBridge with all
 *        subsystems enabled (heartbeat, gyro, battery, lidar).
 *
 * Each subsystem runs on its own FreeRTOS task with no ROS knowledge.
 * MicroRosBridge is the sole IMicroRosParticipant — it reads subsystem data
 * via thread-safe getters and publishes at the configured rates once the
 * micro-ROS agent is reachable.
 *
 * Topics published once agent connects:
 *   mcu/heartbeat        std_msgs/Int32        1 Hz
 *   mcu/imu              sensor_msgs/Imu      50 Hz
 *   mcu/battery_voltage  std_msgs/Float32      1 Hz
 *   mcu/scan             sensor_msgs/LaserScan ~6 Hz (per completed scan)
 *
 * Verify on the host:
 *   ros2 topic hz /mcu/imu             # ~50 Hz
 *   ros2 topic hz /mcu/scan            # ~6 Hz
 *   ros2 topic echo /mcu/battery_voltage
 *   ros2 topic echo /mcu/scan
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <GyroSubsystem.h>
#include <LidarSubsystem.h>
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
// Globals safe to construct at static-init time (no runtime dependencies)
// ---------------------------------------------------------------------------
static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Threads::Mutex i2c_mutex;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);

static constexpr Subsystem::BatteryCalibration kBattCal(
    /*raw_lo=*/1862, /*volt_lo=*/3.0f,
    /*raw_hi=*/2480, /*volt_hi=*/4.2f);
static Subsystem::BatterySetup battery_setup(Config::batt, kBattCal,
                                             /*num_samples=*/16);

// LidarSetup holds a HardwareSerial& — Serial2 is a global object, safe here.
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "bridge_all_node");
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

  blink.beginThreadedPinned(2048, 1, 500, 1);

  // --- WiFi --- core 1 | priority 3 | 100 ms | 4096 words
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  // --- Gyro --- core 1 | priority 5 | 0 ms (semaphore-blocked) | 4096 words
  // GyroSubsystem::init() calls Wire.begin() and Wire.setClock() internally.
  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  if (!gyro.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gyro init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  gyro.beginThreadedPinned(4096, 5, 0, 1);
  Debug::printf(Debug::Level::INFO, "[Main] Gyro started");

  // --- Battery --- core 1 | priority 2 | 100 ms | 4096 words
  auto& battery = Subsystem::BatterySubsystem::getInstance(battery_setup);
  if (!battery.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Battery init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  battery.beginThreadedPinned(4096, 2, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] Battery started");

  // --- Lidar --- core 0 | priority 4 | 1 ms | 6144 words
  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Lidar init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  lidar.beginThreadedPinned(6144, 4, 1, 0);
  Debug::printf(Debug::Level::INFO, "[Main] Lidar started");

  // --- Bridge ---
  // MicroRosBridgeSetup holds subsystem pointers, so it must be constructed
  // after getInstance() calls — not at global-static init time.
  // Declared static so it outlives setup().
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = &gyro;
  bridge_setup.battery = &battery;
  bridge_setup.lidar = &lidar;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  // --- micro-ROS manager --- core 1 | priority 4 | 10 ms | 8192 words
  manager.registerParticipant(&bridge);
  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
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
