/**
 * @file test_bridge_all.cpp
 * @author Aldem Pido
 * @date 4/3/2026
 * @brief Integration test: WiFi micro-ROS + MicroRosBridge with all features
 * enabled (heartbeat, gyro, battery, lidar, debug log).
 *
 * WiFi transport: ESP32WifiSubsystem brings up the network connection before
 * the micro-ROS manager starts. All Debug::printf output is forwarded to
 * /mcu/log via DEBUG_TRANSPORT_MICROROS once the agent is connected.
 * Serial is free for pre-connect debug output (DEBUG_TRANSPORT_SERIAL).
 *
 * Build flags (set in this sketch's platformio.ini):
 *   -DDEBUG_TRANSPORT_SERIAL
 *   -DDEBUG_TRANSPORT_MICROROS
 *   -DBRIDGE_ENABLE_HEARTBEAT=1
 *   -DBRIDGE_ENABLE_GYRO=1
 *   -DBRIDGE_ENABLE_BATTERY=1
 *   -DBRIDGE_ENABLE_LIDAR=1
 *   -DBRIDGE_ENABLE_DEBUG=1
 *
 * Verify on host:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *   ros2 topic echo /mcu/heartbeat
 *   ros2 topic hz   /mcu/imu        # ~50 Hz
 *   ros2 topic echo /mcu/battery_voltage
 *   ros2 topic hz   /mcu/scan       # ~6 Hz
 *   ros2 topic echo /mcu/log
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

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static constexpr Subsystem::BatteryCalibration kBattCalibration(
    /*raw_lo=*/1862, /*volt_lo=*/3.0f,
    /*raw_hi=*/2480, /*volt_hi=*/4.2f);

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Threads::Mutex i2c_mutex;

static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);
static Subsystem::BatterySetup battery_setup(Config::batt, kBattCalibration,
                                             /*num_samples=*/16);

// LidarSetup holds a HardwareSerial& — Serial2 is a global object, safe here.
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "bridge_all_node");
static Subsystem::MicrorosManager manager(manager_setup);

void setup() {
  Serial.begin(921600);
  delay(500);

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  blink.beginThreadedPinned(2048, 1, 500, 1);
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  if (!gyro.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gyro init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  gyro.beginThreadedPinned(4096, 5, 0, 1);

  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  if (!batt.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Battery init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  batt.beginThreadedPinned(2048, 2, 50, 1);

  // --- Lidar --- core 0 | priority 4 | 1 ms | 6144 words
  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Lidar init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  lidar.beginThreadedPinned(6144, 4, 1, 0);
  Debug::printf(Debug::Level::INFO, "[Main] Lidar started");

  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = &gyro;
  bridge_setup.battery = &batt;
  bridge_setup.lidar = &lidar;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

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
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);

  if (wifi.isConnected()) {
    Debug::printf(
        Debug::Level::INFO,
        "[Loop] wifi=CONNECTED  microros=%-18s  ip=%-15s  rssi=%d dBm",
        manager.getStateStr(), wifi.getLocalIP().toString().c_str(),
        wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%-12s  microros=%s",
                  wifi.getState() == Subsystem::WifiState::CONNECTING
                      ? "CONNECTING"
                      : "DISCONNECTED",
                  manager.getStateStr());
  }
  delay(2000);
}
