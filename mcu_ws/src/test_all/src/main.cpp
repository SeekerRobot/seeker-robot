/**
 * @file test_all/src/main.cpp
 * @author Tal Avital
 * @date 4/7/2026
 * @brief Full integration test: micro-ROS bridge + MJPEG camera stream + PDM
 *        audio stream, all running concurrently on the Seeed XIAO ESP32-S3
 *        Sense.
 *
 *   GET http://<static_ip>:80/cam    — MJPEG video stream
 *   GET http://<static_ip>:81/audio  — raw 16-bit PCM, 16kHz, mono
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
#include <CameraSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <GyroSubsystem.h>
#include <LidarSubsystem.h>
#include <MicSubsystem.h>
#include <MicroRosBridge.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <camera_pins.h>
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
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);
static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "test_all_node");
static Subsystem::MicrorosManager manager(manager_setup);

static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80);
static Subsystem::MicSetup mic_setup(I2S_NUM_0,
                                     /*sample_rate=*/16000,
                                     /*clk_pin=*/Config::pdm_clk,
                                     /*data_pin=*/Config::pdm_data,
                                     /*gain=*/4,
                                     /*http_port=*/81);

void setup() {
  Serial.begin(921600);
  delay(500);

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
  batt.beginThreadedPinned(4096, 2, 50, 1);

  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Lidar init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  lidar.beginThreadedPinned(6144, 4, 1, 0);

  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  cam.beginThreadedPinned(8192, 2, 5000, 1);

  // audioStreamTask is pinned to Core 0 internally — keeps I2S reads off
  // Core 1 where camera_task (from esp_camera_init) runs.
  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  mic.beginThreadedPinned(4096, 2, 5000, 0);

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
