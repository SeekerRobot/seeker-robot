/**
 * @file test_bridge_all.cpp
 * @author Aldem Pido
 * @date 4/3/2026
 * @brief Integration test: WiFi micro-ROS + MicroRosBridge with all features
 * enabled (heartbeat, gyro, battery, lidar, debug log) + I2S speaker + OLED.
 *
 * WiFi transport: ESP32WifiSubsystem brings up the network connection before
 * the micro-ROS manager starts. All Debug::printf output is forwarded to
 * /mcu/log via DEBUG_TRANSPORT_MICROROS once the agent is connected.
 * Serial is free for pre-connect debug output (DEBUG_TRANSPORT_SERIAL).
 *
 * OLED display fetches 1024-byte framebuffers over HTTP from the host
 * (AGENT_IP:8390/lcd_out). micro-ROS agent is NOT required for OLED.
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
 *
 * OLED (no micro_ros_agent needed):
 *   ros2 run seeker_display oled_sine_node
 *   # or: ros2 run seeker_media mp4_player_node
 *
 * Speaker / TTS (seeker_tts node on host, port 8383):
 *   FISH_API_KEY=xxx ros2 run seeker_tts tts_node
 *   ros2 topic pub /audio_tts_input std_msgs/String "data: 'hello'" --once
 *   ros2 topic pub /audio_play_file std_msgs/String "data:
 * '/path/to/sound.wav'" --once
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <GyroSubsystem.h>
#include <LidarSubsystem.h>
#include <MicroRosBridge.h>
#include <OledSubsystem.h>
#include <RobotConfig.h>
#include <SpeakerSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;
static IPAddress agent_ip(AGENT_IP);

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
static Subsystem::OledSetup oled_setup(i2c_mutex, agent_ip, 8390);

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
  batt.beginThreadedPinned(4096, 2, 50, 1);

  // --- Lidar --- core 0 | priority 4 | 1 ms | 6144 words
  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Lidar init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  lidar.beginThreadedPinned(6144, 4, 1, 0);
  Debug::printf(Debug::Level::INFO, "[Main] Lidar started");

  // --- Speaker — fetches PCM from seeker_tts on host (AGENT_IP:8383) ---
  // Uses I2S_NUM_1, pinned to Core 1. update() retries gracefully until WiFi
  // is up and the TTS server is running.
  static Subsystem::SpeakerSetup speaker_setup(
      I2S_NUM_1, 16000, Config::spk_bclk, Config::spk_lrclk, Config::spk_dout,
      agent_ip, 8383, 2048);
  auto& spk = Subsystem::SpeakerSubsystem::getInstance(speaker_setup);
  if (!spk.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Speaker init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  spk.beginThreadedPinned(8192, 2, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] Speaker started");

  // --- OLED display --- core 1 | priority 2 | 100 ms (10 Hz)
  auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);
  if (!oled.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] OLED init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  oled.beginThreadedPinned(4096, 2, 100, 1);
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "bridge_all");
  oled.setOverlay(1, 4, 52, "connecting...");
  Debug::printf(Debug::Level::INFO, "[Main] OLED started");

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
