/**
 * @file test_bridge_gait.cpp
 * @date 4/3/2026
 * @brief Integration test: WiFi + micro-ROS agent + GaitController +
 *        GaitRosParticipant.
 * @author Aldem Pido, Claude Code
 * Disclaimer: this file was written mostly with Claude Code.
 *
 * GaitRosParticipant subscribes to geometry_msgs/Twist on /cmd_vel.
 * Any nonzero velocity command enables the gait; a near-zero command triggers
 * a clean stop. The gait controller runs on its own FreeRTOS task completely
 * independently of the ROS layer — if the agent disconnects mid-walk, the last
 * velocity command is retained until explicitly cleared.
 *
 * ============================================================
 * IMPORTANT — PLACEHOLDER DIMENSIONS
 * ============================================================
 * Leg configs use the same placeholder values as test_sub_gait.
 * Measure L1, L2, mount positions, mount angles, and servo channels
 * from the physical robot before running on hardware.
 * ============================================================
 *
 * Drive the robot from the ROS2 host:
 *
 *   # Keyboard teleop (standard ROS2 package, no install needed):
 *   ros2 run teleop_twist_keyboard teleop_twist_keyboard
 *     i / ,   — forward / backward
 *     j / l   — rotate left / right
 *     k       — stop (sends zero Twist → clean stop)
 *     q / z   — increase / decrease linear speed
 *     w / x   — increase / decrease angular speed
 *
 *   # Joystick (requires joystick device + config yaml):
 *   ros2 launch teleop_twist_joy teleop-launch.py
 *
 *   # One-shot test:
 *   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
 *       '{linear: {x: 0.05}}' --once
 *
 * Expected serial output at ~2 s intervals:
 *   [Loop] wifi=CONNECTED    microros=CONNECTED    ip=192.168.x.x  rssi=-XX dBm
 *
 * Verify the gait state on the micro-ROS agent log when a Twist arrives.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <GaitController.h>
#include <GaitRosParticipant.h>
#include <HexapodConfig.h>
#include <HexapodKinematics.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>
#include <Wire.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

// ---------------------------------------------------------------------------
// Network config — injected from network_config.ini via platformio.ini
// ---------------------------------------------------------------------------
static IPAddress static_ip STATIC_IP;
static IPAddress gateway   GATEWAY;
static IPAddress subnet    SUBNET;

// ---------------------------------------------------------------------------
// Global setup structs (no runtime-resolved pointers — safe as globals)
// ---------------------------------------------------------------------------
static Classes::BaseSetup       blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Threads::Mutex i2c_mutex;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                      "bridge_gait_node");
static Subsystem::MicrorosManager manager(manager_setup);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static const char* wifiStateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED: return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:   return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:    return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING: return "RECONNECTING";
    case Subsystem::WifiState::FAILED:       return "FAILED";
    default:                                 return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(500);

  blink.beginThreadedPinned(2048, 1, 500, 1);

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 2 | 100 ms update | 4096 words
  wifi.beginThreadedPinned(4096, 2, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  // --- Servo configs — from HexapodConfig.h ---
  static Subsystem::ServoConfig servo_configs[HexapodConfig::kNumServos];
  for (uint8_t i = 0; i < HexapodConfig::kNumServos; i++) {
    servo_configs[i] = HexapodConfig::kServoConfigs[i];
  }

  // --- ServoSubsystem ---
  static Subsystem::ServoSetup servo_setup(
      Wire, Config::pca_addr, Config::servo_en,
      servo_configs, HexapodConfig::kNumServos,
      HexapodConfig::kServoBudget, HexapodConfig::kPwmFreqHz);

  auto& servos = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  for (uint8_t i = 0; i < HexapodConfig::kNumServos; i++) servos.attach(i);
  if (!servos.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Servo init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 5 | 10 ms update | 4096 words
  servos.beginThreadedPinned(4096, 5, 10, 1);
  Debug::printf(Debug::Level::INFO, "[Main] ServoSubsystem started");

  // --- HexapodKinematics ---
  static Kinematics::HexapodKinematics kin(HexapodConfig::kLegConfigs);

  // --- GaitController ---
  static Gait::GaitSetup gait_setup(HexapodConfig::kGaitConfig, &kin, &servos);
  static Gait::GaitController gait_ctrl(gait_setup);

  if (!gait_ctrl.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gait init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 4 | 10 ms update | 8192 words
  gait_ctrl.beginThreadedPinned(8192, 4, 10, 1);
  Debug::printf(Debug::Level::INFO, "[Main] GaitController started");

  // --- GaitRosParticipant ---
  // Holds a pointer to gait_ctrl — must be constructed after it.
  static Gait::GaitRosParticipantSetup ros_setup{&gait_ctrl};
  static Gait::GaitRosParticipant gait_ros(ros_setup);

  // --- micro-ROS manager ---
  manager.registerParticipant(&gait_ros);

  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  // Core 1 | priority 3 | 10 ms update | 8192 word stack
  manager.beginThreadedPinned(8192, 3, 10, 1);
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
