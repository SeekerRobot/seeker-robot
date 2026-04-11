/**
 * @file test_bridge_oled.cpp
 * @author Claude Code
 * @date 4/10/2026
 * @brief Integration test: WiFi + micro-ROS + OLED bridge subscriber.
 *
 * Subscribes to /mcu/lcd (mcu_msgs/OledFrame) and streams the raw 1024-byte
 * framebuffer to the SSD1306 128x64 display at a hard 10 Hz cap. This sketch
 * isolates the OLED bridge feature — no gyro, battery, lidar, etc.
 *
 * Verify on host:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *
 *   # Python publisher — fills the display with a sine-wave pattern:
 *   python3 -c "
 *   import rclpy, math, time
 *   from rclpy.node import Node
 *   from mcu_msgs.msg import OledFrame
 *   rclpy.init(); n = Node('oled_test')
 *   p = n.create_publisher(OledFrame, '/mcu/lcd', 10)
 *   t = 0.0
 *   while rclpy.ok():
 *       fb = bytearray(1024)
 *       for x in range(128):
 *           y = int(32 + 30*math.sin(x*0.1 + t))
 *           fb[x + (y//8)*128] = 1 << (y%8)
 *       m = OledFrame(); m.framebuffer = list(fb)
 *       p.publish(m); t += 0.1; time.sleep(0.1)
 *   "
 *
 *   # One-shot all-pixels-on test:
 *   ros2 topic pub -1 /mcu/lcd mcu_msgs/msg/OledFrame \
 *       "{framebuffer: [$(python3 -c 'print(",".join(["255"]*1024))')]}"
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <MicroRosBridge.h>
#include <OledSubsystem.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Threads::Mutex i2c_mutex;

static Subsystem::OledSetup oled_setup(i2c_mutex);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "bridge_oled_node");
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

  // --- I2C bus for OLED (no other I2C devices in this sketch) ---
  Wire.begin(Config::sda, Config::scl);
  Wire.setClock(400000);

  // --- OLED display ---
  auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);
  if (!oled.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] OLED init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // Core 1 | priority 2 | 100 ms update (10 Hz)
  oled.beginThreadedPinned(4096, 2, 100, 1);
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "bridge_oled test");
  oled.setOverlay(1, 4, 52, "waiting agent...");
  Debug::printf(Debug::Level::INFO, "[Main] OLED started");

  // --- micro-ROS bridge (OLED subscriber only) ---
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.oled = &oled;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  manager.registerParticipant(&bridge);
  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    auto& o = Subsystem::OledSubsystem::getInstance(oled_setup);
    if (connected) {
      o.setOverlay(1, 4, 52, "ros2 connected ");
    } else {
      o.setOverlay(1, 4, 52, "disconnected   ");
    }
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
