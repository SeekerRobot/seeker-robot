/**
 * @file test-microros-wifi-threaded/main.cpp
 * @brief ESP32 micro-ROS WiFi transport tes
 *
 * Note: generated with Claude Code
 *
 * Minimal test that connects to WiFi and a micro-ROS agent over UDP,
 * then publishes a heartbeat string message every second.
 *
 * Before building, update WIFI_SSID, WIFI_PASSWORD, AGENT_IP, and LOCAL_IP
 * in platformio.ini under [esp32_microros_wifi].
 *
 * Run the agent on your network:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *
 * Verify with:
 *   ros2 topic echo /esp32_wifi_test/status std_msgs/msg/String
 */

#include <Arduino.h>
#include <WiFi.h>

#include "ExampleMicrorosSubsystem.h"
#include "microros_manager_robot.h"

Subsystem::MicrorosManagerSetup managerSetup("esp32_wifi_test");
Subsystem::ExampleSubsystemSetup exampleSetup("esp32_wifi_example");
Subsystem::MicrorosManager manager(managerSetup);
Subsystem::ExampleSubsystem example(exampleSetup);

void setup() {
  Serial.begin(921600);
  delay(2000);  // Allow serial monitor to connect

  Serial.println("=== ESP32 micro-ROS WiFi Transport Test ===");
  {
    uint8_t agent_ip[] = AGENT_IP;
    Serial.printf("Agent: %d.%d.%d.%d:%d\n", agent_ip[0], agent_ip[1],
                  agent_ip[2], agent_ip[3], AGENT_PORT);
  }
  Serial.printf("SSID:  %s\n", WIFI_SSID);

  manager.init();
  manager.begin();
  manager.registerParticipant(&example);

  Serial.printf("WiFi status: %d (3=connected)\n", WiFi.status());
  Serial.printf("Local IP:  %s\n", WiFi.localIP().toString().c_str());
  Serial.println("Setup complete. Waiting for agent...");
}

void loop() {
  manager.update();

  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms > 1000) {
    if (manager.isConnected()) {
      Serial.println("[OK] Agent connected - publishing heartbeat");
      example.publishStatus("OK");
    } else {
      Serial.println("[..] Waiting for agent connection...");
    }
    last_ms = now;
  }
}
