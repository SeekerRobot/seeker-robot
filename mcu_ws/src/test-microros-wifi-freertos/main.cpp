/**
 * @file test-microros-wifi-freertos/main.cpp
 * @brief ESP32 micro-ROS WiFi test using FreeRTOS tasks
 *
 * Same functionality as test-microros-wifi but runs micro-ROS via
 * beginThreaded() on a FreeRTOS task, leaving the main loop free
 * (e.g. for camera streaming on core 1).
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
  delay(2000);

  Serial.println("=== ESP32 micro-ROS WiFi FreeRTOS Test ===");
  {
    uint8_t agent_ip[] = AGENT_IP;
    Serial.printf("Agent: %d.%d.%d.%d:%d\n",
                  agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3],
                  AGENT_PORT);
  }
  Serial.printf("SSID:  %s\n", WIFI_SSID);

  manager.init();
  manager.registerParticipant(&example);
  example.setManager(&manager);

  // Run micro-ROS on core 0, example subsystem on core 1
  manager.beginThreaded(16384, 5, 0);
  example.beginThreaded(4096, 3, 1);

  Serial.printf("WiFi status: %d (3=connected)\n", WiFi.status());
  Serial.printf("Local IP:  %s\n", WiFi.localIP().toString().c_str());
  Serial.println("micro-ROS task on core 0, example subsystem on core 1.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
