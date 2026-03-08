/**
 * @file slam-sensors/main.cpp
 * @brief SLAM sensor input firmware — LD14P LiDAR publisher and camera
 *        subscriber running as FreeRTOS tasks over micro-ROS WiFi.
 *
 * Task layout (ESP32 dual-core):
 *   Core 0: MicrorosManager  (priority 5, 16 KB stack)
 *   Core 1: LidarSubsystem   (priority 4,  8 KB stack)
 *   Core 1: CameraSubscriber  (priority 2,  4 KB stack)
 *
 * Run the micro-ROS agent on your network:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *
 * Verify with:
 *   ros2 topic echo /lidar/scan sensor_msgs/msg/LaserScan
 *   ros2 topic info /camera/image/compressed
 */

#include <Arduino.h>
#include <WiFi.h>

#include "CameraSubscriber.h"
#include "LidarSubsystem.h"
#include "microros_manager_robot.h"

// Manager needs 1 executor handle for the camera subscription
Subsystem::MicrorosManagerSetup managerSetup("seeker_slam",
                                             /*executor_handles=*/1);
Subsystem::LidarSubsystemSetup lidarSetup("lidar_ld14p");
Subsystem::CameraSubscriberSetup cameraSetup("camera_sub");

Subsystem::MicrorosManager manager(managerSetup);
Subsystem::LidarSubsystem lidar(lidarSetup);
Subsystem::CameraSubscriber camera(cameraSetup);

void setup() {
  Serial.begin(921600);
  delay(2000);

  Serial.println("=== Seeker Robot — SLAM Sensor Firmware ===");
  {
    uint8_t agent_ip[] = AGENT_IP;
    Serial.printf("Agent: %d.%d.%d.%d:%d\n", agent_ip[0], agent_ip[1],
                  agent_ip[2], agent_ip[3], AGENT_PORT);
  }
  Serial.printf("SSID:  %s\n", WIFI_SSID);

  manager.init();
  lidar.init();
  camera.init();

  // Register both subsystems as micro-ROS participants
  manager.registerParticipant(&lidar);
  manager.registerParticipant(&camera);

  lidar.setManager(&manager);
  camera.setManager(&manager);

  // Launch FreeRTOS tasks
  // Core 0: micro-ROS networking (heavy, 16 KB)
  manager.beginThreaded(16384, 5, 0);
  // Core 1: LiDAR UART parsing + publishing (time-critical, 8 KB)
  lidar.beginThreaded(8192, 4, 1);
  // Core 1: Camera subscription watchdog (lightweight, 4 KB)
  camera.beginThreaded(4096, 2, 1);

  Serial.printf("WiFi status: %d (3=connected)\n", WiFi.status());
  Serial.printf("Local IP:  %s\n", WiFi.localIP().toString().c_str());
  Serial.println("Tasks started:");
  Serial.println("  Core 0: MicrorosManager (pri 5)");
  Serial.println("  Core 1: LidarSubsystem  (pri 4)");
  Serial.println("  Core 1: CameraSubscriber (pri 2)");
}

void loop() {
  // Main loop idle — all work happens in FreeRTOS tasks.
  // Print periodic status for debug visibility.
  static uint64_t last_status_ms = 0;
  uint64_t now = millis();
  if (now - last_status_ms >= 5000) {
    Serial.printf("[STATUS] connected=%d  cam_frames=%lu  cam_rx=%d\n",
                  manager.isConnected(),
                  static_cast<unsigned long>(camera.getFrameCount()),
                  camera.isReceiving() ? 1 : 0);
    last_status_ms = now;
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
}
