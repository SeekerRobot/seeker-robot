/**
 * @file test_bridge_all.cpp
 * @author Aldem Pido
 * @date 4/3/2026
 * @brief Integration test: serial micro-ROS + MicroRosBridge with all features
 * enabled (heartbeat, gyro, battery, debug log).
 *
 * Serial transport: the USB serial port is owned by micro-ROS. Debug output
 * before the agent connects is routed through DEBUG_TRANSPORT_SERIAL; once
 * connected all Debug::printf calls are also forwarded to /mcu/log via
 * DEBUG_TRANSPORT_MICROROS.
 *
 * Build flags (set in this sketch's platformio.ini):
 *   -DDEBUG_TRANSPORT_SERIAL
 *   -DDEBUG_TRANSPORT_MICROROS
 *   -DBRIDGE_ENABLE_HEARTBEAT=1
 *   -DBRIDGE_ENABLE_GYRO=1
 *   -DBRIDGE_ENABLE_BATTERY=1
 *   -DBRIDGE_ENABLE_DEBUG=1
 *
 * Verify on host:
 *   ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 921600
 *   ros2 topic echo /mcu/heartbeat
 *   ros2 topic hz   /mcu/imu        # ~50 Hz
 *   ros2 topic echo /mcu/battery_voltage
 *   ros2 topic echo /mcu/log
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <GyroSubsystem.h>
#include <MicroRosBridge.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

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

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "bridge_all_node");
static Subsystem::MicrorosManager manager(manager_setup);

void setup() {
  blink.beginThreadedPinned(2048, 1, 500, 1);
  delay(1000); // Let things stabilize before starting anything

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

  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = &gyro;
  bridge_setup.battery = &batt;
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
  Debug::printf(Debug::Level::INFO, "[Loop] microros=%s",
                manager.getStateStr());
  delay(2000);
}
