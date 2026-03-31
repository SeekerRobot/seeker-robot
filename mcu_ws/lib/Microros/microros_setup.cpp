#include <Arduino.h>
#include <micro_ros_platformio.h>

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
void printf() {}
#endif

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) ||      \
    defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA) || \
    defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
byte local_mac[] = LOCAL_MAC;
IPAddress local_ip(LOCAL_IP);
#endif
IPAddress agent_ip(AGENT_IP);
size_t agent_port = AGENT_PORT;
char ssid[] = WIFI_SSID;
char psk[] = WIFI_PASSWORD;

// Forward-declare WiFi UDP transport functions (defined in
// micro_ros_transport.cpp as extern "C")
extern "C" {
bool platformio_transport_open(struct uxrCustomTransport* transport);
bool platformio_transport_close(struct uxrCustomTransport* transport);
size_t platformio_transport_write(struct uxrCustomTransport* transport,
                                  const uint8_t* buf, size_t len,
                                  uint8_t* err);
size_t platformio_transport_read(struct uxrCustomTransport* transport,
                                 uint8_t* buf, size_t len, int timeout,
                                 uint8_t* err);
}
#endif

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
bool platformio_transport_open(struct uxrCustomTransport* transport) {
  return false;
};
bool platformio_transport_close(struct uxrCustomTransport* transport) {
  return false;
};
size_t platformio_transport_write(struct uxrCustomTransport* transport,
                                  const uint8_t* buf, size_t len,
                                  uint8_t* err) {
  return 0;
};
size_t platformio_transport_read(struct uxrCustomTransport* transport,
                                 uint8_t* buf, size_t len, int timeout,
                                 uint8_t* err) {
  return 0;
};
#endif

extern "C" void set_microros_transports() {
  Serial.begin(921600);

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  // Wait for USB CDC serial to be ready (up to 3s)
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) { delay(10); }

  // Flush any stale data from USB enumeration
  delay(100);
  while (Serial.available()) Serial.read();
#endif

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  set_microros_serial_transports(Serial);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
  set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip,
                                          agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) || \
    defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA)
  // WiFi connection is now managed by ESP32WifiSubsystem (must be connected
  // before this point). Only register the UDP transport callbacks here.
  static struct micro_ros_agent_locator locator;
  locator.address = agent_ip;
  locator.port = agent_port;
  rmw_uros_set_custom_transport(false, (void*)&locator,
                                platformio_transport_open,
                                platformio_transport_close,
                                platformio_transport_write,
                                platformio_transport_read);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
  rmw_uros_set_custom_transport(
      MICROROS_TRANSPORTS_FRAMING_MODE, NULL, platformio_transport_open,
      platformio_transport_close, platformio_transport_write,
      platformio_transport_read);
#else
#error "No transport defined"
#endif
}
