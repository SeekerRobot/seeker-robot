#include <Arduino.h>
#include <micro_ros_platformio.h>

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
void printf() {}
#endif

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) ||      \
    defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA) || \
    defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
byte local_mac[] = LOCAL_MAC;
IPAddress local_ip(LOCAL_IP);
IPAddress agent_ip(AGENT_IP);
size_t agent_port = AGENT_PORT;
char ssid[] = WIFI_SSID;
char psk[] = WIFI_PASSWORD;
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
  set_microros_serial_transports(Serial);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
  set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip,
                                          agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) || \
    defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA)
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
  rmw_uros_set_custom_transport(
      MICROROS_TRANSPORTS_FRAMING_MODE, NULL, platformio_transport_open,
      platformio_transport_close, platformio_transport_write,
      platformio_transport_read);
#else
#error "No transport defined"
#endif
}
