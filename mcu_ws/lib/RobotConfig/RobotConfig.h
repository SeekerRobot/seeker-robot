/**
 * @file RobotConfig.h
 * @author Aldem Pido
 * @date 3/26/26
 * @brief Holds pin definitions for different environments, as well as other
 * things
 */
#pragma once
#include <stdint.h>

namespace Config {

#ifdef ENV_ESP32S3SENSE
constexpr static int servo_en = D0;
constexpr static int gyro_int = D1;
constexpr static int rgb_data = D2;
constexpr static int batt = D3;
constexpr static int sda = D4;
constexpr static int scl = D5;
constexpr static int tx = D6;
constexpr static int rx = D7;
// D8 - SCK NC
// D9 - MISO NC
// D10 - MOSI NC
constexpr static int gyro_addr = 0x4A;
constexpr static uint8_t pca_addr = 0x40;
#endif

#ifdef ENV_ESP32DEV
constexpr static int servo_en = 13;
constexpr static int gyro_int = 14;
constexpr static int rgb_data = 27;
constexpr static int batt = 34;  // ADC1, input-only
constexpr static int sda = 21;   // default I2C SDA
constexpr static int scl = 22;   // default I2C SCL
constexpr static int tx = 17;    // UART2 TX
constexpr static int rx = 16;    // UART2 RX
constexpr static int gyro_addr = 0x4B;
constexpr static uint8_t pca_addr = 0x40;
#endif

};  // namespace Config

// Camera pin macros follow the standard esp32-camera naming convention so
// camera_config_t can be initialized with familiar symbols. Macros are placed
// outside the Config namespace because preprocessor definitions are
// namespace-agnostic.

#ifdef ENV_ESP32S3SENSE
// OV2640 on Seeed XIAO ESP32S3 Sense
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13
#endif

#ifdef ENV_ESP32CAM
// OV2640 on AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#endif
