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
constexpr static int spk_bclk = D8;   // I2S1 bit clock (SPI SCK)
constexpr static int spk_lrclk = D9;  // I2S1 word select (SPI MISO)
constexpr static int spk_dout = D10;  // I2S1 data out (SPI MOSI)
constexpr static int gyro_addr = 0x4B;
constexpr static uint8_t pca_addr = 0x40;
constexpr static uint8_t oled_addr = 0x3C;
// Onboard PDM microphone (MSM261D3526H1CPM)
constexpr static int pdm_clk = 42;   // PDM clock  (I2S ws_io_num)
constexpr static int pdm_data = 41;  // PDM data   (I2S data_in_num)
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
constexpr static uint8_t oled_addr = 0x3C;
#endif

#ifdef ENV_ESP32CAM
// AI-Thinker ESP32-CAM — camera occupies most pins (see camera macro block
// below). Gyro uses the SD-slot pins, which only works if the microSD slot is
// unpopulated. ENABLE_MIC must stay 0 — no PDM hardware on this board.
constexpr static int sda = 15;          // BNO085 I2C SDA (SD_CMD when SD used)
constexpr static int scl = 14;          // BNO085 I2C SCL (SD_CLK when SD used)
constexpr static int gyro_int = 13;     // BNO085 INT (SD_DAT3 when SD used)
constexpr static int gyro_addr = 0x4B;  // BNO085 dev board has ADDR pulled high
#endif

// ---------------------------------------------------------------------------
// PCA9685 servo port mapping
//
// The PCB labels servo headers M1–M13. Each label corresponds to a fixed
// PCA9685 channel. Two channels (7, 14, 15) have no header and are marked x.
//
// Use mPort(N) wherever a PCA9685 channel number is required — it converts
// the silkscreen label to the underlying channel at compile time.
//
//   M port  →  PCA9685 channel
//   ──────     ────────────────
//   M1      →  6
//   M2      →  5
//   M3      →  4
//   M4      →  3
//   M5      →  2
//   M6      →  1
//   M7      →  0
//   M8      →  13
//   M9      →  12
//   M10     →  11
//   M11     →  10
//   M12     →  9
//   M13     →  8
//   (x)     →  7, 14, 15  — no header, do not use
// ---------------------------------------------------------------------------

constexpr uint8_t kMPortToChannel[14] = {
    255,  // [0]  — no M0
    6,    // [1]  M1
    5,    // [2]  M2
    4,    // [3]  M3
    3,    // [4]  M4
    2,    // [5]  M5
    1,    // [6]  M6
    0,    // [7]  M7
    13,   // [8]  M8
    12,   // [9]  M9
    11,   // [10] M10
    10,   // [11] M11
    9,    // [12] M12
    8,    // [13] M13
};

/// @brief Returns the PCA9685 channel for the servo header labelled M<port>.
///        Valid range: 1–13. Passing 0 or >13 returns 255 (invalid).
constexpr uint8_t mPort(uint8_t port) {
  return (port >= 1 && port <= 13) ? kMPortToChannel[port] : 255;
}

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
