/**
 * @file RobotConfig.h
 * @author Aldem Pido
 * @date 3/26/26
 * @brief Holds pin definitions for different environments, as well as other things
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
    constexpr static int gyro_addr = 0x4B;
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

};