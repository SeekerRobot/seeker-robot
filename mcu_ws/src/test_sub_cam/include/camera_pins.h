#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

#include "esp_camera.h"

// Pins that perform power/reset operations
#define CAM_PIN_PWDN 32   // Power down pin
#define CAM_PIN_RESET -1  // Reset is performed through software

// Pin that drives the camera's internal clock
#define CAM_PIN_XCLK 0

// Pins that send config messages
#define CAM_PIN_SIOD 26  // Data wire used to send the messages
#define CAM_PIN_SIOC 27  // Clock wire used to synchronize message timing

// Mega highway of pins for sending the images
// Each digit of each 8-bit word is moved by each pin, 1 pulse at a time
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5

// Clock wires
#define CAM_PIN_VSYNC 25  // Marks the start of an image
#define CAM_PIN_HREF 23   // On high while a row is being sent
#define CAM_PIN_PCLK 22   // On high when a new word should be read

// Camera config data structure
static camera_config_t camera_config = {
    // Pin assignment
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // Camera's internal clock
    .xclk_freq_hz = 20000000,  // Frequency

    .ledc_timer = LEDC_TIMER_0,      // Internal clock mechanism
    .ledc_channel = LEDC_CHANNEL_0,  // Internal clock output

    // Image configuration
    .pixel_format = PIXFORMAT_JPEG,  // Image format
    .frame_size = FRAMESIZE_SVGA,    // Resolution
    .jpeg_quality = 15,  // Compression level (lower number = higher quality)
    .fb_count = 1,       // Number of frame buffers
    .grab_mode =
        CAMERA_GRAB_WHEN_EMPTY  // Only write into a buffer when it's empty
};

#endif