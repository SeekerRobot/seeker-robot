// Camera config — pin macros sourced from RobotConfig.h per environment

#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

#include "RobotConfig.h"
#include "esp_camera.h"

static camera_config_t camera_config = {.pin_pwdn = PWDN_GPIO_NUM,
                                        .pin_reset = RESET_GPIO_NUM,
                                        .pin_xclk = XCLK_GPIO_NUM,
                                        .pin_sccb_sda = SIOD_GPIO_NUM,
                                        .pin_sccb_scl = SIOC_GPIO_NUM,

                                        .pin_d7 = Y9_GPIO_NUM,
                                        .pin_d6 = Y8_GPIO_NUM,
                                        .pin_d5 = Y7_GPIO_NUM,
                                        .pin_d4 = Y6_GPIO_NUM,
                                        .pin_d3 = Y5_GPIO_NUM,
                                        .pin_d2 = Y4_GPIO_NUM,
                                        .pin_d1 = Y3_GPIO_NUM,
                                        .pin_d0 = Y2_GPIO_NUM,
                                        .pin_vsync = VSYNC_GPIO_NUM,
                                        .pin_href = HREF_GPIO_NUM,
                                        .pin_pclk = PCLK_GPIO_NUM,

                                        .xclk_freq_hz = 20000000,
                                        .ledc_timer = LEDC_TIMER_0,
                                        .ledc_channel = LEDC_CHANNEL_0,

                                        .pixel_format = PIXFORMAT_JPEG,
                                        .frame_size = FRAMESIZE_SVGA,
                                        .jpeg_quality = 15,
                                        .fb_count = 1,
                                        .fb_location = CAMERA_FB_IN_PSRAM,
                                        .grab_mode = CAMERA_GRAB_WHEN_EMPTY};

#endif
