// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modify By Seeed 7.29/2021 
// Content: adapted to Arduino framwork
/*
 * Example Use
 *
    static camera_config_t camera_example_config = {
        .sccb           = &Wire,
        .xclk_freq_hz   = 20000000,
        .pixel_format   = PIXFORMAT_JPEG,
        .frame_size     = FRAMESIZE_SVGA,
        .jpeg_quality   = 10,
        .fb_count       = 2,
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
    };
*/

#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "cam_utils.h"
#include "sensor.h"
#include "sys/time.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration structure for camera initialization
 */
typedef enum {
    CAMERA_GRAB_WHEN_EMPTY,         /*!< Fills buffers when they are empty. Less resources but first 'fb_count' frames might be old */
    CAMERA_GRAB_LATEST              /*!< Except when 1 frame buffer is used, queue will always contain the last 'fb_count' frames */
} camera_grab_mode_t;

/**
 * @brief Configuration structure for camera initialization
 */
typedef struct {

    TwoWire *sccb;

    int xclk_freq_hz;               /*!< Frequency of XCLK signal, in Hz. EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode */

    pixformat_t pixel_format;       /*!< Format of the pixel data: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG  */
    framesize_t frame_size;         /*!< Size of the output image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA  */

    int jpeg_quality;               /*!< Quality of JPEG output. 0-63 lower means higher quality  */
    size_t fb_count;                /*!< Number of frame buffers to be allocated. If more than one, then each frame will be acquired (double speed)  */
    camera_grab_mode_t grab_mode;   /*!< When buffers should be filled */
} camera_config_t;

/**
 * @brief Data structure of camera frame buffer
 */
typedef struct {
    uint8_t * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
    size_t width;               /*!< Width of the buffer in pixels */
    size_t height;              /*!< Height of the buffer in pixels */
    pixformat_t format;         /*!< Format of the pixel data */
    struct timeval timestamp;   /*!< Timestamp since boot of the first DMA buffer of the frame */
} camera_fb_t;

#define ERR_CAMERA_BASE 0x20000
#define ERR_CAMERA_NOT_DETECTED             (ERR_CAMERA_BASE + 1)
#define ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE (ERR_CAMERA_BASE + 2)
#define ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT (ERR_CAMERA_BASE + 3)
#define ERR_CAMERA_NOT_SUPPORTED            (ERR_CAMERA_BASE + 4)

/**
 * @brief Initialize the camera driver
 *
 * @note call camera_probe before calling this function
 *
 * This function detects and configures camera over I2C interface,
 * allocates framebuffer and DMA buffers,
 * initializes parallel I2S input, and sets up DMA descriptors.
 *
 * Currently this function can only be called once and there is
 * no way to de-initialize this module.
 *
 * @param config  Camera configuration parameters
 *
 * @return OK on success
 */
err_t camera_init(const camera_config_t* config);

/**
 * @brief Deinitialize the camera driver
 *
 * @return
 *      - OK on success
 *      - ERR_INVALID_STATE if the driver hasn't been initialized yet
 */
cam_err_t camera_deinit();

/**
 * @brief Obtain pointer to a frame buffer.
 *
 * @return pointer to the frame buffer
 */
camera_fb_t* camera_fb_get();

/**
 * @brief Return the frame buffer to be reused again.
 *
 * @param fb    Pointer to the frame buffer
 */
void camera_fb_return(camera_fb_t * fb);

/**
 * @brief Get a pointer to the image sensor control structure
 *
 * @return pointer to the sensor
 */
sensor_t * camera_sensor_get();

/**
 * @brief Save camera settings to non-volatile-storage (NVS)
 * 
 * @param key   A unique nvs key name for the camera settings 
 */
err_t camera_save_to_nvs(const char *key);

/**
 * @brief Load camera settings from non-volatile-storage (NVS)
 * 
 * @param key   A unique nvs key name for the camera settings 
 */
err_t camera_load_from_nvs(const char *key);

#ifdef __cplusplus
}
#endif

