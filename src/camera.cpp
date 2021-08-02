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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "sensor.h"
#include "sccb.h"
#include "camera.h"
#include "ll_cam.h"

#define TAG "camera"
#include "cam_log.h"

#define CONFIG_OV2640_SUPPORT 1

#if CONFIG_OV2640_SUPPORT
#include "sensors/ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
#include "ov7725.h"
#endif
#if CONFIG_OV3660_SUPPORT
#include "ov3660.h"
#endif
#if CONFIG_OV5640_SUPPORT
#include "ov5640.h"
#endif
#if CONFIG_NT99141_SUPPORT
#include "nt99141.h"
#endif
#if CONFIG_OV7670_SUPPORT
#include "ov7670.h"
#endif
#if CONFIG_GC2145_SUPPORT
#include "gc2145.h"
#endif
#if CONFIG_GC032A_SUPPORT
#include "gc032a.h"
#endif
#if CONFIG_GC0308_SUPPORT
#include "gc0308.h"
#endif

#define FB_GET_TIMEOUT 4000

typedef struct {
    sensor_t sensor;
    camera_fb_t fb;
} camera_state_t;

static camera_state_t *s_state = NULL;

typedef struct {
    int (*detect)(sensor_t *sensor, sensor_id_t *id);
    int (*init)(sensor_t *sensor);
} sensor_func_t;

static const sensor_func_t g_sensors[] = {
#if CONFIG_OV7725_SUPPORT
    {ov7725_detect, ov7725_init},
#endif
#if CONFIG_OV7670_SUPPORT
    {ov7670_detect, ov7670_init},
#endif
#if CONFIG_OV2640_SUPPORT
    {ov2640_detect, ov2640_init},
#endif
#if CONFIG_OV3660_SUPPORT
    {ov3660_detect, ov3660_init},
#endif
#if CONFIG_OV5640_SUPPORT
    {ov5640_detect, ov5640_init},
#endif
#if CONFIG_NT99141_SUPPORT
    {nt99141_detect, nt99141_init},
#endif
#if CONFIG_GC2145_SUPPORT
    {gc2145_detect, gc2145_init},
#endif
#if CONFIG_GC032A_SUPPORT
    {gc032a_detect, gc032a_init},
#endif
#if CONFIG_GC0308_SUPPORT
    {gc0308_detect, gc0308_init},
#endif
};

static cam_obj_t *cam_obj = NULL;

static const uint32_t JPEG_SOI_MARKER = 0xFFD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;  // written in little-endian for esp32

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    uint32_t sig = *((uint32_t *)inbuf) & 0xFFFFFF;
    if(sig != JPEG_SOI_MARKER) {
        for (uint32_t i = 0; i < length; i++) {
            sig = *((uint32_t *)(&inbuf[i])) & 0xFFFFFF;
            if (sig == JPEG_SOI_MARKER) {
                CAM_LOGW(TAG, "SOI: %d", i);
                return i;
            }
        }
        CAM_LOGW(TAG, "NO-SOI");
        return -1;
    }
    return 0;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length)
{
    int offset = -1;
    uint8_t *dptr = (uint8_t *)inbuf + length - 2;
    while (dptr > inbuf) {
        uint16_t sig = *((uint16_t *)dptr);
        if (JPEG_EOI_MARKER == sig) {
            offset = dptr - inbuf;
            //CAM_LOGW(TAG, "EOI: %d", length - (offset + 2));
            return offset;
        }
        dptr--;
    }
    return -1;
}

static bool cam_get_next_frame(int * frame_pos)
{
    if(!cam_obj->frames[*frame_pos].en){
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}


// static bool cam_start_frame(int * frame_pos)
// {
//     if (cam_get_next_frame(frame_pos)) {
//         if(ll_cam_start(cam_obj, *frame_pos)){
//             uint64_t us = (uint64_t)millis();
//             cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000UL;
//             cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000UL;
//             return true;
//         }
//     }
//     return false;
// }


cam_err_t cam_init(const camera_config_t *config)
{

    cam_err_t ret = CAM_OK;
    cam_obj = (cam_obj_t *)malloc(sizeof(cam_obj_t));

    ret = ll_cam_config(cam_obj, config);

    CAM_LOGI("cam init ok");

    return CAM_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return CAM_FAIL;
}


cam_err_t cam_deinit(void)
{
    if (!cam_obj) {
        return CAM_FAIL;
    }

    //cam_stop();

    ll_cam_deinit(cam_obj);

    free(cam_obj);
    cam_obj = NULL;
    return CAM_OK;
}

static cam_err_t camera_probe(const camera_config_t *config, camera_model_t *out_camera_model)
{
    *out_camera_model = CAMERA_NONE;

    if (s_state != NULL) {
        return CAM_ERR_INVALID_STATE;
    }
    

    s_state = (camera_state_t *) calloc(sizeof(camera_state_t), 1);
    if (!s_state) {
        return CAM_ERR_INVALID_STATE;
    }

    if (config->pin_xclk >= 0) {
        CAM_LOGD("Enabling XCLK output");
        camera_enable_out_clock(config->pin_xclk, config->xclk_freq_hz);
    }

    CAM_LOGD("Searching for camera address");
    delay(10);

    SCCB_Init(config->sccb);

    uint8_t slv_addr = SCCB_Probe(config->sccb);

    if (slv_addr == 0) {
        camera_disable_out_clock(config->pin_xclk);
        return CAM_ERR_NOT_FOUND;
    }

    CAM_LOGI("Detected camera at address=0x%02x", slv_addr);

    s_state->sensor.sccb = config->sccb;
    s_state->sensor.slv_addr = slv_addr;
    s_state->sensor.xclk_freq_hz = config->xclk_freq_hz;

    /**
     * Read sensor ID and then initialize sensor
     * Attention: Some sensors have the same SCCB address. Therefore, several attempts may be made in the detection process
     */
    sensor_id_t *id = &s_state->sensor.id;
    for (size_t i = 0; i < sizeof(g_sensors) / sizeof(sensor_func_t); i++) {
        if (g_sensors[i].detect(&s_state->sensor, id)) {
            camera_sensor_info_t *info = camera_sensor_get_info(id);
            if (NULL != info) {
                *out_camera_model = info->model;
                CAM_LOGI("Detected %s camera", info->name);
                g_sensors[i].init(&s_state->sensor);
                break;
            }
        }
    }

    if (CAMERA_NONE == *out_camera_model) { //If no supported sensors are detected
        //CAMERA_DISABLE_OUT_CLOCK();
        CAM_LOGE("Detected camera not supported.");
        return CAM_ERR_NOT_SUPPORTED;
    }

    CAM_LOGI("Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
             id->PID, id->VER, id->MIDH, id->MIDL);

    CAM_LOGD("Doing SW reset of sensor");
    delay(10);
    s_state->sensor.reset(&s_state->sensor);

    return CAM_OK;
}

cam_err_t camera_init(const camera_config_t *config)
{
    cam_err_t err;
    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;
    err = cam_init(config);
    if (err != CAM_OK) {
        CAM_LOGE("Camera init failed with error 0x%x", err);
        return err;
    }

    camera_model_t camera_model = CAMERA_NONE;
    err = camera_probe(config, &camera_model);   
    if (err != CAM_OK) {
        CAM_LOGE("Camera probe failed with error 0x%x", err);
        goto fail;
    }

    CAM_LOGD("Camera probe success:%d\n\r", s_state->sensor.slv_addr);

    if (PIXFORMAT_JPEG == pix_format && (!camera_sensor[camera_model].support_jpeg)) {
        CAM_LOGE("JPEG format is not supported on this sensor");
        err = CAM_ERR_NOT_SUPPORTED;
        goto fail;
    }

    if (frame_size > camera_sensor[camera_model].max_size) {
        CAM_LOGW("The frame size exceeds the maximum for this sensor, it will be forced to the maximum possible value");
        frame_size = camera_sensor[camera_model].max_size;
    }

    s_state->sensor.status.framesize = frame_size;
    s_state->sensor.pixformat = pix_format;
    CAM_LOGD("Setting frame size to %dx%d", resolution[frame_size].width, resolution[frame_size].height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        CAM_LOGE("Failed to set frame size");
        err = CAM_ERR_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }
    s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    if (s_state->sensor.id.PID == OV2640_PID) {
        s_state->sensor.set_gainceiling(&s_state->sensor, GAINCEILING_2X);
        s_state->sensor.set_bpc(&s_state->sensor, false);
        s_state->sensor.set_wpc(&s_state->sensor, true);
        s_state->sensor.set_lenc(&s_state->sensor, true);
    }

    if (pix_format == PIXFORMAT_JPEG) {
        s_state->sensor.set_quality(&s_state->sensor, config->jpeg_quality);
    }
    s_state->sensor.init_status(&s_state->sensor);

    //cam_start();

    return CAM_OK;

fail:
    camera_deinit();
    return CAM_FAIL;
}


cam_err_t camera_deinit()
{
    cam_err_t ret = cam_deinit();

    if (s_state) {
        SCCB_Deinit(s_state->sensor.sccb);

        free(s_state);
        s_state = NULL;
    }

    return ret;
}

// camera_fb_t *camera_fb_get()
// {
//     if (s_state == NULL) {
//         return NULL;
//     }
//     camera_fb_t *fb = cam_take(FB_GET_TIMEOUT);
//     //set the frame properties
//     if (fb) {
//         fb->width = resolution[s_state->sensor.status.framesize].width;
//         fb->height = resolution[s_state->sensor.status.framesize].height;
//         fb->format = s_state->sensor.pixformat;
//     }
//     return fb;
// }

// void camera_fb_return(camera_fb_t *fb)
// {
//     if (s_state == NULL) {
//         return;
//     }
//     cam_give(fb);
// }

// sensor_t *camera_sensor_get()
// {
//     if (s_state == NULL) {
//         return NULL;
//     }
//     return &s_state->sensor;
// }
