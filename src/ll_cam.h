#pragma once

#include <stdint.h>
#include "camera.h"
#include "cam_utils.h"

typedef struct {
    camera_fb_t fb;
    uint8_t en;
} cam_frame_t;


typedef struct {
    cam_frame_t *frames;

    uint8_t jpeg_mode;
    uint32_t frame_cnt;
    uint32_t recv_size;
    
} cam_obj_t;

void camera_enable_out_clock(int pin, uint32_t xclk_freq_hz);
void camera_disable_out_clock(int pin);

bool ll_cam_stop();
bool ll_cam_start();

cam_err_t ll_cam_config(cam_obj_t *cam, const camera_config_t *config);
cam_err_t ll_cam_deinit(cam_obj_t *cam);

