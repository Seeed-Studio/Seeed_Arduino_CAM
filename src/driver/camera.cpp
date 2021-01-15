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
#include "driver/include/sensor.h"
#include "driver/include/sccb.h"
#include "driver/include/arduino_camera.h"
#include "driver/include/camera_common.h"
#include "driver/include/xclk.h"
#include "PinConfigured.h"



#if CONFIG_OV2640_SUPPORT
#include "sensors/include/ov2640.h"
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

DCMI_HandleTypeDef DCMI_Handle;
DMA_HandleTypeDef DMA_Handle_dcmi;

typedef enum {
    CAMERA_NONE = 0,
    CAMERA_UNKNOWN = 1,
    CAMERA_OV7725 = 7725,
    CAMERA_OV2640 = 2640,
    CAMERA_OV3660 = 3660,
    CAMERA_OV5640 = 5640,
    CAMERA_OV7670 = 7670,
    CAMERA_NT99141 = 9141,
} camera_model_t;

#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

#define REG16_CHIDH     0x300A
#define REG16_CHIDL     0x300B

static const char* CAMERA_SENSOR_NVS_KEY = "sensor";
static const char* CAMERA_PIXFORMAT_NVS_KEY = "pixformat";

typedef void (*dma_filter_t)(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);

typedef struct camera_fb_s {
    uint8_t * buf;
    size_t len;
    size_t width;
    size_t height;
    pixformat_t format;
    struct timeval timestamp;
    size_t size;
    uint8_t ref;
    uint8_t bad;
    struct camera_fb_s * next;
} camera_fb_int_t;

typedef struct fb_s {
    uint8_t * buf;
    size_t len;
    struct fb_s * next;
} fb_item_t;

typedef struct {
    camera_config_t config;
    sensor_t sensor;

    camera_fb_int_t *fb;
    size_t fb_size;
    size_t data_size;

    size_t width;
    size_t height;
    size_t in_bytes_per_pixel;
    size_t fb_bytes_per_pixel;

    size_t dma_received_count;
    size_t dma_filtered_count;
    size_t dma_per_line;
    size_t dma_buf_width;
    size_t dma_sample_count;

    lldesc_t *dma_desc;
    dma_elem_t **dma_buf;
    size_t dma_desc_count;
    size_t dma_desc_cur;

    dma_filter_t dma_filter;
    QueueHandle_t data_ready;
    QueueHandle_t fb_in;
    QueueHandle_t fb_out;

    SemaphoreHandle_t frame_ready;
    TaskHandle_t dma_filter_task;
} camera_state_t;

camera_state_t* s_state = NULL;

static void DMA_Config();
static void signal_dma_buf_received(bool *need_yield);
static uint8_t dma_desc_init();
static void dma_desc_deinit();
static void dma_filter_task(void *pvParameters);

#ifdef __cplusplus
extern "C"
{
#endif
    void DMA2_Stream1_IRQHandler(void)
    {
        HAL_DMA_IRQHandler(&DMA_Handle_dcmi);
        // signal_dma_buf_received(&need_yield);
    }

    void DCMI_PSSI_IRQHandler(void)
    {
        HAL_DCMI_IRQHandler(&DCMI_Handle);
    }

    void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
    {
        BaseType_t xHigherPriorityTaskWoken;
        #if 0
        printf("+++++++++++++++++++++++++++\n");
        printf("+++++++++++++++++++++++++++\n");
        for (int i = 0; i < 18432; i++)
        {
            if (i % 48 == 0)
            {
                printf("\n");
            }
            printf("%02X ", s_state->dma_buffer[i]);
        }
        printf("---------------------------\n");
        printf("---------------------------\n");
        printf("---------------------------\n");
        #endif
        bool need_yield = false;
        DMA_Config();
        signal_dma_buf_received(&need_yield);

    }
#ifdef __cplusplus
}
#endif

static int _gpio_get_level(gpio_num_t gpio_num)
{
    if (gpio_num < 32) {
        return (GPIO.in >> gpio_num) & 0x1;
    } else {
        return (GPIO.in1.data >> (gpio_num - 32)) & 0x1;
    }
}

static void vsync_intr_disable()
{
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_DISABLE);
}

static void vsync_intr_enable()
{
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_NEGEDGE);
}

static int skip_frame()
{
    if (s_state == NULL) {
        return -1;
    }
    unsigned long st_t = millis();
    while (digitalRead(s_state->config.pin_vsync) == 0) {
        if((millis() - st_t) > 1000000LL){
            goto timeout;
        }
    }
    while (digitalRead(s_state->config.pin_vsync) != 0) {
        if((millis() - st_t) > 1000000LL){
            goto timeout;
        }
    }

    while (digitalRead(s_state->config.pin_vsync) == 0) {
        if((millis() - st_t) > 1000000LL){
            goto timeout;
        }
    }
    return 0;
timeout:
    printf("Timeout waiting for VSYNC\n");
    return 1;
}

static void camera_fb_deinit()
{
    camera_fb_int_t * _fb1 = s_state->fb, * _fb2 = NULL;
    while(s_state->fb) {
        _fb2 = s_state->fb;
        s_state->fb = _fb2->next;
        if(_fb2->next == _fb1) {
            s_state->fb = NULL;
        }
        free(_fb2->buf);
        free(_fb2);
    }
}

static uint8_t camera_fb_init(size_t count)
{
    if(!count) {
        return ESP_ERR_INVALID_ARG;
    }

    camera_fb_deinit();

    printf("Allocating %u frame buffers (%d KB total)\n", count, (s_state->fb_size * count) / 1024);

    camera_fb_int_t * _fb = NULL, * _fb1 = NULL, * _fb2 = NULL;
    for(size_t i = 0; i < count; i++) {
        _fb2 = (camera_fb_int_t *)malloc(sizeof(camera_fb_int_t));
        if(!_fb2) {
            goto fail;
        }
        memset(_fb2, 0, sizeof(camera_fb_int_t));
        _fb2->size = s_state->fb_size;
        _fb2->buf = (uint8_t*) malloc(_fb2->size);
        if(!_fb2->buf) {
            printf("Allocating %d KB frame buffer in PSRAM\n", s_state->fb_size/1024);
            _fb2->buf = (uint8_t*) heap_caps_calloc(_fb2->size, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        } else {
            printf("Allocating %d KB frame buffer in OnBoard RAM\n", s_state->fb_size/1024);
        }
        if(!_fb2->buf) {
            free(_fb2);
            printf("Allocating %d KB frame buffer Failed\n", s_state->fb_size/1024);
            goto fail;
        }
        memset(_fb2->buf, 0, _fb2->size);
        _fb2->next = _fb;
        _fb = _fb2;
        if(!i) {
            _fb1 = _fb2;
        }
    }
    if(_fb1) {
        _fb1->next = _fb;
    }

    s_state->fb = _fb;//load first buffer

    return ESP_OK;

fail:
    while(_fb) {
        _fb2 = _fb;
        _fb = _fb->next;
        free(_fb2->buf);
        free(_fb2);
    }
    return ESP_ERR_NO_MEM;
}

static uint8_t dma_desc_init()
{
    s_state->dma_buffer = (uint8_t *) malloc(s_state->height * s_state->width * s_state->in_bytes_per_pixel);
    if (s_state->dma_buffer == nullptr)
    {
        printf("disbuf malloc failed\n");
        return -1;
    }
    return 0; 
}

static void dma_desc_deinit()
{
    if (s_state->dma_buf) {
        for (int i = 0; i < s_state->dma_desc_count; ++i) {
            free(s_state->dma_buf[i]);
        }
    }
    free(s_state->dma_buf);
    free(s_state->dma_desc);
}


static void hw_gpio_init(const camera_config_t* config)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    PinName p = digitalPinToPinName(config->pin_vsync);
    pin_function(p, STM_PIN_DATA(STM_MODE_AF_PP,GPIO_PULLUP, 13));

    p = digitalPinToPinName(config->pin_pwdn);
    pin_function(p, STM_PIN_DATA(STM_MODE_OUTPUT_PP,GPIO_NOPULL, 13));

    const int pins[] = {
        config->pin_d7,
        config->pin_d6,
        config->pin_d5,
        config->pin_d4,
        config->pin_d3,
        config->pin_d2,
        config->pin_d1,
        config->pin_d0, 
        config->pin_href,
        config->pin_pclk
    };

    for (int i = 0; i < sizeof(pins) / sizeof(int); ++i) {
        PinName x = digitalPinToPinName(pins[i]);
        pin_function(x, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, 13));
    }
 
}

static void dcmi_init()
{
    __HAL_RCC_DCMI_CLK_ENABLE();


	DCMI_Handle.Instance              = DCMI;    

	DCMI_Handle.Init.SynchroMode      = DCMI_MODE_CONTINUOUS;

	DCMI_Handle.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;

	DCMI_Handle.Init.PCKPolarity      = DCMI_PCKPOLARITY_RISING;

	DCMI_Handle.Init.VSPolarity       = DCMI_VSPOLARITY_LOW;

	DCMI_Handle.Init.HSPolarity       = DCMI_HSPOLARITY_LOW;

	DCMI_Handle.Init.CaptureRate      = DCMI_CR_ALL_FRAME;

	DCMI_Handle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
	HAL_DCMI_Init(&DCMI_Handle);

    if (HAL_DCMI_GetState(&DCMI_Handle) == 1)
	{
		printf("init dcmi ok\n");
	}

	HAL_NVIC_SetPriority(DCMI_IRQn, 0 ,5);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);

    DMA_Config();
}

static void DMA_Config()
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    DMA_Handle_dcmi.Instance = DMA2_Stream1;
    DMA_Handle_dcmi.Init.Request = DMA_REQUEST_DCMI;
    DMA_Handle_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DMA_Handle_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_Handle_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    DMA_Handle_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    DMA_Handle_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DMA_Handle_dcmi.Init.Mode = DMA_CIRCULAR;
    DMA_Handle_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    DMA_Handle_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    DMA_Handle_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    DMA_Handle_dcmi.Init.MemBurst = DMA_MBURST_INC8;
    DMA_Handle_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&DMA_Handle_dcmi);
    __HAL_LINKDMA(&DCMI_Handle, DMA_Handle, DMA_Handle_dcmi);

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    if (HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_CONTINUOUS, (uint32_t)s_state->dma_buffer,(((s_state->width) * (s_state->height)) / 2)) == HAL_OK)
    {
        printf("HAL DCMI Start DMA ok\n");
    }
}


static void signal_dma_buf_received(bool* need_yield)
{
    uint8_t *dma_desc_filled;
    dma_desc_filled = s_state->dma_buffer;
    if (!s_state->fb->ref && s_state->fb->bad)
    {
        *need_yield = false;
        return;
    }
    BaseType_t higher_priority_task_woken;
    if (s_state->data_ready == NULL)
    {
        return;
    }
    BaseType_t ret = xQueueSendFromISR(s_state->data_ready,dma_desc_filled, &higher_priority_task_woken);
    if (ret != pdTRUE)
    {
        if (!s_state->fb->ref)
        {
            s_state->fb->bad = 1;
        }
    }
    *need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
    portYIELD_FROM_ISR();
}


static void camera_fb_done()
{
    camera_fb_int_t * fb = NULL, * fb2 = NULL;
    BaseType_t taskAwoken = 0;

    if(s_state->config.fb_count == 1) {
        xSemaphoreGive(s_state->frame_ready);
        return;
    }

    fb = s_state->fb;
    if(!fb->ref && fb->len) {
        //add reference
        fb->ref = 1;

        //check if the queue is full
        if(xQueueIsQueueFullFromISR(s_state->fb_out) == pdTRUE) {
            //pop frame buffer from the queue
            if(xQueueReceiveFromISR(s_state->fb_out, &fb2, &taskAwoken) == pdTRUE) {
                //free the popped buffer
                fb2->ref = 0;
                fb2->len = 0;
                //push the new frame to the end of the queue
                xQueueSendFromISR(s_state->fb_out, &fb, &taskAwoken);
            } else {
                //queue is full and we could not pop a frame from it
            }
        } else {
            //push the new frame to the end of the queue
            xQueueSendFromISR(s_state->fb_out, &fb, &taskAwoken);
        }
    } else {
        //frame was referenced or empty
    }

    //return buffers to be filled
    while(xQueueReceiveFromISR(s_state->fb_in, &fb2, &taskAwoken) == pdTRUE) {
        fb2->ref = 0;
        fb2->len = 0;
    }

    //advance frame buffer only if the current one has data
    if(s_state->fb->len) {
        s_state->fb = s_state->fb->next;
    }
    //try to find the next free frame buffer
    while(s_state->fb->ref && s_state->fb->next != fb) {
        s_state->fb = s_state->fb->next;
    }
    //is the found frame buffer free?
    if(!s_state->fb->ref) {
        //buffer found. make sure it's empty
        s_state->fb->len = 0;
        *((uint32_t *)s_state->fb->buf) = 0;
    } else {
        //stay at the previous buffer
        s_state->fb = fb;
    }
}


static void dma_finish_frame()
{
    if (!s_state->fb->ref)
    {  
        s_state->fb->width = resolution[s_state->sensor.status.framesize].width;
        s_state->fb->height = resolution[s_state->sensor.status.framesize].height;
        s_state->fb->len = (s_state->height) * (s_state->width) * s_state->fb_bytes_per_pixel;
        s_state->fb->format = s_state->sensor.pixformat;
        uint64_t us = (uint64_t)millis();
        s_state->fb->timestamp.tv_sec = us / 1000000UL;
        s_state->fb->timestamp.tv_usec = us % 1000000UL;
        camera_fb_done();
    }
}

static void dma_filter_task(void *pvParameters)
{
    s_state->dma_filtered_count = 0;
    uint8_t *buf_idx = (uint8_t *)malloc(s_state->width * s_state->in_bytes_per_pixel);
    while (true)
    {
        if(xQueueReceive(s_state->data_ready, buf_idx, portMAX_DELAY) == pdTRUE) {
            printf("into xxQueueReceive\n");
            //this is the end of the frame
            dma_finish_frame();
        }
    }
}

/*
 * Public Methods
 * */

uint8_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model)
{
    if (s_state != NULL) {
        return -1;
    }

    s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
    if (!s_state) {
        return -1;
    }
#if 0
    TODO: to be removed?
    if(config->pin_xclk >= 0) {
    //   printf(TAG, "Enabling XCLK output");
      camera_enable_out_clock(config);
    }
#endif
    if (config->pin_sscb_sda != -1) {
      printf("Initializing SSCB\n");
      SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);
    }
	
    printf("Searching for camera address\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uint8_t slv_addr = SCCB_Probe();
    if (slv_addr == 0) {
        *out_camera_model = CAMERA_NONE;
        return ESP_ERR_CAMERA_NOT_DETECTED;
    }
    
    // slv_addr = 0x30;
    sensor_id_t* id = &s_state->sensor.id;

#if CONFIG_OV2640_SUPPORT
    if (slv_addr == 0x30) {
        //camera might be OV2640. try to reset it
        SCCB_Write(0x30, 0xFF, 0x01);//bank sensor
        SCCB_Write(0x30, 0x12, 0x80);//reset
        delay(10);
        slv_addr = SCCB_Probe();
    }
#endif
#if CONFIG_NT99141_SUPPORT
   if (slv_addr == 0x2a)
    {
        printf("Resetting NT99141\n");
        SCCB_Write16(0x2a, 0x3008, 0x01);//bank sensor
    }
#endif 

    s_state->sensor.slv_addr = slv_addr;
    s_state->sensor.xclk_freq_hz = config->xclk_freq_hz;

#if (CONFIG_OV3660_SUPPORT || CONFIG_OV5640_SUPPORT || CONFIG_NT99141_SUPPORT)
    if(s_state->sensor.slv_addr == 0x3c){
        id->PID = SCCB_Read16(s_state->sensor.slv_addr, REG16_CHIDH);
        id->VER = SCCB_Read16(s_state->sensor.slv_addr, REG16_CHIDL);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        printf("Camera PID=0x%02x VER=0x%02x\n", id->PID, id->VER);
    } else if(s_state->sensor.slv_addr == 0x2a){
        id->PID = SCCB_Read16(s_state->sensor.slv_addr, 0x3000);
        id->VER = SCCB_Read16(s_state->sensor.slv_addr, 0x3001);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        printf("Camera PID=0x%02x VER=0x%02x\n", id->PID, id->VER);
        if(config->xclk_freq_hz > 10000000)
        {
            printf("NT99141: only XCLK under 10MHz is supported, and XCLK is now set to 10M\n");
            s_state->sensor.xclk_freq_hz = 10000000;
        }    
    } else {
#endif
        id->PID = SCCB_Read(s_state->sensor.slv_addr, REG_PID);
        id->VER = SCCB_Read(s_state->sensor.slv_addr, REG_VER);
        id->MIDL = SCCB_Read(s_state->sensor.slv_addr, REG_MIDL);
        id->MIDH = SCCB_Read(s_state->sensor.slv_addr, REG_MIDH);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        printf("Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x\n",
                 id->PID, id->VER, id->MIDH, id->MIDL);

#if (CONFIG_OV3660_SUPPORT || CONFIG_OV5640_SUPPORT || CONFIG_NT99141_SUPPORT)
    }
#endif


    switch (id->PID) {
#if CONFIG_OV2640_SUPPORT
    case OV2640_PID:
        *out_camera_model = CAMERA_OV2640;
        ov2640_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV7725_SUPPORT
    case OV7725_PID:
        *out_camera_model = CAMERA_OV7725;
        ov7725_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV3660_SUPPORT
    case OV3660_PID:
        *out_camera_model = CAMERA_OV3660;
        ov3660_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV5640_SUPPORT
    case OV5640_PID:
        *out_camera_model = CAMERA_OV5640;
        ov5640_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV7670_SUPPORT
    case OV7670_PID:
        *out_camera_model = CAMERA_OV7670;
        ov7670_init(&s_state->sensor);
        break;
#endif
#if CONFIG_NT99141_SUPPORT
        case NT99141_PID:
        *out_camera_model = CAMERA_NT99141;
        NT99141_init(&s_state->sensor);
        break;
#endif
    default:
        id->PID = 0;
        *out_camera_model = CAMERA_UNKNOWN;
        printf("Detected camera not supported.\n");
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    printf("Doing SW reset of sensor\n");
    s_state->sensor.reset(&s_state->sensor);

    return ESP_OK;
}

uint8_t camera_init(const camera_config_t* config)
{
    if (!s_state) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->sensor.id.PID == 0) {
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }
    memcpy(&s_state->config, config, sizeof(*config));
    uint8_t err = ESP_OK;
    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;

    switch (s_state->sensor.id.PID) {
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            if (frame_size > FRAMESIZE_UXGA) {
                frame_size = FRAMESIZE_UXGA;
            }
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            if (frame_size > FRAMESIZE_VGA) {
                frame_size = FRAMESIZE_VGA;
            }
            break;
#endif
#if CONFIG_OV3660_SUPPORT
        case OV3660_PID:
            if (frame_size > FRAMESIZE_QXGA) {
                frame_size = FRAMESIZE_QXGA;
            }
            break;
#endif
#if CONFIG_OV5640_SUPPORT
        case OV5640_PID:
            if (frame_size > FRAMESIZE_QSXGA) {
                frame_size = FRAMESIZE_QSXGA;
            }
            break;
#endif
#if CONFIG_OV7670_SUPPORT
        case OV7670_PID:
            if (frame_size > FRAMESIZE_VGA) {
                frame_size = FRAMESIZE_VGA;
            }
            break;
#endif
#if CONFIG_NT99141_SUPPORT
        case NT99141_PID:
            if (frame_size > FRAMESIZE_HD) {
                frame_size = FRAMESIZE_HD;
            }
            break;
#endif
        default:
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    s_state->width = resolution[frame_size].width;
    s_state->height = resolution[frame_size].height;

    if (pix_format == PIXFORMAT_GRAYSCALE) {
        s_state->fb_size = s_state->width * s_state->height;
        if (s_state->sensor.id.PID == OV3660_PID || s_state->sensor.id.PID == OV5640_PID || s_state->sensor.id.PID == NT99141_PID) {
            s_state->in_bytes_per_pixel = 1;       // camera sends Y8
        } else {
            s_state->in_bytes_per_pixel = 2;       // camera sends YU/YV
        }
        s_state->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
    } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
            s_state->fb_size = s_state->width * s_state->height * 2;
            s_state->in_bytes_per_pixel = 2;       // camera sends YU/YV
            s_state->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
    } else if (pix_format == PIXFORMAT_RGB888) {
        s_state->fb_size = s_state->width * s_state->height * 3;
        s_state->in_bytes_per_pixel = 2;       // camera sends RGB565
        s_state->fb_bytes_per_pixel = 3;       // frame buffer stores RGB888
    } else if (pix_format == PIXFORMAT_JPEG) {
        if (s_state->sensor.id.PID != OV2640_PID && s_state->sensor.id.PID != OV3660_PID && s_state->sensor.id.PID != OV5640_PID  && s_state->sensor.id.PID != NT99141_PID) {
            printf("JPEG format is only supported for ov2640, ov3660 and ov5640\n");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        int qp = config->jpeg_quality;
        int compression_ratio_bound = 1;
        if (qp > 10) {
            compression_ratio_bound = 16;
        } else if (qp > 5) {
            compression_ratio_bound = 10;
        } else {
            compression_ratio_bound = 4;
        }
        (*s_state->sensor.set_quality)(&s_state->sensor, qp);
        s_state->in_bytes_per_pixel = 2;
        s_state->fb_bytes_per_pixel = 2;
        s_state->fb_size = (s_state->width * s_state->height * s_state->fb_bytes_per_pixel) / compression_ratio_bound;
    } else {
        printf("Requested format is not supported\n");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    s_state->data_ready = xQueueCreate(16, sizeof(s_state->width * s_state->height * s_state->fb_bytes_per_pixel));
    if (s_state->data_ready == NULL) {
        printf("Failed to dma queue\n");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    if(s_state->config.fb_count == 1) {
        s_state->frame_ready = xSemaphoreCreateBinary();
        if (s_state->frame_ready == NULL) {
            printf("Failed to create semaphore\n");
            err = ESP_ERR_NO_MEM;
            goto fail;
        }
    } else {
        s_state->fb_in = xQueueCreate(s_state->config.fb_count, sizeof(camera_fb_t *));
        s_state->fb_out = xQueueCreate(1, sizeof(camera_fb_t *));
        if (s_state->fb_in == NULL || s_state->fb_out == NULL) {
            printf("Failed to fb queues\n");
            err = ESP_ERR_NO_MEM;
            goto fail;
        }
    }

    //ToDo: core affinity?
#if CONFIG_CAMERA_CORE0
    if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 0))
#elif CONFIG_CAMERA_CORE1
    if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 1))
#else
    if (!xTaskCreate(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task))
#endif
    {
        printf("Failed to create DMA filter task\n");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    s_state->sensor.status.framesize = frame_size;
    s_state->sensor.pixformat = pix_format;
    printf("Setting frame size to %dx%d\n", s_state->width, s_state->height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        printf("Failed to set frame size\n");
        err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }
    s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    if (s_state->sensor.id.PID == OV2640_PID) {
        s_state->sensor.set_gainceiling(&s_state->sensor, GAINCEILING_2X);
        s_state->sensor.set_bpc(&s_state->sensor, false);
        s_state->sensor.set_wpc(&s_state->sensor, true);
        s_state->sensor.set_lenc(&s_state->sensor, true);
    }

    //todo: for some reason the first set of the quality does not work.
    if (pix_format == PIXFORMAT_JPEG) {
        (*s_state->sensor.set_quality)(&s_state->sensor, config->jpeg_quality);
    }
    s_state->sensor.init_status(&s_state->sensor);

    err = dma_desc_init();
    if (err != ESP_OK) {
        printf("Failed to initialize DMA\n");
        goto fail;
    }

    dcmi_init();
    if (skip_frame())
    {
        err = ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT;
        goto fail;
    }
    // s_state->fb_size = 75 * 1024;
    err = camera_fb_init(s_state->config.fb_count);
    if (err != ESP_OK) {
        printf("Failed to allocate frame buffer\n");
        goto fail;
    }
    printf("camera init end\n");
    return ESP_OK;

fail:
    arduino_camera_deinit();
    return err;
}

uint8_t arduino_camera_init(const camera_config_t* config)
{
    camera_model_t camera_model = CAMERA_NONE;
    hw_gpio_init(config);
    uint8_t err = camera_probe(config, &camera_model);
    if (err != ESP_OK) {
        printf("Camera probe failed with error 0x%x\n", err);
        goto fail;
    }
    if (camera_model == CAMERA_OV7725) {
        printf("Detected OV7725 camera\n");
        if(config->pixel_format == PIXFORMAT_JPEG) {
            printf("Camera does not support JPEG\n");
            err = ESP_ERR_CAMERA_NOT_SUPPORTED;
            goto fail;
        }
    } else if (camera_model == CAMERA_OV2640) {
        printf("Detected OV2640 camera\n");
    } else if (camera_model == CAMERA_OV3660) {
        printf("Detected OV3660 camera\n");
    } else if (camera_model == CAMERA_OV5640) {
        printf("Detected OV5640 camera\n");
    } else if (camera_model == CAMERA_OV7670) {
        printf("Detected OV7670 camera\n");
    } else if (camera_model == CAMERA_NT99141) {
        printf("Detected NT99141 camera\n");
    } else {
        printf("Camera not supported\n");
        err = ESP_ERR_CAMERA_NOT_SUPPORTED;
        goto fail;
    }
    err = camera_init(config);
    if (err != ESP_OK) {
        printf("Camera init failed with error 0x%x\n", err);
        return err;
    }
    printf("arduino camera init end\n");
    while(1)
    {
        delay(100);
        portYIELD();

    }
    return ESP_OK;

fail:
    free(s_state);
    s_state = NULL;
    camera_disable_out_clock();
    return err;
}

uint8_t arduino_camera_deinit()
{
    if (s_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->dma_filter_task) {
        vTaskDelete(s_state->dma_filter_task);
    }
    if (s_state->data_ready) {
        vQueueDelete(s_state->data_ready);
    }
    if (s_state->fb_in) {
        vQueueDelete(s_state->fb_in);
    }
    if (s_state->fb_out) {
        vQueueDelete(s_state->fb_out);
    }
    if (s_state->frame_ready) {
        vSemaphoreDelete(s_state->frame_ready);
    }
#if 0
    gpio_isr_handler_remove(s_state->config.pin_vsync);
    if (s_state->i2s_intr_handle) {
        esp_intr_disable(s_state->i2s_intr_handle);
        esp_intr_free(s_state->i2s_intr_handle);
    }
#endif
    dma_desc_deinit();
    camera_fb_deinit();

    if(s_state->config.pin_xclk >= 0) {
      camera_disable_out_clock();
    }
    free(s_state);
    s_state = NULL;
    periph_module_disable(PERIPH_I2S0_MODULE);
    return ESP_OK;
}

#define FB_GET_TIMEOUT (4000 / portTICK_PERIOD_MS)

camera_fb_t* arduino_camera_fb_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    if(!I2S0.conf.rx_start) {
        if(s_state->config.fb_count > 1) {
            printf("i2s_run\n");
        }
        if (i2s_run() != 0) {
            return NULL;
        }
    }
    bool need_yield = false;
    if (s_state->config.fb_count == 1) {
        if (xSemaphoreTake(s_state->frame_ready, FB_GET_TIMEOUT) != pdTRUE){
            i2s_stop(&need_yield);
            printf("Failed to get the frame on time!\n");
            return NULL;
        }
        return (camera_fb_t*)s_state->fb;
    }
    camera_fb_int_t * fb = NULL;
    if(s_state->fb_out) {
        if (xQueueReceive(s_state->fb_out, &fb, FB_GET_TIMEOUT) != pdTRUE) {
            i2s_stop(&need_yield);
            printf("Failed to get the frame on time!\n");
            return NULL;
        }
    }
    return (camera_fb_t*)fb;
}

void arduino_camera_fb_return(camera_fb_t * fb)
{
    if(fb == NULL || s_state == NULL || s_state->config.fb_count == 1 || s_state->fb_in == NULL) {
        return;
    }
    xQueueSend(s_state->fb_in, &fb, portMAX_DELAY);
}

sensor_t * arduino_camera_sensor_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    return &s_state->sensor;
}

uint8_t arduino_camera_save_to_nvs(const char *key) 
{
#if ESP_IDF_VERSION_MAJOR > 3
    nvs_handle_t handle;
#else
    nvs_handle handle;
#endif
    uint8_t ret = nvs_open(key,NVS_READWRITE,&handle);
    
    if (ret == ESP_OK) {
        sensor_t *s = arduino_camera_sensor_get();
        if (s != NULL) {
            ret = nvs_set_blob(handle,CAMERA_SENSOR_NVS_KEY,&s->status,sizeof(camera_status_t));
            if (ret == ESP_OK) {
                uint8_t pf = s->pixformat;
                ret = nvs_set_u8(handle,CAMERA_PIXFORMAT_NVS_KEY,pf);
            }
            return ret;
        } else {
            return ESP_ERR_CAMERA_NOT_DETECTED; 
        }
        nvs_close(handle);
        return ret;
    } else {
        return ret;
    }
}

uint8_t arduino_camera_load_from_nvs(const char *key) 
{
#if ESP_IDF_VERSION_MAJOR > 3
    nvs_handle_t handle;
#else
    nvs_handle handle;
#endif
  uint8_t pf;

  uint8_t ret = nvs_open(key,NVS_READWRITE,&handle);
  
  if (ret == ESP_OK) {
      sensor_t *s = arduino_camera_sensor_get();
      camera_status_t st;
      if (s != NULL) {
        size_t size = sizeof(camera_status_t);
        ret = nvs_get_blob(handle,CAMERA_SENSOR_NVS_KEY,&st,&size);
        if (ret == ESP_OK) {
            s->set_ae_level(s,st.ae_level);
            s->set_aec2(s,st.aec2);
            s->set_aec_value(s,st.aec_value);
            s->set_agc_gain(s,st.agc_gain);
            s->set_awb_gain(s,st.awb_gain);
            s->set_bpc(s,st.bpc);
            s->set_brightness(s,st.brightness);
            s->set_colorbar(s,st.colorbar);
            s->set_contrast(s,st.contrast);
            s->set_dcw(s,st.dcw);
            s->set_denoise(s,st.denoise);
            s->set_exposure_ctrl(s,st.aec);
            s->set_framesize(s,st.framesize);
            s->set_gain_ctrl(s,st.agc);          
            s->set_gainceiling(s,st.gainceiling);
            s->set_hmirror(s,st.hmirror);
            s->set_lenc(s,st.lenc);
            s->set_quality(s,st.quality);
            s->set_raw_gma(s,st.raw_gma);
            s->set_saturation(s,st.saturation);
            s->set_sharpness(s,st.sharpness);
            s->set_special_effect(s,st.special_effect);
            s->set_vflip(s,st.vflip);
            s->set_wb_mode(s,st.wb_mode);
            s->set_whitebal(s,st.awb);
            s->set_wpc(s,st.wpc);
        }  
        ret = nvs_get_u8(handle,CAMERA_PIXFORMAT_NVS_KEY,&pf);
        if (ret == ESP_OK) {
          s->set_pixformat(s,pf);
        }
      } else {
          return ESP_ERR_CAMERA_NOT_DETECTED;
      }
      nvs_close(handle);
      return ret;
  } else {
      printf("Error (%d) opening nvs key \"%s\"\n",ret,key);
      return ret;
  }
}
