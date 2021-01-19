/*
    SD card read/write

    This example shows how to read and write data to and from an SD card file
    The circuit:
    SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

    created   Nov 2010
    by David A. Mellis
    modified 9 Apr 2012
    by Tom Igoe

    This example code is in the public domain.

*/
#include <sys/param.h>
#include <string.h>
#include "STM32FreeRTOS.h"
#include "seeed_camera.h"
#include <Seeed_FS.h>
#include "SDMMC/Seeed_SDMMC.h"
#define ESP_OK 0
TaskHandle_t loopTaskHandle = NULL;

#define DEV  SDMMC

File RootWrite;
File RootRead;

SDMMCFS SDMMC;
/ WROVER-KIT PIN Map
#define CAM_PIN_PWDN 24  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 28
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 22
#define CAM_PIN_D6 20
#define CAM_PIN_D5 19
#define CAM_PIN_D4 17
#define CAM_PIN_D3 15
#define CAM_PIN_D2 13
#define CAM_PIN_D1 14
#define CAM_PIN_D0 16
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 18


static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

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

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_96X96,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};

static void init_camera(void* arg)
{
    //initialize the camera
    arduino_camera_init(&camera_config);
}

static void init_loop(void* arg)
{
  while(1)
  {
     camera_fb_t *pic = arduino_camera_fb_get();
     // use pic->buf to access the image
     RootWrite.write(pic->buf,pic->len);
     // close the file:
     RootWrite.close();
     printf("done.\r\n");
     printf("Picture taken! Its size was: %d bytes\n", pic->len);
    }
 }


void setup() {
    printf("initialization done..\r\n");
    if (!DEV.begin()) {
        printf("initialization error.\r\n");
    }
    printf("initialization done.\r\n");
    RootWrite = DEV.open("0:/hello.txt", "w+");

    portBASE_TYPE s1,s2;
    s1 = xTaskCreate(init_camera, NULL, 4096, NULL, 5,NULL);
    s2 = xTaskCreate(init_loop, NULL,2048, NULL, 5,NULL);
    if (s1 != pdPASS) {
    printf("Creation s1 problem\n");
    }
    if (s2 != pdPASS) {
    printf("Creation s2 problem\n");
    }
    vTaskStartScheduler();  
    
    // if the file opened okay, write to it:
    if (RootWrite) {
        printf("Writing to hello.txt...\r\n");
    } else {
        // if the file didn't open, print an error:
        printf("error opening hello.txt\r\n");
    }
    printf("write done...\r\n");
    delay(100);
    // re-open the file for reading:
    RootRead= DEV.open("0:/hello.txt");
    if (RootRead) {
        printf("hello.txt:");

        // read from the file until there's nothing else in it:
        while (RootRead.available()) {
            printf("%x ", RootRead.read());
        }
        // close the file:
        RootRead.close();
    } else {
        // if the file didn't open, print an error:
        printf("error opening hello.txt\r\n");
    }
    printf("read done...\r\n");
}

void loop() {
    // nothing happens after setup
}
