/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */

#include "driver/include/sccb.h"

int SCCB_Init(int pin_sda, int pin_scl)
{
    Wire.setSCL(config->pin_sscb_scl);
    Wire.setSDA(config->pin_sscb_sda);
    Wire.begin();
    return 0;
}

uint8_t SCCB_Probe()
{

    return 0x60;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;
    Wire.beginTransmission(slv_addr);
    Wire.write(0xff);
    Wire.write(0x01); 
    Wire.write(reg);
    Wire.endTransmission();
    while (Wire.available())
    {
        data = Wire.read();
        return data;
    }
    
}

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(slv_addr);
    Wire.write(0xff);
    Wire.write(0x01);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
    return 0;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{

    return 0;
}

uint8_t SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    return 0;
}
