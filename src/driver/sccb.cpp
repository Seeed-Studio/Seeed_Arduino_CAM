/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */

#include "driver/include/sccb.h"

#define LITTLETOBIG(x) ((x << 8) | (x >> 8))

int SCCB_Init(int pin_sda, int pin_scl)
{
    Wire.setSCL((uint32_t)pin_scl);
    Wire.setSDA((uint32_t)pin_sda);
    Wire.begin();
    return 0;
}

uint8_t SCCB_Probe()
{
    uint8_t error, address;
    int nDevices;
    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
             
            printf("I2C device found at address 0x%x\n",address);

            nDevices++;
            switch (address)
            {
            case 0x30:
                break;
            case 0x2a:
                break;
            case 0x3c:
                break;
            default:
                continue;
            }
            return address;
        }
        else if (error == 4)
        {
            printf("Unknown error at address 0x%x\n",address);
        }
    }
    if (nDevices == 0)
        printf("No I2C devices found");
    else
        printf("done");
    return 0;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;
    Wire.beginTransmission(slv_addr);
    Wire.endTransmission();
    Wire.requestFrom(slv_addr,1,reg,1,true);
    while (Wire.available())
    {
        data = Wire.read();
        return data;
    }
    
}

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(slv_addr);
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
