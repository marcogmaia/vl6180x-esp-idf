
/*******************************************************************************
Copyright � 2014, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/
/*
 * $Date: 2015-07-06 15:44:31 +0200 (Mon, 06 Jul 2015) $
 * $Revision: 2430 $
 */

/**
 * @file vl6180x_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */
#include "vl6180x_i2c.h"
#include "esp_log.h"

#ifndef I2C_BUFFER_CONFIG
#error "I2C_BUFFER_CONFIG not defined"
/* TODO you must define value for  I2C_BUFFER_CONFIG in configuration or
 * platform h */
#endif


#if I2C_BUFFER_CONFIG == 0
/* GLOBAL config buffer */
uint8_t i2c_global_buffer[VL6180x_MAX_I2C_XFER_SIZE];

#define DECL_I2C_BUFFER
#define VL6180x_GetI2cBuffer(dev, n_byte) i2c_global_buffer

#elif I2C_BUFFER_CONFIG == 1
/* ON STACK */
#define DECL_I2C_BUFFER uint8_t LocBuffer[VL6180x_MAX_I2C_XFER_SIZE];
#define VL6180x_GetI2cBuffer(dev, n_byte) LocBuffer
#elif I2C_BUFFER_CONFIG == 2
/* user define buffer type declare DECL_I2C_BUFFER  as access  via
 * VL6180x_GetI2cBuffer */
#define DECL_I2C_BUFFER
#else
#error "invalid I2C_BUFFER_CONFIG "
#endif

static const char TAG[] = "VL6180X";
int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_address << 1) | I2C_MASTER_WRITE,
                          true);
    i2c_master_write(cmd, buff, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret
        = i2c_master_cmd_begin(dev->i2c_port_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %s",
                 dev->i2c_address, I2C_NUM_0, esp_err_to_name(ret));
        esp_restart();
    }

    return 0;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_address << 1) | I2C_MASTER_READ, true);

    i2c_master_read(cmd, buff, len, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret
        = i2c_master_cmd_begin(dev->i2c_port_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %s",
                 dev->i2c_address, I2C_NUM_0, esp_err_to_name(ret));
        esp_restart();
    }

    return 0;
}

void VL6180x_GetI2CAccess(VL6180xDev_t dev) {
    xSemaphoreTake(dev->mutex, portMAX_DELAY);
}

void VL6180x_DoneI2CAccess(VL6180xDev_t dev) {
    xSemaphoreGive(dev->mutex);
}

int VL6180x_WrByte(VL6180xDev_t dev, uint16_t index, uint8_t data) {
    int status;
    uint8_t *buffer;
    DECL_I2C_BUFFER
    VL6180x_I2C_USER_VAR

        VL6180x_GetI2CAccess(dev);

    buffer    = VL6180x_GetI2cBuffer(dev, 3);
    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;
    buffer[2] = data;

    status = VL6180x_I2CWrite(dev, buffer, (uint8_t)3);
    VL6180x_DoneI2CAccess(dev);
    return status;
}

int VL6180x_WrWord(VL6180xDev_t dev, uint16_t index, uint16_t data) {
    int status;
    DECL_I2C_BUFFER
    uint8_t *buffer;
    VL6180x_I2C_USER_VAR

        VL6180x_GetI2CAccess(dev);

    buffer    = VL6180x_GetI2cBuffer(dev, 4);
    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;
    buffer[2] = data >> 8;
    buffer[3] = data & 0xFF;

    status = VL6180x_I2CWrite(dev, buffer, (uint8_t)4);
    VL6180x_DoneI2CAccess(dev);
    return status;
}

int VL6180x_WrDWord(VL6180xDev_t dev, uint16_t index, uint32_t data) {
    VL6180x_I2C_USER_VAR DECL_I2C_BUFFER int status;
    uint8_t *buffer;


    VL6180x_GetI2CAccess(dev);
    buffer    = VL6180x_GetI2cBuffer(dev, 6);
    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;
    buffer[2] = data >> 24;
    buffer[3] = (data >> 16) & 0xFF;
    buffer[4] = (data >> 8) & 0xFF;
    buffer[5] = data & 0xFF;
    status    = VL6180x_I2CWrite(dev, buffer, (uint8_t)6);
    VL6180x_DoneI2CAccess(dev);

    return status;
}

int VL6180x_UpdateByte(VL6180xDev_t dev, uint16_t index, uint8_t AndData,
                       uint8_t OrData) {
    VL6180x_I2C_USER_VAR int status;
    uint8_t *buffer;
    DECL_I2C_BUFFER

    VL6180x_GetI2CAccess(dev);

    buffer    = VL6180x_GetI2cBuffer(dev, 3);
    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;

    status = VL6180x_I2CWrite(dev, (uint8_t *)buffer, (uint8_t)2);
    if(!status) {
        /* read data direct onto buffer */
        status = VL6180x_I2CRead(dev, &buffer[2], 1);
        if(!status) {
            buffer[2] = (buffer[2] & AndData) | OrData;
            status    = VL6180x_I2CWrite(dev, buffer, (uint8_t)3);
        }
    }

    VL6180x_DoneI2CAccess(dev);

    return status;
}

int VL6180x_RdByte(VL6180xDev_t dev, uint16_t index, uint8_t *data) {
    VL6180x_I2C_USER_VAR int status;
    uint8_t *buffer;
    DECL_I2C_BUFFER

    VL6180x_GetI2CAccess(dev);

    buffer    = VL6180x_GetI2cBuffer(dev, 2);
    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;

    status = VL6180x_I2CWrite(dev, buffer, (uint8_t)2);
    if(!status) {
        status = VL6180x_I2CRead(dev, buffer, 1);
        if(!status) {
            *data = buffer[0];
        }
    }
    VL6180x_DoneI2CAccess(dev);

    return status;
}

int VL6180x_RdWord(VL6180xDev_t dev, uint16_t index, uint16_t *data) {
    VL6180x_I2C_USER_VAR int status;
    uint8_t *buffer;
    DECL_I2C_BUFFER

    VL6180x_GetI2CAccess(dev);

    buffer    = VL6180x_GetI2cBuffer(dev, 2);
    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;

    status = VL6180x_I2CWrite(dev, buffer, (uint8_t)2);
    if(!status) {
        status = VL6180x_I2CRead(dev, buffer, 2);
        if(!status) {
            /* VL6180x register are Big endian if cpu is be direct read direct
             * into *data is possible */
            *data = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
        }
    }
    VL6180x_DoneI2CAccess(dev);
    return status;
}

int VL6180x_RdDWord(VL6180xDev_t dev, uint16_t index, uint32_t *data) {
    VL6180x_I2C_USER_VAR int status;
    uint8_t *buffer;
    DECL_I2C_BUFFER

    VL6180x_GetI2CAccess(dev);
    buffer = VL6180x_GetI2cBuffer(dev, 4);

    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;

    status = VL6180x_I2CWrite(dev, (uint8_t *)buffer, (uint8_t)2);
    if(!status) {
        status = VL6180x_I2CRead(dev, buffer, 4);
        if(!status) {
            /* VL6180x register are Big endian if cpu is be direct read direct
             * into data is possible */
            *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16)
                    | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3]);
        }
    }
    VL6180x_DoneI2CAccess(dev);
    return status;
}


int VL6180x_RdMulti(VL6180xDev_t dev, uint16_t index, uint8_t *data,
                    int nData) {
    VL6180x_I2C_USER_VAR int status;
    uint8_t *buffer;
    DECL_I2C_BUFFER

    VL6180x_GetI2CAccess(dev);
    buffer = VL6180x_GetI2cBuffer(dev, 2);

    buffer[0] = index >> 8;
    buffer[1] = index & 0xFF;

    status = VL6180x_I2CWrite(dev, (uint8_t *)buffer, (uint8_t)2);
    if(!status) {
        status = VL6180x_I2CRead(dev, data, nData);
    }
    VL6180x_DoneI2CAccess(dev);
    return status;
}
