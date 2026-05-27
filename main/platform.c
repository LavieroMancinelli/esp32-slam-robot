/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"
#include "driver/i2c.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0


uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAddress, uint32_t *value)
{
	uint8_t addr[2] = {RegisterAddress >> 8, RegisterAddress & 0xFF};
    uint8_t data[4];

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT,
        dev,
        addr,
        2,
        data,
        4,
        pdMS_TO_TICKS(100)
    );

    *value = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];

    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAddress, uint16_t *value)
{
	uint8_t addr[2] = {RegisterAddress >> 8, RegisterAddress & 0xFF};
    uint8_t data[2];

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT,
        dev,
        addr,
        2,
        data,
        2,
        pdMS_TO_TICKS(100)
    );

    *value = (data[0] << 8) | data[1];

    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAddress, uint8_t *value)
{
	uint8_t addr[2] = {RegisterAddress >> 8, RegisterAddress & 0xFF};

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT,
        dev,
        addr,
        2,
        value,
        1,
        pdMS_TO_TICKS(100)
    );

    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAddress, uint8_t value)
{
	uint8_t buf[3] = {RegisterAddress >> 8, RegisterAddress & 0xFF, value};

    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT,
        dev,
        buf,
        3,
        pdMS_TO_TICKS(100)
    );

    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAddress, uint16_t value)
{
	uint8_t buf[4] = {
        RegisterAddress >> 8,
        RegisterAddress & 0xFF,
        value >> 8,
        value & 0xFF
    };

    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT,
        dev,
        buf,
        4,
        pdMS_TO_TICKS(100)
    );

    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAddress, uint32_t value)
{
	uint8_t buf[6] = {
        RegisterAddress >> 8,
        RegisterAddress & 0xFF,
        value >> 24,
        value >> 16,
        value >> 8,
        value & 0xFF
    };

    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT,
        dev,
        buf,
        6,
        pdMS_TO_TICKS(100)
    );

    return (err == ESP_OK) ? 0 : 1;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
	vTaskDelay(pdMS_TO_TICKS(TimeMs));
    return 0;
}


