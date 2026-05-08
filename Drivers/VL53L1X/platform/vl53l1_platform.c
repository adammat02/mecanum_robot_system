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

#include "vl53l1_platform.h"
#include <string.h>

#define I2C_TIMEOUT_MS 100U

int8_t VL53L1_WriteMulti(VL53L1_Dev_t *dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, index,
																							 I2C_MEMADD_SIZE_16BIT, pdata, (uint16_t)count, I2C_TIMEOUT_MS);
	return (status == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(VL53L1_Dev_t *dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, index,
																							I2C_MEMADD_SIZE_16BIT, pdata, (uint16_t)count, I2C_TIMEOUT_MS);
	return (status == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_WrByte(VL53L1_Dev_t *dev, uint16_t index, uint8_t data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, index,
																							 I2C_MEMADD_SIZE_16BIT, &data, 1, I2C_TIMEOUT_MS);
	return (status == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_WrWord(VL53L1_Dev_t *dev, uint16_t index, uint16_t data)
{
	uint8_t buf[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, index,
																							 I2C_MEMADD_SIZE_16BIT, buf, 2, I2C_TIMEOUT_MS);
	return (status == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_WrDWord(VL53L1_Dev_t *dev, uint16_t index, uint32_t data)
{
	uint8_t buf[4] = {
			(uint8_t)(data >> 24),
			(uint8_t)(data >> 16),
			(uint8_t)(data >> 8),
			(uint8_t)(data & 0xFF)};
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, index,
																							 I2C_MEMADD_SIZE_16BIT, buf, 4, I2C_TIMEOUT_MS);
	return (status == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_RdByte(VL53L1_Dev_t *dev, uint16_t index, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, index,
																							I2C_MEMADD_SIZE_16BIT, data, 1, I2C_TIMEOUT_MS);
	return (status == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_RdWord(VL53L1_Dev_t *dev, uint16_t index, uint16_t *data)
{
	uint8_t buf[2];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, index,
																							I2C_MEMADD_SIZE_16BIT, buf, 2, I2C_TIMEOUT_MS);
	if (status != HAL_OK)
		return -1;
	*data = ((uint16_t)buf[0] << 8) | buf[1];
	return 0;
}

int8_t VL53L1_RdDWord(VL53L1_Dev_t *dev, uint16_t index, uint32_t *data)
{
	uint8_t buf[4];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, index,
																							I2C_MEMADD_SIZE_16BIT, buf, 4, I2C_TIMEOUT_MS);
	if (status != HAL_OK)
		return -1;
	*data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
	return 0;
}

int8_t VL53L1_WaitMs(VL53L1_Dev_t *dev, int32_t wait_ms)
{
	(void)dev;
	HAL_Delay((uint32_t)wait_ms);
	return 0;
}
