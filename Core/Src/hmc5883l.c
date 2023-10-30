/*
 * hmc5883l.c
 *
 *  Created on: Jul 18, 2023
 *      Author: lovzs
 */
#include "hmc5883l.h"
#include <math.h>


uint8_t HMC5883L_Init(I2C_HandleTypeDef *i2c, HMC5883L_t *hmc)
{
	//wrtie crB: set gain
	if(HAL_I2C_Mem_Write(i2c, HMC_WRITE, HMC_CONFIG_B, 1, hmc->gain, 1, HAL_MAX_DELAY)!=HAL_OK) return 1;
	//write modereg: set mode
	if(HAL_I2C_Mem_Write(i2c, HMC_WRITE, HMC_MODE, 1, hmc->mode, 1, HAL_MAX_DELAY)!=HAL_OK) return 2;
	//write crA: set sample rate if continuous mode
	if(hmc->mode==HMC_MODE_CONTINOUS)
	{
		if(HAL_I2C_Mem_Write(i2c, HMC_WRITE, HMC_CONFIG_A, 1, hmc->sample_rate, 1, HAL_MAX_DELAY)!=HAL_OK) return 3;
	}
	return 0;
}


void HMC5883L_ReadData(I2C_HandleTypeDef *i2c, HMC5883L_t *hmc)
{
	uint8_t buffer[BUFFER_SIZE];
	HAL_I2C_Mem_Read(i2c, HMC_READ, HMC_X_DATA_H, 1, buffer, BUFFER_SIZE, HAL_MAX_DELAY);
	hmc->x_data=((uint16_t)buffer[0]<<8) | buffer[1];
	hmc->z_data=((uint16_t)buffer[2]<<8) | buffer[3];
	hmc->y_data=((uint16_t)buffer[4]<<8) | buffer[5];
}

void HMC5883L_Calculate_direction(HMC5883L_t *hmc)
{
	//TODO
}

