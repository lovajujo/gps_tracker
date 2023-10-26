/*
 * mpu6500.c
 *
 *  Created on: Jul 5, 2023
 *      Author: lovzs
 */

#include "mpu6500.h"

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu)
{
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDRESS, PWR_MGMT_1, 1, RESET, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDRESS, GYRO_CONFIG, 1, &mpu->config.gRange, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDRESS, ACCEL_CONFIG, 1, &mpu->config.aRange, 1, HAL_MAX_DELAY);
	return HAL_OK;
}

void MPU6500_GetRawData(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu)
{
	uint8_t addr=MPU6500_ADDRESS<<1;
	int8_t acc_gyro[A_G_DATA_SIZE];
	HAL_I2C_Mem_Read(I2Cx, addr, ACCEL_XOUT_H, 1, acc_gyro, A_G_DATA_SIZE, HAL_MAX_DELAY);
	mpu->rawData.ax = (int16_t)acc_gyro[0] << 8 | acc_gyro[1];
	mpu->rawData.ay = (int16_t)acc_gyro[2] << 8 | acc_gyro[3];
	mpu->rawData.az = (int16_t)acc_gyro[4] << 8 | acc_gyro[5];
	mpu->rawData.gx = (int16_t)acc_gyro[8] << 8 | acc_gyro[9];
	mpu->rawData.gy = (int16_t)acc_gyro[10] << 8 | acc_gyro[11];
	mpu->rawData.gz = (int16_t)acc_gyro[12] << 8 | acc_gyro[13];
}
void MPU6500_GetData(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu)
{
	MPU6500_GetRawData(I2Cx, mpu);
	float aMaxRange=MPU6500_SetAccRange(I2Cx, mpu);
	float gMaxRange=MPU6500_SetGyroRange(I2Cx, mpu);
	mpu->sensorData.ax=mpu->rawData.ax/aMaxRange;
	mpu->sensorData.ay=mpu->rawData.ay/aMaxRange;
	mpu->sensorData.az=mpu->rawData.az/aMaxRange;
	mpu->sensorData.gx=mpu->rawData.gx/gMaxRange;
	mpu->sensorData.gy=mpu->rawData.gy/gMaxRange;
	mpu->sensorData.gz=mpu->rawData.gz/gMaxRange;
}


float MPU6500_SetAccRange(I2C_HandleTypeDef *I2Cx,  MPU6500_t *mpu)
{
	float maxrange=0.0f;
	switch(mpu->config.aRange)
	{
		case AFSR_2G:
			maxrange=AFSR_2G_MAX;
			break;
		case AFSR_4G:
			maxrange=AFSR_4G_MAX;
			break;
		case AFSR_8G:
			maxrange=AFSR_8G_MAX;
			break;
		case AFSR_16G:
			maxrange=AFSR_16G_MAX;
			break;
		default:
			maxrange=AFSR_2G_MAX;
			break;
	}
	return maxrange;
}
float MPU6500_SetGyroRange(I2C_HandleTypeDef *I2Cx,  MPU6500_t *mpu)
{
	float maxrange=0.0f;
	switch(mpu->config.gRange)
	{
		case GFSR_250DPS:
			maxrange=GFSR_250DPS_MAX;
			break;
		case GFSR_500DPS:
			maxrange=GFSR_500DPS_MAX;
			break;
		case GFSR_1000DPS:
			maxrange=GFSR_1000DPS_MAX;
			break;
		case GFSR_2000DPS:
			maxrange=GFSR_2000DPS_MAX;
			break;
		default:
			maxrange=GFSR_250DPS_MAX;
			break;
	}
	return maxrange;
}


