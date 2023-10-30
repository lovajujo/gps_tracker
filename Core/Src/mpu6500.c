/*
 * mpu6500.c
 *
 *  Created on: Jul 5, 2023
 *      Author: lovzs
 */

#include "mpu6500.h"

uint8_t MPU6500_Init(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu)
{
	uint8_t reset=0u;
	uint16_t address=MPU6500_ADDRESS << 1;
	HAL_StatusTypeDef r=HAL_I2C_IsDeviceReady(I2Cx, address, 1, 1000);
	if(HAL_I2C_Mem_Write(I2Cx, address, PWR_MGMT_1, 1, &reset, 1, 1000)!=HAL_OK) return 1;
	if(HAL_I2C_Mem_Write(I2Cx, address, GYRO_CONFIG, 1u, &mpu->config.gRange, 1u, HAL_MAX_DELAY)!=HAL_OK) return 2;
	if(HAL_I2C_Mem_Write(I2Cx, address, ACCEL_CONFIG, 1u, &mpu->config.aRange, 1u, HAL_MAX_DELAY)!=HAL_OK) return 3;
	return 0;
}

void MPU6500_GetRawData(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu)
{
	uint8_t address=MPU6500_ADDRESS << 1;
	int8_t acc_gyro[A_G_DATA_SIZE];
	HAL_I2C_Mem_Read(I2Cx, address, ACCEL_XOUT_H, 1, acc_gyro, A_G_DATA_SIZE, HAL_MAX_DELAY);
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


