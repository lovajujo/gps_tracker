/*
 * mpu6050.c
 *
 *  Created on: Jul 5, 2023
 *      Author: lovzs
 */

#include <mpu6050.h>

/*
 * initialization of MPU6xxx sensor: reset sensor, configure gyroscope and accelerometer sensitivity and max range
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu)
{
	uint8_t reset=0u;
	uint16_t address=MPU6050_ADDRESS << 1;
	//reset sensor
	if(HAL_I2C_Mem_Write(I2Cx, address, PWR_MGMT_1, 1u, &reset, 1u, HAL_MAX_DELAY)!=HAL_OK) return 1;
	//gyroscope and accelerometer full scale
	if(HAL_I2C_Mem_Write(I2Cx, address, GYRO_CONFIG, 1u, &mpu->config.gRange, 1u, HAL_MAX_DELAY)!=HAL_OK) return 2;
	if(HAL_I2C_Mem_Write(I2Cx, address, ACCEL_CONFIG, 1u, &mpu->config.aRange, 1u, HAL_MAX_DELAY)!=HAL_OK) return 3;
	//gyro and acc. sensitivity
	MPU6050_SetAccRange(I2Cx, mpu);
	MPU6050_SetGyroRange(I2Cx, mpu);
	return 0;
}

/*
 * read data sensor data
 */
void MPU6050_GetRawData(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu)
{
	uint8_t address=MPU6050_ADDRESS << 1;
	int8_t acc_gyro[A_G_DATA_SIZE];
	//read data
	HAL_I2C_Mem_Read(I2Cx, address, ACCEL_XOUT_H, 1, acc_gyro, A_G_DATA_SIZE, HAL_MAX_DELAY);
	//convert high and low byte into a 16 bit variable
	mpu->rawData.ax = (int16_t)acc_gyro[0] << 8 | acc_gyro[1];
	mpu->rawData.ay = (int16_t)acc_gyro[2] << 8 | acc_gyro[3];
	mpu->rawData.az = (int16_t)acc_gyro[4] << 8 | acc_gyro[5];
	mpu->rawData.gx = (int16_t)acc_gyro[8] << 8 | acc_gyro[9];
	mpu->rawData.gy = (int16_t)acc_gyro[10] << 8 | acc_gyro[11];
	mpu->rawData.gz = (int16_t)acc_gyro[12] << 8 | acc_gyro[13];
}

/*
 * calculate acceleration and angular velocity
 */
void MPU6050_GetSensorData(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu)
{
	MPU6050_GetRawData(I2Cx, mpu);
	mpu->sensorData.ax=mpu->rawData.ax/mpu->config.aMaxRange;
	mpu->sensorData.ay=mpu->rawData.ay/mpu->config.aMaxRange;
	mpu->sensorData.az=mpu->rawData.az/mpu->config.aMaxRange;
	mpu->sensorData.gx=mpu->rawData.gx/mpu->config.gMaxRange;
	mpu->sensorData.gy=mpu->rawData.gy/mpu->config.gMaxRange;
	mpu->sensorData.gz=mpu->rawData.gz/mpu->config.gMaxRange;
}

/*
 * set maximum range of accelerometer
 */
void MPU6050_SetAccRange(I2C_HandleTypeDef *I2Cx,  MPU6050_t *mpu)
{
	switch(mpu->config.aRange)
	{
		case AFSR_2G:
			mpu->config.aMaxRange=AFSR_2G_MAX;
			break;
		case AFSR_4G:
			mpu->config.aMaxRange=AFSR_4G_MAX;
			break;
		case AFSR_8G:
			mpu->config.aMaxRange=AFSR_8G_MAX;
			break;
		case AFSR_16G:
			mpu->config.aMaxRange=AFSR_16G_MAX;
			break;
		default:
			mpu->config.aMaxRange=AFSR_2G_MAX;
			break;
	}
}

/*
 * set maximum range of gyroscope
 */
void MPU6050_SetGyroRange(I2C_HandleTypeDef *I2Cx,  MPU6050_t *mpu)
{
	switch(mpu->config.gRange)
	{
		case GFSR_250DPS:
			mpu->config.gMaxRange=GFSR_250DPS_MAX;
			break;
		case GFSR_500DPS:
			mpu->config.gMaxRange=GFSR_500DPS_MAX;
			break;
		case GFSR_1000DPS:
			mpu->config.gMaxRange=GFSR_1000DPS_MAX;
			break;
		case GFSR_2000DPS:
			mpu->config.gMaxRange=GFSR_2000DPS_MAX;
			break;
		default:
			mpu->config.gMaxRange=GFSR_250DPS_MAX;
			break;
	}
}


