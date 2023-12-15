/*
 * mpu6050.h
 *
 *  Created on: Jul 5, 2023
 *      Author: lovzs
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define MPU6050_ADDRESS 0x68
#define USER_CONTROL 0x6A
#define ACCEL_XOUT_H 0x3B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B
#define A_G_DATA_SIZE 14

#define GFSR_250DPS 0x00
#define GFSR_500DPS 0x08
#define GFSR_1000DPS 0x10
#define GFSR_2000DPS 0x18

#define AFSR_2G 0x00
#define AFSR_4G 0x08
#define AFSR_8G 0x10
#define AFSR_16G 0x18

#define GFSR_250DPS_MAX 131.0f
#define GFSR_500DPS_MAX 65.5f
#define GFSR_1000DPS_MAX 32.8f
#define GFSR_2000DPS_MAX 16.4f

#define AFSR_2G_MAX 16384.0f
#define AFSR_4G_MAX 8192.0f
#define AFSR_8G_MAX 4096.0f
#define AFSR_16G_MAX 2048.0f


typedef struct
{
	struct
	{
		volatile int16_t ax, ay, az, gx, gy, gz;
	} rawData;

	struct
	{
		volatile float ax, ay, az, gx, gy, gz;
	} sensorData;

	struct
	{
		uint8_t aRange, gRange;
		uint16_t aMaxRange, gMaxRange;
	} config;
} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_GetRawData(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_GetSensorData(I2C_HandleTypeDef *I2Cx, MPU6050_t *mpu);
void MPU6050_SetAccRange(I2C_HandleTypeDef *I2Cx,  MPU6050_t *mpu);
void MPU6050_SetGyroRange(I2C_HandleTypeDef *I2Cx,  MPU6050_t *mpu);

#endif /* INC_MPU6050_H_ */
