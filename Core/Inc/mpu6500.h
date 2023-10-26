/*
 * mpu6500.h
 *
 *  Created on: Jul 5, 2023
 *      Author: lovzs
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define MPU6500_ADDRESS 0x68
#define USER_CONTROL 106
#define ACCEL_XOUT_H 59
#define GYRO_CONFIG 27
#define ACCEL_CONFIG 28
#define MSB_SET 0x80
#define CS_SELECT 0
#define CS_DESELECT 1
#define PWR_MGMT_1 107
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

#define MFSF 0.6f


typedef struct
{
	struct
	{
		int16_t ax, ay, az, gx, gy, gz;
	} rawData;

	struct
	{
		float ax, ay, az, gx, gy, gz;
	} sensorData;

	struct
	{
		uint8_t aRange, gRange;
	} config;
} MPU6500_t;

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu);
void MPU6500_GetRawData(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu);
void MPU6500_GetData(I2C_HandleTypeDef *I2Cx, MPU6500_t *mpu);
float MPU6500_SetAccRange(I2C_HandleTypeDef *I2Cx,  MPU6500_t *mpu);
float MPU6500_SetGyroRange(I2C_HandleTypeDef *I2Cx,  MPU6500_t *mpu);

#endif /* INC_MPU6500_H_ */
