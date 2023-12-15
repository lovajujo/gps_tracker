/*
 * hmc5883l.h
 *
 *  Created on: Jul 18, 2023
 *      Author: lovzs
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

#include "stm32l4xx_hal.h"

#define HMC_READ 0x3C
#define HMC_WRITE 0x3D
#define HMC_CONFIG_A 0x00
#define HMC_CONFIG_B 0x01
#define HMC_MODE 0x02
#define HMC_X_DATA_H 0x03

#define HMC_GAIN_1370 0x00
#define HMC_GAIN_1090 0x20
#define HMC_GAIN_820 0x40
#define HMC_GAIN_660 0x60
#define HMC_GAIN_390 0x80
#define HMC_GAIN_330 0xA0
#define HMC_GAIN_230 0xE0

#define HMC_SAMPLE_RATE_0_75 0x00
#define HMC_SAMPLE_RATE_1_5 0x04
#define HMC_SAMPLE_RATE_3 0x08
#define HMC_SAMPLE_RATE_7_5 0x0C
#define HMC_SAMPLE_RATE_15 0x10
#define HMC_SAMPLE_RATE_30 0x14
#define HMC_SAMPLE_RATE_75 0x1C

#define HMC_MODE_CONTINOUS 0x00
#define HMC_MODE_SINGLE 0x01
#define HMC_MODE_IDLE 0x10

#define BUFFER_SIZE 6

typedef struct
{
	volatile uint16_t x_data, y_data, z_data;
	uint8_t sample_rate, gain, mode;
}HMC5883L_t;

uint8_t HMC5883L_Init(I2C_HandleTypeDef *i2c, HMC5883L_t *hmc);
void HMC5883L_ReadData(I2C_HandleTypeDef *i2c, HMC5883L_t *hmc);

#endif /* INC_HMC5883L_H_ */
