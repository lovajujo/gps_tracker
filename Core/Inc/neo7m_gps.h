/*
 * neo7m_gps.h
 *
 *  Created on: 2023. okt. 2.
 *      Author: lovzs
 */

#ifndef INC_NEO7M_GPS_H_
#define INC_NEO7M_GPS_H_

#include "stm32l4xx_hal.h"

#define MEASUREMENT_RATE_1000MS 0xE83
#define MEASUREMENT_RATE_500MS 0x1F4
#define MEASUREMENT_RATE_250MS 0xFA
#define MEASUREMENT_RATE_200MS 0xC8
#define MEASUREMENT_RATE_100MS 0x64
#define CFG_HEADER_1 0xB5
#define CFG_HEADER_2 0x62
#define CFG_RATE_ID_1 0x06
#define CFG_RATE_ID_2 0x08
#define CFG_RATE_LENGTH 0x06
#define CFG_RATE_NAV_RATE 0x01
#define CFG_RATE_TIME_REF 0x01
#define CFG_MSG_ID_1 0x06
#define CFG_MSG_ID_2 0x01
#define CFG_MSG_LENGTH 0x03
#define CFG_MSG_CLASS 0xF0
#define CFG_MSG_GGA_ID 0x00
#define CFG_MSG_GLL_ID 0x01
#define CFG_MSG_GSA_ID 0x02
#define CFG_MSG_GSV_ID 0x03
#define GGA_CHECKSUM_A 0xFA
#define GGA_CHECKSUM_B 0x0F
#define GLL_CHECKSUM_A 0xFB
#define GLL_CHECKSUM_B 0x11
#define GSA_CHECKSUM_A 0xFC
#define GSA_CHECKSUM_B 0x13
#define GSV_CHECKSUM_A 0xFD
#define GSV_CHECKSUM_B 0x15
#define MASK 0xFF
#define EMPTY_BYTE 0x00
#define DISABLE_MESSAGES 1
#define ENABLE_MESSAGES 0
#define CFG_MSG_NUMBER 4
#define CFG_MSG_SIZE 11

typedef struct{
	uint16_t measurement_rate;
	volatile uint8_t gps_data, rx_cplt;
	uint8_t recommended_min_info;
} GPS_t;

extern GPS_t gps;

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *huart, uint16_t msg_rate, uint8_t recom_min_info);
void GPS_Transmit(UART_HandleTypeDef *huart, uint8_t *message, uint8_t size);
void GPS_Receive(UART_HandleTypeDef *huart, uint8_t *buffer, uint8_t size);
void Calc_checksum(uint8_t *message, uint8_t arraysize);

#endif /* INC_NEO7M_GPS_H_ */
