/*
 * neo7m_gps.h
 *
 *  Created on: 2023. okt. 2.
 *      Author: lovzs
 */

#ifndef INC_NEO7M_GPS_H_
#define INC_NEO7M_GPS_H_

#include "stm32l4xx_hal.h"

#define MEASUREMENT_RATE_1HZ 0xE83
#define MEASUREMENT_RATE_2HZ 0x1F4
#define MEASUREMENT_RATE_4HZ 0xFA
#define MEASUREMENT_RATE_5HZ 0xC8
#define MEASUREMENT_RATE_10HZ 0x64
#define CFG_HEADER_1 0xB5
#define CFG_HEADER_2 0x62
#define CFG_RATE_ID_1 0x06
#define CFG_RATE_ID_2 0x08
#define CFG_RATE_LENGTH 0x06
#define CFG_MSG_ID_1 0x06
#define CFG_MSG_ID_2 0x01
#define CFG_MSG_LENGTH 0x03
#define CFG_MSG_CLASS 0xF0
#define CFG_MSG_GGA_ID 0x00
#define CFG_MSG_GLL_ID 0x01
#define CFG_MSG_GSA_ID 0x02
#define CFG_MSG_GSV_ID 0x03
#define GGA_CHECKSUM_A 0xFA
#define GGA_CHECKSUM_B 0x05
#define GLL_CHECKSUM_A 0xFB
#define GLL_CHECKSUM_B 0x07
#define GSA_CHECKSUM_A 0xFC
#define GSA_CHECKSUM_B 0x09
#define GSV_CHECKSUM_A 0xFD
#define GSV_CHECKSUM_B 0x0B
#define MASK 0xFF

const uint8_t disableGLL[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GLL_ID,0x00,GLL_CHECKSUM_A,GLL_CHECKSUM_B};//??
const uint8_t disableGSV[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GSV_ID,0x00,GSV_CHECKSUM_A,GSV_CHECKSUM_B};
const uint8_t disableGSA[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GSA_ID,0x00,GSA_CHECKSUM_A,GSA_CHECKSUM_B};
const uint8_t disableGGA[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GGA_ID,0x00,GGA_CHECKSUM_A,GGA_CHECKSUM_B};
const uint8_t cfg_msg[][4]={disableGLL, disableGSV, disableGSA, disableGGA};
uint8_t cfg_rate[]={0xB5,0x62,0x06,0x08,0x06,0,0,0x01,0x00,0x01,0x00,0,0};
volatile uint8_t cfg_msg_index;

uint8_t GPS_Init(USART_TypeDef *huart);
void GPS_Config_Rate(USART_TypeDef *huart, uint16_t measurement_rate);
void GPS_Transmit(USART_TypeDef *huart, uint8_t *message);
void GPS_Receive(USART_TypeDef *huart, char *new_data);
void Calc_checksum();

#endif /* INC_NEO7M_GPS_H_ */
