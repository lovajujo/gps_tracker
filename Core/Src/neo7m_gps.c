/*
 * neo7m_gps.c
 *
 *  Created on: Oct 2, 2023
 *      Author: lovzs
 */
#include "neo7m_gps.h"

uint8_t cfg_rate[]={CFG_HEADER_1,CFG_HEADER_2,CFG_RATE_ID_1,CFG_RATE_ID_2,CFG_RATE_LENGTH,EMPTY_BYTE,EMPTY_BYTE,CFG_RATE_NAV_RATE,EMPTY_BYTE,CFG_RATE_TIME_REF,EMPTY_BYTE,0,0};
const uint8_t cfg_msg[CFG_MSG_NUMBER][CFG_MSG_SIZE]={
		{CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,EMPTY_BYTE,CFG_MSG_CLASS,CFG_MSG_GLL_ID,EMPTY_BYTE,GLL_CHECKSUM_A,GLL_CHECKSUM_B},
		{CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,EMPTY_BYTE,CFG_MSG_CLASS,CFG_MSG_GSV_ID,EMPTY_BYTE,GSV_CHECKSUM_A,GSV_CHECKSUM_B},
		{CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,EMPTY_BYTE,CFG_MSG_CLASS,CFG_MSG_GSA_ID,EMPTY_BYTE,GSA_CHECKSUM_A,GSA_CHECKSUM_B},
		{CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,EMPTY_BYTE,CFG_MSG_CLASS,CFG_MSG_GGA_ID,EMPTY_BYTE,GGA_CHECKSUM_A,GGA_CHECKSUM_B}
};
uint8_t cfg_msg_index=0u;
GPS_t gps;

/**
 * initialize gps module
 * set message rate; disable unnecessary nmea sentences
 * highbyte, lowbyte: milliseconds are 16 bit, but ubx message contains 8 bit values-->mask
 */
HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *huart, uint16_t msg_rate, uint8_t disable_unused)
{
	gps.rx_cplt=0u;
	gps.gps_data=0u;
	gps.disable_unused=disable_unused;
	gps.measurement_rate=msg_rate;
	uint8_t highbyte;
	uint8_t lowbyte;
	highbyte=(gps.measurement_rate>>8) & MASK;
	lowbyte=gps.measurement_rate & MASK;
	cfg_rate[5]=highbyte;
	cfg_rate[6]=lowbyte;
	Calc_checksum(cfg_rate, sizeof(cfg_rate));
	GPS_Transmit(huart, cfg_rate, sizeof(cfg_rate));
	while(cfg_msg_index<4);
	return HAL_OK;
}

/**
 * send message
 */
void GPS_Transmit(UART_HandleTypeDef *huart, uint8_t *message, uint8_t size)
{
	HAL_UART_Transmit_DMA(huart, message, size);
}

/**
 * receive message
 */
void GPS_Receive(UART_HandleTypeDef *huart, uint8_t *buffer, uint8_t size)
{
	HAL_UART_Receive_DMA(huart, buffer, size);
}

/**
 * calculate checksum_a and checksum_b of ubx message
 *
 */
void Calc_checksum(uint8_t *message, uint8_t arraysize)
{
	uint8_t c_s_A=0u;
	uint8_t c_s_B=0u;
	uint8_t index;
	for(index=2; index<arraysize-2;index++)
	{
		c_s_A=(c_s_A+message[index]) & MASK;
		c_s_B=(c_s_A+c_s_B) & MASK;
	}
	message[arraysize-2]=c_s_A;
	message[arraysize-1]=c_s_B;
}

/**
 *
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(cfg_msg_index<4 && gps.disable_unused)
	{
		cfg_msg_index++;
		GPS_Transmit(huart, cfg_msg[cfg_msg_index-1], sizeof(cfg_msg[cfg_msg_index-1]));
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	gps.rx_cplt=1;
}
