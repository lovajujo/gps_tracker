/*
 * neo7m_gps.c
 *
 *  Created on: Oct 2, 2023
 *      Author: lovzs
 */
#include "neo7m_gps.h"
const uint8_t disableGLL[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GLL_ID,0x00,GLL_CHECKSUM_A,GLL_CHECKSUM_B};//??
const uint8_t disableGSV[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GSV_ID,0x00,GSV_CHECKSUM_A,GSV_CHECKSUM_B};
const uint8_t disableGSA[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GSA_ID,0x00,GSA_CHECKSUM_A,GSA_CHECKSUM_B};
const uint8_t disableGGA[] = {CFG_HEADER_1,CFG_HEADER_2,CFG_MSG_ID_1,CFG_MSG_ID_2,CFG_MSG_LENGTH,CFG_MSG_CLASS,CFG_MSG_GGA_ID,0x00,GGA_CHECKSUM_A,GGA_CHECKSUM_B};
uint8_t cfg_msg[][5]={disableGLL, disableGSV, disableGGA, disableGSA};
uint8_t cfgg_rate[]={0xB5,0x62,0x06,0x08,0x06,0,0,0x01,0x00,0x01,0x00,0,0};

uint8_t GPS_Init(USART_TypeDef *UARTx)
{
	HAL_UART_Receive_IT(&UARTx, &uart_data, 1);
}
void GPS_Config_Rate(USART_TypeDef *UARTx, uint16_t measurement_rate)
{
	uint8_t highbyte=0u;
	uint8_t lowbyte=0u;
	uint8_t c_s_A=0u;
	uint8_t c_s_B=0u;
	//TODO checksum for msg rates
	switch (measurement_rate) {
		case MEASUREMENT_RATE_1HZ:
			highbyte=(MEASUREMENT_RATE_1HZ>>8) & MASK;
			lowbyte=MEASUREMENT_RATE_1HZ & MASK;
			break;
		case MEASUREMENT_RATE_2HZ:
			highbyte=(MEASUREMENT_RATE_2HZ>>8) & MASK;
			lowbyte=MEASUREMENT_RATE_2HZ & MASK;
		case MEASUREMENT_RATE_4HZ:
			highbyte=(MEASUREMENT_RATE_4HZ>>8) & MASK;
			lowbyte=MEASUREMENT_RATE_4HZ & MASK;
			break;
		case MEASUREMENT_RATE_5HZ:
			highbyte=(MEASUREMENT_RATE_5HZ>>8) & MASK;
			lowbyte=MEASUREMENT_RATE_5HZ & MASK;
			break;
		case MEASUREMENT_RATE_10HZ:
			highbyte=(MEASUREMENT_RATE_10HZ>>8) & MASK;
			lowbyte=MEASUREMENT_RATE_10HZ & MASK;
			break;
		default:
			highbyte=(MEASUREMENT_RATE_10HZ>>8) & MASK;
			lowbyte=MEASUREMENT_RATE_10HZ & MASK;
			break;
	}
	cfg_msg[5]=highbyte;
	cfg_msg[6]=lowbyte;
}
void GPS_Transmit(USART_TypeDef *UARTx, char *message)
{

}
void GPS_Receive(USART_TypeDef *UARTx, char *new_data)
{

}
