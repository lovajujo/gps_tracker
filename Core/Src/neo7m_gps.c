/*
 * neo7m_gps.c
 *
 *  Created on: Oct 2, 2023
 *      Author: lovzs
 */
#include "neo7m_gps.h"


uint8_t cfg_rate[]={0xB5,0x62,0x06,0x08,0x06,0,0,0x01,0x00,0x01,0x00,0,0};
volatile uint8_t cfg_msg_index=0u;

uint8_t GPS_Init(USART_TypeDef *huart, NEO7M_t *gps)
{
	GPS_Config_Rate(&huart, &gps);
	return 0;
}
void GPS_Config_Rate(USART_TypeDef *huart, NEO7M_t *gps)
{
	uint8_t highbyte;
	uint8_t lowbyte;
	highbyte=(gps->measurement_rate>>8) & MASK;
	lowbyte=gps->measurement_rate & MASK;
	cfg_rate[5]=highbyte;
	cfg_rate[6]=lowbyte;
	Calc_checksum();
	GPS_Transmit(&huart, cfg_rate);
}
void GPS_Transmit(USART_TypeDef *huart, uint8_t *message)
{
	HAL_UART_Transmit_DMA(&huart, &message, sizeof(message));
}

void Calc_checksum()
{
	uint8_t c_s_A=0u;
	uint8_t c_s_B=0u;
	uint8_t index;
	for(index=2; index<11;index++)
	{
		c_s_A=(c_s_A+cfg_rate[index]) & MASK;
		c_s_B=(c_s_A+c_s_B) & MASK;
	}
	cfg_rate[11]=c_s_A;
	cfg_rate[12]=c_s_B;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(cfg_msg_index<4)
	{
		cfg_msg_index++;
		GPS_Transmit(&huart, cfg_msg[cfg_msg_index-1]);
	}
}

