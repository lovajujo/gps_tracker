/*
 * neo7m_gps.c
 *
 *  Created on: Oct 2, 2023
 *      Author: lovzs
 */
#include "neo7m_gps.h"



uint8_t GPS_Init(USART_TypeDef *huart, uint16_t measurement_rate, char byte_received)
{
	cfg_msg_index=0u;
	GPS_Config_Rate(&huart, measurement_rate);
	GPS_Receive(&huart, &byte_received);
	return 0;
}
void GPS_Config_Rate(USART_TypeDef *huart, uint16_t measurement_rate)
{
	cfg_msg[5]=(uint8_t)(measurement_rate>>8) & MASK;
	cfg_msg[6]=(uint8_t)measurement_rate & MASK;
	Calc_checksum();
	GPS_Transmit(&huart, *cfg_rate);
}
void GPS_Transmit(USART_TypeDef *huart, uint8_t *message)
{
	HAL_UART_Transmit_DMA(&huart, &message, sizeof(message));
}
void GPS_Receive(USART_TypeDef *huart, char *new_data)
{
	HAL_UART_Receive_DMA(&huart, &new_data, 1);
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
