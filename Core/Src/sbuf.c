/*
 * shift_register.c
 *
 *  Created on: Jan 19, 2022
 *      Author: halo9
 */

#include "main.h"
#include "sbuf.h"

void USR_SBUF_Init()
{
	HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin|SBUF_CLR_Pin|SBUF_SCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, SBUF_CLR_Pin, GPIO_PIN_SET);
}
void USR_SBUF_UdpHandler(uint8_t *udp_data)
{
	uint16_t value = 0, i = 0;
	for (; i < 10; i++)
	{
		if ((udp_data[i+1] - '0') <= 1)
		{
			value += (udp_data[i+1] - '0') << (9 - i);
		}
	}
	USR_SBUF_Write(value);
}
void USR_SBUF_Clear()
{
	HAL_GPIO_WritePin(GPIOA, SBUF_CLR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, SBUF_CLR_Pin, GPIO_PIN_SET);
}
void USR_SBUF_Serialwrite(uint16_t sd)
{
	uint8_t i = 0;
	for(;i<10;i++)
	{
		if((sd & 0x0001)) HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin,GPIO_PIN_RESET);
		USR_SBUF_PulseSCLK();
		sd = sd >> 1;
	}
}
void USR_SBUF_Write(uint16_t sdata)
{
	USR_SBUF_Serialwrite(sdata);
	USR_SBUF_PulseLCLK();
}
void USR_SBUF_PulseSCLK()
{
	HAL_GPIO_WritePin(GPIOA, SBUF_SCLK_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, SBUF_SCLK_Pin,GPIO_PIN_RESET);
}
void USR_SBUF_PulseLCLK()
{
	HAL_GPIO_WritePin(SBUF_LCLK_GPIO_Port, SBUF_LCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SBUF_LCLK_GPIO_Port, SBUF_LCLK_Pin, GPIO_PIN_RESET);
}
