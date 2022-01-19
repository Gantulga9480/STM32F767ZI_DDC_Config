/*
 * shift_register.c
 *
 *  Created on: Jan 19, 2022
 *      Author: halo9
 */

#include "main.h"
#include "shift_register.h"

void init_sbuf()
{
	HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin|SBUF_CLR_Pin|SBUF_SCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, SBUF_CLR_Pin, GPIO_PIN_SET);
}
void clear_sbuf()
{
	HAL_GPIO_WritePin(GPIOA, SBUF_CLR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, SBUF_CLR_Pin, GPIO_PIN_SET);
}
void send_sbuf(uint16_t sdata)
{
	uint16_t sdata_temp;
	uint16_t i;

	sdata_temp = sdata;

	for(i=0;i<10;i++)
	{
		if((sdata_temp&0x0001))
		{
			HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin,GPIO_PIN_SET);
		}
		else
			HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, SBUF_SCLK_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, SBUF_SCLK_Pin,GPIO_PIN_RESET);
		sdata_temp = sdata_temp >> 1;
		//HAL_GPIO_WritePin(GPIOA, SBUF_DATA_Pin|SBUF_CLR_Pin|SBUF_SCLK_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(SBUF_LCLK_GPIO_Port, SBUF_LCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SBUF_LCLK_GPIO_Port, SBUF_LCLK_Pin, GPIO_PIN_RESET);
}
