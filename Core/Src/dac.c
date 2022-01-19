/*
 * dac.c
 *
 *  Created on: Jan 19, 2022
 *      Author: halo9
 */
#include "main.h"
#include "dac.h"

USR_StatusTypeDef USR_DAC_Init()
{
	HAL_GPIO_WritePin(GPIOC, DAC_SYNC_Pin|DAC_SCLK_Pin|DAC_DIN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DAC_LDAC_Pin, GPIO_PIN_SET);
	return USR_DAC_Control(DAC_MODE_LDAC, DAC_LDAC_SINGLE);
}

void USR_DAC_UdpHandler(uint8_t *udp_data)
{
	uint16_t value;

	/* Extracting four digit DAC value from received udp buffer */
	value = (udp_data[1] - '0') * 1000 + (udp_data[2] - '0') * 100 + (udp_data[3] - '0') * 10 + (udp_data[4] - '0');

	/* Handle UDP -> DAC operatoin here */
	USR_DAC_Write(DAC_OUT_A, value);
	USR_DAC_Write(DAC_OUT_B, value);
	USR_DAC_Write(DAC_OUT_C, value);
	USR_DAC_Write(DAC_OUT_D, value);
	USR_DAC_Write(DAC_OUT_E, value);
	USR_DAC_Write(DAC_OUT_F, value);
	USR_DAC_Write(DAC_OUT_G, value);
	USR_DAC_Write(DAC_OUT_H, value);
	/* Handle UDP -> DAC operatoin here */
}

USR_StatusTypeDef USR_DAC_Write(uint16_t channel, uint16_t value)
{
	if (value <= DAC_MAX)
	{
#ifdef __AD5308
		value = value << (DAC_DATA_BIT_RES - DAC_RESOLUTION);
		value = value | channel | DAC_WRITE;
#endif
#ifdef __AD5318
		value = value << (DAC_DATA_BIT_RES - DAC_RESOLUTION);
		value = value | channel | DAC_WRITE;
#endif
#ifdef __AD5328
		value = value | channel | DAC_WRITE;
#endif
		USR_DAC_SerialWrite(value);
		USR_DAC_PulseLDAC();
		return USR_OK;
	}
	else return USR_ERR;
}

USR_StatusTypeDef USR_DAC_Control(uint16_t mode, uint16_t control)
{
	control = DAC_CONTROL | mode | control;
	USR_DAC_SerialWrite(control);
	return USR_OK;
}

void USR_DAC_SerialWrite(uint16_t value)
{
	uint8_t i = 0;
	HAL_GPIO_WritePin(GPIOC, DAC_SYNC_Pin, GPIO_PIN_RESET);
	for(;i<16;i++)
	{
		if((value & 0x8000) != 0x00) HAL_GPIO_WritePin(GPIOC, DAC_DIN_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOC, DAC_DIN_Pin, GPIO_PIN_RESET);
		USR_DAC_PulseCLK();
		value = value << 1;
	}
	HAL_GPIO_WritePin(GPIOC, DAC_SYNC_Pin, GPIO_PIN_SET);
}

void USR_DAC_PulseCLK()
{
	HAL_GPIO_WritePin(GPIOC, DAC_SCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, DAC_SCLK_Pin, GPIO_PIN_SET);
}
void USR_DAC_PulseLDAC()
{
	HAL_GPIO_WritePin(GPIOA, DAC_LDAC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, DAC_LDAC_Pin, GPIO_PIN_SET);
}
