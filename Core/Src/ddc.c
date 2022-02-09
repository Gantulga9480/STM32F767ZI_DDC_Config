/*
 * ddc.c
 *
 *  Created on: Dec 30, 2021
 *      Author: halo9
 */

#include "main.h"
#include "ddc.h"
#include "string.h"
#include "math.h"

int16_t DDC_FIR_COEF[RCF_SIZE] = {
    0, 0, 0, -1, 1, -1, 1, -1, 1, -1, 1, 0, 0, 0, -1, 1, -2, 3, -3, 4, -4, 5, -5, 4, -4, 3, -2, 0, 2, -4, 7, -9, 12, -14, 16, -18, 19, -19, 18, -16, 13, -7, 0, 10, -23, 41, -67, 110, -201, 633, 633, -201, 110, -67, 41, -23, 10, 0, -7, 13, -16, 18, -19, 19, -18, 16, -14, 12, -9, 7, -4, 2, 0, -2, 3, -4, 4, -5, 5, -4, 4, -3, 3, -2, 1, -1, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0,
};

void USR_DDC_Init(DDC_ConfigTypeDef conf)
{
	hardReset();                                               // Hard reset
	if (conf.FIR) write_rcf_ram();                             // Write FIR rams if using
	//else flush_RCF_ram();
	flush_iq_ram();											   // Flush remaining I and Q data in DDC rams
	write_ddc(DDC_MODE,               DDC_SOFT_RESET);         // Soft reset
	write_ddc(DDC_NCO_MODE,           conf.NCO_Mode);          // NCO active, Phase-Amplitude Dither
	write_ddc(DDC_NCO_SYNC_MASK,      0x00FFFFFFFF);           // NCO Sync mask
	write_ddc(DDC_NCO_FREQUENCY,      conf.NCO_Frequency);     // NCO Frequency
	write_ddc(DDC_NCO_PHASE_OFFSET,   conf.NCO_PhaseOffset);   // NCO Phase offset
	write_ddc(DDC_CIC2_SCALE,         conf.CIC2_Scale);        // In/CIC2 control
	write_ddc(DDC_CIC2_DECIMATION,    conf.CIC2_Decimation);   // CIC2 Decimation N-1
	write_ddc(DDC_CIC5_SCALE,         conf.CIC5_Scale);        // CIC5 Scale
	write_ddc(DDC_CIC5_DECIMATION,    conf.CIC5_Decimation);   // CIC5 Decimation N-1
	write_ddc(DDC_RCF_SCALE,          conf.RCF_Scale);  	   // Out/RCF control
	write_ddc(DDC_RCF_DECIMATION,     conf.RCF_Decimation);    // RCF Decimation N-1
	write_ddc(DDC_RCF_ADDRESS_OFFSET, conf.RCF_AddressOffset); // RCF address offset
	write_ddc(DDC_RCF_FILTER_TAPS,    conf.RCF_FilterTaps);    // N taps N-1
	write_ddc(0x30D,                  DDC_RESERVED);           // Reserved
	write_ddc(DDC_MODE,               conf.DDC_Mode);   	   // Operating mode
}
void USR_DDC_UdpHandler(uint8_t *udp_data)
{
	uint16_t addr = 0, i = 0;
	uint64_t value = 0;
	DDC_ConfigTypeDef ddc_conf;
	int16_t addr_start = 1;
	int16_t val_start = 4;
	for (; i < 14; i++)
	{
		addr = get_addr(udp_data, addr_start);
		value = get_value(udp_data, val_start);
		if (addr == DDC_MODE)
			ddc_conf.DDC_Mode = value;
		else if (addr == DDC_NCO_MODE)
			ddc_conf.NCO_Mode = value;
		else if (addr == DDC_NCO_SYNC_MASK)
			ddc_conf.NCO_SyncMask = value;
		else if (addr == DDC_NCO_FREQUENCY)
			ddc_conf.NCO_Frequency = value;
		else if (addr == DDC_NCO_PHASE_OFFSET)
			ddc_conf.NCO_PhaseOffset = value;
		else if (addr == DDC_CIC2_SCALE)
			ddc_conf.CIC2_Scale = value;
		else if (addr == DDC_CIC2_DECIMATION)
			ddc_conf.CIC2_Decimation = value;
		else if (addr == DDC_CIC5_SCALE)
			ddc_conf.CIC5_Scale = value;
		else if (addr == DDC_CIC5_DECIMATION)
			ddc_conf.CIC5_Decimation = value;
		else if (addr == DDC_RCF_SCALE)
			ddc_conf.RCF_Scale = value;
		else if (addr == DDC_RCF_DECIMATION)
			ddc_conf.RCF_Decimation = value;
		else if (addr == DDC_RCF_ADDRESS_OFFSET)
			ddc_conf.RCF_AddressOffset = value;
		else if (addr == DDC_RCF_FILTER_TAPS)
			ddc_conf.RCF_FilterTaps = value;
		else
			ddc_conf.FIR = value;
		addr_start += 16;
		val_start += 16;
	}
	USR_DDC_Init(ddc_conf);
}
void flush_iq_ram()
{
	uint16_t addr = 0x100;
	for (; addr <= 0x1FF; addr++)
	{
		write_ddc(addr, 0x00);
	}
}
void flush_RCF_ram()
{
	uint16_t addr = 0x00;
	for (; addr <= 0x0FF; addr++)
	{
		write_ddc(addr, 0x01);
	}
}
void write_rcf_ram()
{
	uint16_t addr = 0;
	for (; addr < RCF_SIZE; addr++)
	{
		write_ddc(addr, DDC_FIR_COEF[addr]);
	}
}
uint64_t read_ddc(uint16_t address)
{
	uint64_t DR0,DR1,DR2,DR3,DR4;
	uint8_t LAR,AMR;
	uint64_t result = 0;
	LAR = (address & 0x00FF);
	AMR = (address >> 8);
	uPort_write(7,AMR);
	uPort_write(6,LAR);
	DR0 = uPort_read(0) & 0xFF;
	DR1 = uPort_read(1) & 0xFF;
	DR2 = uPort_read(2) & 0xFF;
	DR3 = uPort_read(3) & 0xFF;
	DR4 = uPort_read(4) & 0xFF;
	result = DR0 + (DR1 << 8) + (DR2 << 16) + (DR3 << 24) + (DR4 << 32);
	return result;
}
uint64_t write_ddc(uint16_t address, uint64_t data)
{
	uint8_t DR0,DR1,DR2,DR3,DR4;
	uint8_t LAR,AMR;
	uint64_t result = 0;
	LAR = (address & 0x00FF);
	AMR = (address >> 8);
	DR4 = (data & 0xFF00000000) >> 32;
	DR3 = (data & 0x00FF000000) >> 24;
	DR2 = (data & 0x0000FF0000) >> 16;
	DR1 = (data & 0x000000FF00) >> 8;
	DR0 = (data & 0x00000000FF);
	uPort_write(7,AMR);
	uPort_write(6,LAR);
	uPort_write(4,DR4);
	uPort_write(3,DR3);
	uPort_write(2,DR2);
	uPort_write(1,DR1);
	uPort_write(0,DR0);
	return result;
}
uint8_t uPort_write(uint8_t address, uint8_t data)
{
	uint16_t temp,i;
	uint8_t result;
	DDC_READY_FLAG = 0;
	i = 500;
	ddc_rdy_int_mode(GPIO_MODE_IT_FALLING);
	data_port_mode(GPIO_MODE_OUTPUT_PP);
	temp = address;
	temp = temp << 10;
	temp = temp & 0b0001110000000000;
	GPIOC -> ODR &= 0b1110001111111111;
	GPIOC -> ODR |= temp;
	GPIOG -> ODR &= 0xFF00;
	GPIOG -> ODR |= data;
	HAL_GPIO_WritePin(GPIOC, DDC1_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, DDC_WR_Pin, GPIO_PIN_RESET);
	while(DDC_READY_FLAG == 0 && i > 0)
	{
		i--;
	}
	if(i > 0)
		result = 1;
	else
		result = 0;
	HAL_GPIO_WritePin(GPIOG, DDC_WR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, DDC1_CS_Pin, GPIO_PIN_SET);
	return result;
}
uint16_t uPort_read(uint8_t address)
{
	uint16_t temp,i;
	uint8_t result;
	DDC_READY_FLAG = 0;
	i = 500;
	ddc_rdy_int_mode(GPIO_MODE_IT_RISING);
	data_port_mode(GPIO_MODE_INPUT);
	temp = address;
	temp = temp << 10;
	temp = temp & 0b0001110000000000;
	GPIOC -> ODR &= 0b1110001111111111;
	GPIOC -> ODR |= temp;

	HAL_GPIO_WritePin(GPIOC, DDC1_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, DDC_RD_Pin, GPIO_PIN_RESET);
	while(DDC_READY_FLAG == 0 && i > 0)
	{
		i--;
	}
	if(i > 0)
	{
		temp = GPIOG -> IDR;
		result = temp & 0x00FF;
	}
	else
		result = 0;
	HAL_GPIO_WritePin(GPIOG, DDC_RD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, DDC1_CS_Pin, GPIO_PIN_SET);
	return result;
}
void data_port_mode(uint32_t port_mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(port_mode == GPIO_MODE_OUTPUT_PP)
	{
		GPIO_InitStruct.Pin = DDC_CD0_Pin|DDC_CD1_Pin|DDC_CD2_Pin|DDC_CD3_Pin
		                          |DDC_CD4_Pin|DDC_CD5_Pin|DDC_CD6_Pin|DDC_CD7_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	}
	else if(port_mode == GPIO_MODE_INPUT)
	{
		GPIO_InitStruct.Pin = DDC_CD0_Pin|DDC_CD1_Pin|DDC_CD2_Pin|DDC_CD3_Pin
				                          |DDC_CD4_Pin|DDC_CD5_Pin|DDC_CD6_Pin|DDC_CD7_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	}
}
void ddc_rdy_int_mode(uint32_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DDC1_RDY_Pin;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DDC1_RDY_GPIO_Port, &GPIO_InitStruct);
}
void hardReset()
{
	HAL_GPIO_WritePin(GPIOG, DDC_WR_Pin|DDC_RD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, DDC1_CS_Pin|DDC1_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, DDC1_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, DDC1_RST_Pin, GPIO_PIN_SET);
}
uint16_t get_addr(uint8_t *s, int16_t start)
{
	return (s[start] - '0') * 10 + (s[start+1] - '0') + 0x300;
}
uint64_t get_value(uint8_t *s, int16_t start)
{
	uint64_t b = 0;
	uint8_t i = 0;
	for (; i < 12; i++)
    {
	   b = b + (s[start+i] - '0') * pow(10, 11-i);
    }
	return b;
}
