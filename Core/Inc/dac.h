/*
 * dac.h
 *
 *  Created on: Jan 19, 2022
 *      Author: halo9
 */

#include "usr.h"

#ifndef INC_DAC_H_
#define INC_DAC_H_

#define __AD5328
// #define __AD5318
// #define __AD5308

#define DAC_DATA_BIT_RES           12
#define DAC_SHIFT_REG_DATA_BIT_RES 16

/* DAC address */
#define DAC_OUT_A 0 << 12
#define DAC_OUT_B 1 << 12
#define DAC_OUT_C 2 << 12
#define DAC_OUT_D 3 << 12
#define DAC_OUT_E 4 << 12
#define DAC_OUT_F 5 << 12
#define DAC_OUT_G 6 << 12
#define DAC_OUT_H 7 << 12

/* DAC control bit */
#define DAC_WRITE   0 << 15
#define DAC_CONTROL 1 << 15

/* DAC control modes */
#define DAC_MODE_GAIN_REF_SEL  0
#define DAC_MODE_LDAC          1 << 13
#define DAC_MODE_POWER_DOWN    2 << 13
#define DAC_MODE_RESET         3 << 13

/* LDAC mode options */
#define DAC_LDAC_LOW    0
#define DAC_LDAC_HIGH   1
#define DAC_LDAC_SINGLE 2

/* Use buffered reference of DACs */
#define DAC_BUF_GROUP_ABCD 1 << 2
#define DAC_BUF_GROUP_EFGH 2 << 2

/* Set gain of DACs 0 to 2*Vref */
#define DAC_GAIN_GROUP_ABCD 1 << 4
#define DAC_GAIN_GROUP_EFGH 2 << 4

/* Use VDD as a reference voltage */
#define DAC_VDD_GROUP_ABCD 1
#define DAC_VDD_GROUP_EFGH 2

/* Power down selected DAC channels */
#define DAC_POWER_DOWN_A 0b00000001
#define DAC_POWER_DOWN_B 0b00000010
#define DAC_POWER_DOWN_C 0b00000100
#define DAC_POWER_DOWN_D 0b00001000
#define DAC_POWER_DOWN_E 0b00010000
#define DAC_POWER_DOWN_F 0b00100000
#define DAC_POWER_DOWN_G 0b01000000
#define DAC_POWER_DOWN_H 0b10000000

/* DAC reset data and control bits */
#define DAC_RESET_DATA_CONTROL 1 << 12

#ifdef __AD5308
#define DAC_RESOLUTION 8
#define DAC_MAX 255
#endif

#ifdef __AD5318
#define DAC_RESOLUTION 10
#define DAC_MAX 1023
#endif

#ifdef __AD5328
#define DAC_RESOLUTION 12
#define DAC_MAX 4095
#endif

USR_StatusTypeDef USR_DAC_Init();
USR_StatusTypeDef USR_DAC_Write(uint16_t channel, uint16_t value);
USR_StatusTypeDef USR_DAC_Control(uint16_t mode, uint16_t control);
void USR_DAC_UdpHandler(uint8_t *udp_data);
void USR_DAC_SerialWrite(uint16_t value);
void USR_DAC_PulseCLK();
void USR_DAC_PulseLDAC();

#endif /* INC_DAC_H_ */
