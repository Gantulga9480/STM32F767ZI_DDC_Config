/*
 * coder.h
 *
 *  Created on: Jan 27, 2022
 *      Author: halo9
 */

#ifndef INC_CODER_H_
#define INC_CODER_H_

#include "stdint.h"

#define START_1_D 2356
#define START_2_D 15
#define START_3_D 110
#define START_4_D 4426
#define START_5_D 1000

#define POWER_ON_1_D 110
#define POWER_ON_2_D 14

#define POWER_OFF_1_D 100
#define POWER_OFF_2_D 705

#define TRIGGER_ON_1_D 110
#define TRIGGER_ON_2_D 15

#define TRIGGER_OFF_1_D 30
#define TRIGGER_OFF_2_D 15

#define CH_START_D 55
#define CH_D 1137

void USR_CODER_StartCode();
void USR_CODER_Long(uint16_t code, uint16_t delay);
void USR_CODER_Short(uint16_t *code, uint16_t length);
void USR_CODER_UdpHandler(uint8_t *udp_data);

#endif /* INC_CODER_H_ */
