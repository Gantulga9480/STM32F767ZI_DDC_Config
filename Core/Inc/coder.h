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

#define CH1_F_START 1018
#define CH2_F_START 1019

#define CH_F_START_D 100
#define CH1_F_150_2_D 2211
#define CH1_F_151_2_D 2205
#define CH1_F_152_2_D 2196
#define CH1_F_153_2_D 2205
#define CH1_F_154_2_D 2213
#define CH1_F_155_2_D 2205
#define CH1_F_156_2_D 2196
#define CH1_F_157_2_D 2205
#define CH1_F_158_2_D 2212
#define CH1_F_159_2_D 2212
#define CH1_F_160_2_D 2212
#define CH1_F_161_2_D 2205
#define CH1_F_162_2_D 2205
#define CH1_F_163_2_D 2205
#define CH1_F_164_2_D 2213
#define CH1_F_165_2_D 2212
#define CH1_F_166_2_D 2205
#define CH1_F_167_2_D 2205
#define CH1_F_168_2_D 2212
#define CH1_F_169_2_D 2205
#define CH1_F_170_2_D 2212

#define CH2_F_150_2_D 2213
#define CH2_F_151_2_D 2204
#define CH2_F_152_2_D 2196
#define CH2_F_153_2_D 2205
#define CH2_F_154_2_D 2204
#define CH2_F_155_2_D 2196
#define CH2_F_156_2_D 2205
#define CH2_F_157_2_D 2212
#define CH2_F_158_2_D 2196
#define CH2_F_159_2_D 2204
#define CH2_F_160_2_D 2195
#define CH2_F_161_2_D 2196
#define CH2_F_162_2_D 2205
#define CH2_F_163_2_D 2205
#define CH2_F_164_2_D 2213
#define CH2_F_165_2_D 2205
#define CH2_F_166_2_D 2205
#define CH2_F_167_2_D 2205
#define CH2_F_168_2_D 2212
#define CH2_F_169_2_D 2205
#define CH2_F_170_2_D 2213

void USR_CODER_Start();
void USR_CODER_PowerOn();
void USR_CODER_PowerOff();
void USR_CODER_TriggerOn();
void USR_CODER_TriggerOff();
void USR_CODER_StateSend();
void USR_CODER_ChannelCode(uint8_t index);
void USR_CODER_ChannelFreq(uint8_t channel, uint8_t index);
void USR_CODER_Long(uint16_t code, uint16_t delay);
void USR_CODER_Short(const uint16_t *code, uint16_t length);
void USR_CODER_UdpHandler(uint8_t *udp_data);
void USR_CODER_StateSend();

#endif /* INC_CODER_H_ */
