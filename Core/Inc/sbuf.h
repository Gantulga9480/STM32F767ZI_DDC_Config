/*
 * shift_register.h
 *
 *  Created on: Jan 19, 2022
 *      Author: halo9
 */

#ifndef INC_SBUF_H_
#define INC_SBUF_H_

#include "stdint.h"

void USR_SBUF_Init();
void USR_SBUF_UdpHandler(uint8_t *udp_data);
void USR_SBUF_Write(uint16_t sdata);
void USR_SBUF_Serialwrite(uint16_t sd);

void USR_SBUF_Clear();
void USR_SBUF_PulseSCLK();
void USR_SBUF_PulseLCLK();

#endif /* INC_SBUF_H_ */
