/*
 * vars.h
 *
 *  Created on: Jan 24, 2022
 *      Author: halo9
 */

#ifndef INC_VARS_H_
#define INC_VARS_H_

uint8_t dbuf_index = 0;
uint8_t prev_index = 0;
uint16_t buffer_index = HEADER_SIZE;
uint16_t tmp_buffer_index = 0;

int16_t DDC1_Buffer1[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC1_Buffer2[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];

int16_t DDC2_Buffer1[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC2_Buffer2[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];

int16_t *buffers1[2] = {DDC1_Buffer1, DDC1_Buffer2};

int16_t *buffers2[2] = {DDC2_Buffer1, DDC2_Buffer2};

#endif /* INC_VARS_H_ */
