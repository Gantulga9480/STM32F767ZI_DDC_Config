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
uint16_t buffer_index = 5;
uint16_t tmp_buffer_index = 0;

int16_t DDC_Buffer1[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE + 3];
int16_t DDC_Buffer2[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE + 3];

int16_t *buffers[2] = {DDC_Buffer1, DDC_Buffer2};

#endif /* INC_VARS_H_ */
