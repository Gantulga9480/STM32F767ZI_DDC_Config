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
uint8_t CODE_DATA_READY = 0;

int16_t DDC_Buffer1[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer2[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer3[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer4[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer5[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];

int16_t *buffers[] = {DDC_Buffer1, DDC_Buffer2, DDC_Buffer3, DDC_Buffer4, DDC_Buffer5};

#endif /* INC_VARS_H_ */
