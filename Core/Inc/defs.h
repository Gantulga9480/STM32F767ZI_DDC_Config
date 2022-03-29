/*
 * defs.h
 *
 *  Created on: Jan 24, 2022
 *      Author: halo9
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_

#define UDP_SEND_PORT 10
#define UDP_RECEIVE_PORT 11

#define BUFFER_COUNT 2
#define BUFFER_SIZE 1000 // 2000 byte
#define HEADER_SIZE 2    // 4 byte
#define FOOTER_SIZE 2    // 4 byte
#define PACKET_SIZE ((BUFFER_SIZE / 2) + HEADER_SIZE) * 2 // 16 bit size to 8 bit size

#define HEADER (('B' << 8) + 'A')    // ABAB
#define FOOTER (('D' << 8) + 'C')    // CDCD

#define SW_INT 0
#define SW_EXT 1

#endif /* INC_DEFS_H_ */
