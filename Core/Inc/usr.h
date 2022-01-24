/*
 * usr.h
 *
 *  Created on: Jan 3, 2022
 *      Author: halo9
 */

#ifndef INC_USR_H_
#define INC_USR_H_

typedef enum
{
	USR_OK       = 0x00U,
	USR_ERR      = 0x01U
} USR_StatusTypeDef;

typedef enum
{
  USR_UNLOCKED = 0x00U,
  USR_LOCKED   = 0x01U
} USR_LockTypeDef;

#endif /* INC_USR_H_ */
