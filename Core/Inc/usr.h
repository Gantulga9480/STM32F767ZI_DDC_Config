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

typedef enum
{
	true  = 0x01,
	false = 0x00
} bool;

#endif /* INC_USR_H_ */
