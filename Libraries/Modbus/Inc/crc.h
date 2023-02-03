/*
 * crc.h
 *
 *  Created on: Feb 3, 2023
 *      Author: anh
 */

#ifndef CRC_H_
#define CRC_H_

#include "stdio.h"


#ifdef __cplusplus
extern "C"{
#endif

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);


#ifdef __cplusplus
}
#endif


#endif /* CRC_H_ */
