/*
 * TM1637.h
 *
 *  Created on: Jun 9, 2024
 *      Author: arunrawat
 */

#ifndef INC_TM1637_H_
#define INC_TM1637_H_

#include "main.h"


void TM1637_WriteData (uint8_t Addr, uint8_t *data, int size);
void Delay_us (int time);

#endif /* INC_TM1637_H_ */
