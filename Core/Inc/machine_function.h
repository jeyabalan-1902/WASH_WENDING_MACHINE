/*
 * machine_function.h
 *
 *  Created on: Jan 10, 2025
 *      Author: kjeyabalan
 */

#ifndef INC_MACHINE_FUNCTION_H_
#define INC_MACHINE_FUNCTION_H_


#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "TM1637.h"


#define RESET_DELAY_MS 2000

typedef enum {
    IDLE,
    COIN_1,
    COIN_2,
	COIN_3,
    COIN2_OPERATION,
	COIN2_OP2,
	COIN2_OP3,
	COIN3_OPERATION,
    DISPLAY_OFF,
    BACK_TO_IDLE
} machineState;

extern machineState current_state;

void StateMachine_Run(void);
void DisplayDashes(void);
void Display_fifty(void);
void TM1637_Countdown_20Sec(void);
void Display_1dhiram(void);
void TM1637_DisplayClear(void);
void Display_2dhiram(void);
void ProcessCoin3Operation(void);
void ProcessCoin2Operation(void);


#endif /* INC_MACHINE_FUNCTION_H_ */
