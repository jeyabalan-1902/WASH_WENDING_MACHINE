/*
 * machine_function.c
 *
 *  Created on: Jan 10, 2025
 *      Author: kjeyabalan
 */


#include "machine_function.h"

machineState current_state = IDLE;
extern volatile uint32_t current_time_ms;
static uint32_t coin_pulse_count;
extern volatile uint8_t countdown_active;
static uint32_t last_pulse_time;
static uint8_t process_pulse;     // Flag to indicate pulses need processing
extern uint8_t initial_display_done;
extern uint8_t state;            // State variable to manage tasks
extern uint32_t task_start_time;

extern uint8_t digit_map[13];


void StateMachine_Run(void)
{
    static uint32_t reset_time = 0;

    switch (current_state)
    {
        case IDLE:
            DisplayDashes();

            static uint32_t reset_time = 0;

			if (process_pulse)
			{
				if (coin_pulse_count == 1)
				{
					printf("coin pulse 1\n\r");
					current_state = COIN_1;
					initial_display_done = 0;
				}
				else if (coin_pulse_count == 2)
				{
					printf("coin pulse 2\n\r");
					current_state = COIN_2;
					initial_display_done = 0;
				}
//				else if (coin_pulse_count >= 3)
//				{
//					printf("coin pulse 3\n\r");
//					//current_state = PROCESS_3_PULSES;
//				}

				// Start the reset timer
				reset_time = current_time_ms;
				process_pulse = 0; // Clear processing flag
			}

			// Reset the pulse count after RESET_DELAY_MS
			if ((current_time_ms - reset_time) >= RESET_DELAY_MS && coin_pulse_count > 0)
			{
				coin_pulse_count = 0;  // Reset pulse count
				last_pulse_time = 0;   // Reset last pulse time
				// Deactivate countdown
			}

            break;

        case COIN_1:
        	if (!initial_display_done)
			  {
				  Display_fifty(); // Display "50"
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1; // Mark initial display as done
			  }

			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_SET); // Activate relay signal
				  if (HAL_GetTick() - task_start_time < 20000) // Check if 20 seconds haven't elapsed
				  {
					  TM1637_Countdown_20Sec(); // Update the countdown
				  }
				  else
				  {
					  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET); // Deactivate relay signal
					  printf("return to IDLE\n\r");

					  current_state = IDLE;
				  }

			  }
            break;

        case COIN_2:
        	if (!initial_display_done)
			  {
				  Display_1dhiram();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  current_state = COIN2_OPERATION;
			  }
            break;

        case COIN_3:
//            Display_2dhiram();
//            TM1637_Countdown_20Sec();
//            if (countdown_seconds == 0)
//            {
//                current_state = COIN3_OPERATION;
//                last_update_time = current_time_ms;
//            }
            break;

        case COIN2_OPERATION:
        	 if (HAL_GetTick() - task_start_time < 20000) // Check if 20 seconds haven't elapsed
			  {
				  TM1637_Countdown_20Sec(); // Update the countdown
			  }
			  else
			  {
				  printf("GPIO 3 5 6 enabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = COIN2_OP2;
			  }
            break;

        case COIN2_OP2:
        	if (HAL_GetTick() - task_start_time >= 30000) // Wait for 30 seconds
			  {
				  printf("GPIO 3 5 6 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = COIN2_OP3;
			  }

        case COIN2_OP3:
        	printf("GPIO A & B enabled\n\r");
			  HAL_GPIO_WritePin(SIGNAL_A_GPIO_Port, SIGNAL_A_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(SIGNAL_B_GPIO_Port, SIGNAL_B_Pin, GPIO_PIN_SET);
			  HAL_Delay(5000);
			  printf("GPIO A & B disabled\n\r");
			  HAL_GPIO_WritePin(SIGNAL_A_GPIO_Port, SIGNAL_A_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SIGNAL_B_GPIO_Port, SIGNAL_B_Pin, GPIO_PIN_RESET);
			  task_start_time = HAL_GetTick(); // Record the start time for the next state
			  state = DISPLAY_OFF;
			  break;

        case DISPLAY_OFF:
        	TM1637_DisplayClear();
			  if (HAL_GetTick() - task_start_time >= 90000) // Wait for 2 minutes
			  {
				  printf("return to IDLE\n\r");
				  current_state = IDLE; // Return to the initial state
			  }
			  break;

        default:
            current_state = IDLE;
            break;
    }
}

void TM1637_Countdown_20Sec(void)
{
	static bool colon_state = false;
	static int countdown_seconds = 20;
	static uint32_t last_update_time = 0;
	uint8_t display_data[4] = {0x00, 0x00, 0x00, 0x00};
    if ((current_time_ms - last_update_time) >= 1000)
    {
        last_update_time = current_time_ms;
        colon_state = !colon_state;

        display_data[0] = digit_map[0];
		display_data[1] = digit_map[0];
        display_data[2] = digit_map[(countdown_seconds / 10)];
        display_data[3] = digit_map[(countdown_seconds % 10)];

        if (colon_state)
        {
			display_data[1] |= 0x80;
		} else
		{
			display_data[1] |= 0x00;
		}
        TM1637_WriteData(0xC0, display_data, 4);
        printf("Countdown: %02d seconds\n", countdown_seconds);
        countdown_seconds--;
        if (countdown_seconds < 0) {
            countdown_seconds = 20;
        }
    }
}

//void ProcessCoin2Operation(void)
//{
//    printf("Processing COIN_2 operation\n");
//    HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
//
//    if ((current_time_ms - last_update_time) >= 30000)
//    {
//        HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
//        HAL_Delay(5000);
//        current_state = DISPLAY_OFF;
//    }
//}
//
//void ProcessCoin3Operation(void)
//{
//    printf("Processing COIN_3 operation\n");
//    HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
//
//    if ((current_time_ms - last_update_time) >= 30000)
//    {
//        HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
//        HAL_Delay(5000);
//        current_state = DISPLAY_OFF;
//    }
//}



void Display_fifty(void)
{
	uint8_t data[4] = {0x00, digit_map[5], digit_map[0], digit_map[10]};
	TM1637_WriteData(0xC0, data, 4);
}

void Display_1dhiram(void)
{
	uint8_t data[4] = {0x00, 0x00, digit_map[1], digit_map[11]};
	TM1637_WriteData(0xC0, data, 4);
}

void Display_2dhiram(void)
{
	uint8_t data[4] = {0x00, 0x00, digit_map[2], digit_map[11]};
	TM1637_WriteData(0xC0, data, 4);
}

void DisplayDashes(void)
{
    uint8_t data[4] = {digit_map[12],digit_map[12], digit_map[12], digit_map[12]};
    TM1637_WriteData(0xC0, data, 4);
    //printf("Display Dashes\n\r");
    HAL_GPIO_WritePin(SIGNAL_4_GPIO_Port, SIGNAL_4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_SET);
}

void TM1637_DisplayClear(void)
{
	uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
	TM1637_WriteData(0xC0, data, 4);
}
