/* Chase Target v1.5.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Gulia
 *
 * Date: 29 Oct 2022
 *
 * Algorithm:
 * > Spin & scan for the obstacle (sensor 0 & 7)
 * > If target Detected, move forward
 * > If tracking lost, scan again and start moving
 * > Else, continue moving & stop on reaching the target
 * > if target moved out of proximity, move forward, if tracking lost, scan mode
 *
 * Fix:
 * 1. exit from standby state
 * => Switch to scan mode if target not in range (tracking lost)
 *
 * Known Problems:
 * 1. Need a move back state.
 * - If target moves forward to the bot while bot is at standby mode
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

// header files for UART
#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"

#include "sensors/proximity.h"
#include "motors.h"

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
void move_forward(void);
int target_detected(void);
int target_in_proximity(void);
void turn_right(void);
void should_stop_turn(void);
void should_stop_move_forward(void);
int tracking_lost(void);
void should_stop_standby(void);
int target_moved_out_of_proximity(void);

/******* Global Variables ********/

/* Bot State:
 * 0: scan mode
 * 1: move forward mode
 * 2: standy mode */
int bot_state = 0;

// Speed
const int TURN_SPEED = 200;
const int MOVE_SPEED = 800;

// Target Proximity threshold value : IR reading above this val => don't get closer
const int TARGET_PROX_THR = 500;
// Range to detect target
const int RANGE_THR = 40;

/******* ***** ********** ********/

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    motors_init();

	// Initiate inter-process communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Start & Calibrate the proximity sensor
    proximity_start();
    calibrate_ir();

    // initialize UART1 channel
    serial_start();

    /* Infinite loop. */
    while (1) {
    	// delay in milliseconds. 10Hz
        chThdSleepMilliseconds(100);

		// DEV feedback
		char str[100];
		int str_length = sprintf(str, "bot_state: %d \n", bot_state);
		e_send_uart1_char(str, str_length);

        switch(bot_state) {
			// Scan mode
			case 0:
				turn_right();
				should_stop_turn();
				break;

        	// moving forward mode
    		case 1:
    			move_forward();
    			should_stop_move_forward();
    			break;

    		// standby mode
    		case 2:
    			should_stop_standby();
    			break;
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

/*************** Helper Functions *************************/

/**************** Turn ******************/

// turn the bot clockwise
void turn_right(void) {
	right_motor_set_speed(-1 * TURN_SPEED);
	left_motor_set_speed(TURN_SPEED);
}

/* Switch bot state from scanning (turning) to:
 * - move forward (1) if target detected
 * i.e stop turning & start moving forward */
void should_stop_turn(void) {
	if(target_detected()) {
		bot_state = 1;

		// DEV feedback
		char str[100];
		int str_length = sprintf(str, "Target Detected. Switching to move forward mode. \n");
		e_send_uart1_char(str, str_length);
	}
}

/**************** Move Forward ******************/

// Move the bot forward;
void move_forward(void) {
	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
}

/* Switch bot state from move forward to:
 * - to standby mode(2) if target reached
 * - to scan mode(0) if tracking lost */
void should_stop_move_forward(void) {
	if(target_in_proximity()) {
		bot_state = 2;

		// DEV feedback
		char str[100]; // resulting string of sprintf will be stored here
		int str_length = sprintf(str, "Target Reached. Switching to standby mode. \n");
		e_send_uart1_char(str, str_length);
	}
	else if(tracking_lost()) {
		bot_state = 0;

		// DEV feedback
		char str[100];
		int str_length = sprintf(str, "Tracking Lost. Switching to scan mode. \n");
		e_send_uart1_char(str, str_length);
	}
}

/******************** Stand by ********************/

/* Switch bot state from stand by:
 * - to move forward state if target moved out of proximity
 * - to scan mode if tracking lost
 */
void should_stop_standby(void) {
	if(target_moved_out_of_proximity()) {
		if(tracking_lost()) {
			bot_state = 0;

			// DEV feedback
			char str[100];
			int str_length = sprintf(str, "Target moved out of proximity & Tracking Lost. Switching to scan mode. \n");
			e_send_uart1_char(str, str_length);

		} else {
			bot_state = 1;

			// DEV feedback
			char str[100];
			int str_length = sprintf(str, "Target moved out of proximity. Switching to move forward mode. \n");
			e_send_uart1_char(str, str_length);
		}
	}
}

/*********** Target Detection *********************/

/* Check if the front sensors have any object in the range
 *
 * return:
 * 	1 : target found
 *  0 : target not found
 */
int target_detected(void) {
	// Using the two front sensors. s0 & s7.
	// target found if both s0 & s7 registers object.
	int target_found = (get_prox(0)>RANGE_THR) && (get_prox(7)>RANGE_THR);

	// DEV feedback
	char str[100];
	int str_length = sprintf(str, " s0: %d (%d) | s7: %d (%d)", get_prox(0), get_calibrated_prox(0), get_prox(7), get_calibrated_prox(7));
	e_send_uart1_char(str, str_length);

	return target_found;
}

/* Check if bot is in the proximity of the target
 *
 * return:
 * 	1 : target in proximity
 *  0 : target not in proximity
 */
int target_in_proximity(void) {
	// Using the two front sensors. s0 & s7.
	// target_in_proximity if either of the sensors register target at proximity
	int in_proximity = (get_prox(0)>TARGET_PROX_THR) || (get_prox(7)>TARGET_PROX_THR);

	// DEV feedback
	char str[100];
	int str_length = sprintf(str, " s0: %d (%d) | s7: %d (%d)", get_prox(0), get_calibrated_prox(0), get_prox(7), get_calibrated_prox(7));
	e_send_uart1_char(str, str_length);

	return in_proximity;
}

/* Check if bot lost track of the target
 *
 * return:
 * 1: tracking lost 		=> target not detected ahead
 * 0: tracking not lost		=> target detected ahead
 */
int tracking_lost(void) {
	return !target_detected();
}

/* Check if target moved out of proximity
 * Use 90% of TARGET_PROX_THR value for comparison -> play (to prevent vibration)
 *
 * return:
 * 	1 : target moved out of proximity
 *  0 : target didn't move out of proximity
 */
int target_moved_out_of_proximity(void) {
	// Using the two front sensors. s0 & s7.
	// target moved out of proximity if neither of the
	// sensors register target at at-least 90% proximity threshold
	const int thr = 0.9 * TARGET_PROX_THR;
	int out_of_proximity = (get_prox(0)<thr) || (get_prox(7)<thr);

	// DEV feedback
	char str[100];
	int str_length = sprintf(str, " s0: %d (%d) | s7: %d (%d)", get_prox(0), get_calibrated_prox(0), get_prox(7), get_calibrated_prox(7));
	e_send_uart1_char(str, str_length);

	return out_of_proximity;
}

/************** THE END ;D ******************/
