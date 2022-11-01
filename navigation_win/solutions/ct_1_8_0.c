/* Chase Target v1.8.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Gulia
 *
 * Date: 1 Nov 2022
 *
 * Algorithm:
* Scan
	if s0 && s7 > range_thr			: Target detected
		move forward
* Move forward
	if s0 || s7 > prox_thr			: Target reached
		go to standby
	if s0 || s7 < range_thr			: Tracking lost
		start scan
* Standby
	if s0 && s7 < outer_prox_thr	: Target moved out
		start scan
	if s0 || s7 > inner_prox_thr	: Target too close
		move back
* Move back
	if s0 && s7 < prox_thr			: Safe distance
		go to standby
 *
 * Fix:
 * 1. Bi directional turn
 *
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

/********* CONSTANTS ***********/

// Speed
#define TURN_SPEED 400
#define MOVE_SPEED 600

// Target Proximity threshold value : IR reading above this val => don't get closer
#define TARGET_PROX_THR 400
#define INNER_PROX_THR (1.5 * TARGET_PROX_THR)
#define OUTER_PROX_THR (0.7 * TARGET_PROX_THR)

// Range to detect target
#define RANGE_THR 70

/**********************************/

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
void move_forward(void);
void move_back(void);
int target_detected(void);
int target_in_proximity(void);
void turn(void);
void should_stop_turn(void);
void should_stop_move_forward(void);
void should_stop_move_back(void);
int tracking_lost(void);
void should_stop_standby(void);
int target_moved_out_of_proximity(void);
void standby(void);
int target_too_close(void);
void set_turn_direction(void);
void print_state_error(void);

/******* Global Variables ********/

/* Bot State:
 * 0: scan mode
 * 1: move forward mode
 * 2: standy mode
 * 3: move back mode*/
int bot_state = 0;

int scan_state = 0;

int turn_direction = -1;

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
    	// delay in milliseconds. 20Hz
        chThdSleepMilliseconds(50);

		// DEV feedback
//		char str[100];
//		int str_length = sprintf(str, "bot_state: %d \n", bot_state);
//		e_send_uart1_char(str, str_length);

        switch(bot_state) {
			// Scan mode
			case 0:
				switch(scan_state) {
					case 0:
						// identify direction
						set_turn_direction();
						scan_state = 1;
						break;
					case 1:
						// turn
						turn();
						should_stop_turn();
						break;
					default:
						// not supposed to enter here
						print_state_error();
				}
				break;

        	// moving forward mode
    		case 1:
    			move_forward();
    			should_stop_move_forward();
    			break;

    		// standby mode
    		case 2:
    			standby();
    			should_stop_standby();
    			break;

			// moving back mode
			case 3:
				move_back();
				should_stop_move_back();
				break;
			default:
				// not supposed to enter here
				print_state_error();
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

/* If any object detected on right side, set 1. else -1
 *
 * 1: clockwise
 * -1: counter clockwise
 */
void set_turn_direction(void) {
	int direction = -1;
	for(int i=0; i<4; i++) {
		if(get_prox(i) > RANGE_THR) {
			direction = 1;
		}
	}

	turn_direction = direction;
}

/* Turn the bot
 *
 * turn direction = 1: turn clockwise
 * turn direction = -1: turn counter clockwise
 */
void turn(void) {
	right_motor_set_speed((-1) * (turn_direction) * TURN_SPEED);
	left_motor_set_speed((turn_direction) * TURN_SPEED);
}

/* Switch bot state from scanning (turning) to:
 * - move forward (1) if target detected
 * i.e stop turning & start moving forward
 *
 * + Reset scan_state */
void should_stop_turn(void) {
	if(target_detected()) {
		bot_state = 1;
		scan_state = 0;	// reset scan state;

		// DEV feedback
//		char str[100];
//		int str_length = sprintf(str, "Target Detected. Switching to move forward mode. \n");
//		e_send_uart1_char(str, str_length);
	}
}

/**************** Move Forward/Backward ******************/

// Move the bot forward;
void move_forward(void) {
	// Add PID
	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
}

// Move the bot back;
void move_back(void) {
	right_motor_set_speed(-1 * MOVE_SPEED);
	left_motor_set_speed(-1 * MOVE_SPEED);
}

/* Switch bot state from move forward to:
 * - to standby mode(2) if target reached
 * - to scan mode(0) if tracking lost */
void should_stop_move_forward(void) {
	if(target_in_proximity()) {
		bot_state = 2;

		// DEV feedback
//		char str[100]; // resulting string of sprintf will be stored here
//		int str_length = sprintf(str, "Target Reached. Switching to standby mode. \n");
//		e_send_uart1_char(str, str_length);
	}
	else if(tracking_lost()) {
		bot_state = 0;

		// DEV feedback
//		char str[100];
//		int str_length = sprintf(str, "Tracking Lost. Switching to scan mode. \n");
//		e_send_uart1_char(str, str_length);
	}
}

/* Switch bot state from move back to:
 * - to standby mode(2) if target back in proximity */
void should_stop_move_back(void) {
	if(target_in_proximity()) {
		bot_state = 2;

		// DEV feedback
//		char str[100];
//		int str_length = sprintf(str, "Target back in safe proximity. Switching to standby mode. \n");
//		e_send_uart1_char(str, str_length);
	}
}

/******************** Stand by ********************/

// Stand by: Stop the movement
void standby(void) {
	// Add PID
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

/* Switch bot state from stand by:
 * - to scan mode if target moved out of proximity
 */
void should_stop_standby(void) {
	if(target_moved_out_of_proximity()) {
		bot_state = 0;

		// DEV feedback
//			char str[100];
//			int str_length = sprintf(str, "Target moved out of proximity. Switching to scan mode. \n");
//			e_send_uart1_char(str, str_length);
	} else if (target_too_close()) {
		bot_state = 3;
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
//	char str[100];
//	int str_length = sprintf(str, " s0: %d (%d) | s7: %d (%d)", get_prox(0), get_calibrated_prox(0), get_prox(7), get_calibrated_prox(7));
//	e_send_uart1_char(str, str_length);

	return target_found;
}

/* Check if bot is in the proximity of the target
 *
 * while moving forward:
 * 		prox values increase & crosses TARGET_PROX_THR (>)
 * while moving backward:
 * 		prox values decrease & crosses TARGET_PROX_THR (<)
 *
 * return:
 * 	1 : target in proximity
 *  0 : target not in proximity
 */
int target_in_proximity(void) {
	// Using the two front sensors. s0 & s7.
	int in_proximity;
	const int thr = TARGET_PROX_THR;	// target proximity threshold

	if(bot_state == 1) {			// moving forward
		in_proximity = (get_prox(0)>thr) || (get_prox(7)>thr);
	} else {					// moving backward
		// ensure both sensors register target out of threshold
		in_proximity = (get_prox(0)<thr) && (get_prox(7)<thr);
	}

	// DEV feedback
//	char str[100];
//	int str_length = sprintf(str, " s0: %d (%d) | s7: %d (%d)", get_prox(0), get_calibrated_prox(0), get_prox(7), get_calibrated_prox(7));
//	e_send_uart1_char(str, str_length);

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
 * Use OUTER_PROX_THR for comparison -> buffer (to prevent vibration)
 *
 * return:
 * 	1 : target moved out of proximity
 *  0 : target didn't move out of proximity
 */
int target_moved_out_of_proximity(void) {
	// Using the two front sensors. s0 & s7.
	// target moved out of proximity if neither of the
	// sensors register target inside outer proximity threshold
	const int thr = OUTER_PROX_THR;
	int out_of_proximity = (get_prox(0)<thr) && (get_prox(7)<thr);

	// DEV feedback
//	char str[100];
//	int str_length = sprintf(str, " s0: %d (%d) | s7: %d (%d)", get_prox(0), get_calibrated_prox(0), get_prox(7), get_calibrated_prox(7));
//	e_send_uart1_char(str, str_length);

	return out_of_proximity;
}

/* Using the two front sensors. s0 & s7.
* target moved too close to the bot if either of the
* sensors register target at max INNER_PROX_THR
*/
int target_too_close(void) {
	const int thr = INNER_PROX_THR;
	int too_close = (get_prox(0)>thr) || (get_prox(7)>thr);

	return too_close;
}

/************** Error *********************/
void print_state_error(void) {
	// DEV feedback
	char str[200];
	int str_length = sprintf(str, "ERROR: State not identified. Bot state: %d, Scan state: %d \n",bot_state, scan_state);
	e_send_uart1_char(str, str_length);
}

/************** THE END ;D ******************/
