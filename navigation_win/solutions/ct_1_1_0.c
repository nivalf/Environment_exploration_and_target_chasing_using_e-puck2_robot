/* Chase Target v1.1.0
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
 *
 * Fix:
 * 1. Issue of bot moving tangent to the target
 * - For scan, use  (s0 > range && s7 > range) which will orient the bot
 * better than using OR condition.
 *
 * Known Problems:
 * 1. No exit from move forward state
 * Need to identify if target reached or if target moved & switch to
 * corresponding states
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
void turn_right(void);
void should_stop_turn(void);

/******* Global Variables ********/

/* Bot State:
 * 0: scan mode
 * 1: move forward mode */
int bot_state = 0;

// Speed
const int TURN_SPEED = 200;
const int MOVE_SPEED = 800;

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

// Move the bot forward;
void move_forward(void) {
	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
}

/**************** Turn ******************/

// turn the bot clockwise
void turn_right(void) {
	right_motor_set_speed(-1 * TURN_SPEED);
	left_motor_set_speed(TURN_SPEED);
}

/* If target detected, set bot_state = 1
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

/************** THE END ;D ******************/
