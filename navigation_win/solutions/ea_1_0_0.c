/* Explore Arena v1.0.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Singh Gulia
 *
 * Date: 27 Oct 2022
 *
 * Algorithm:
 * > Move forward
 * > If obstacle Detected, turn right
 * > If obstacle clear, move forward
 *
 * Possible Issues:
 *
 * 1. Bot might vibrate
 * The threshold IR value to determine obstacle ahead and path clear is the same.
 * Thus the bot frequently switches b/w the two states causing vibration.
 *
 * Sol: Keep the threshold IR value to stop turning lower than the one to stop moving forward.
 *
 * 2. Incomplete exploration
 * Bot only turns to right which will result in incomplete exploration
 *
 * Sol: Identify which side is more open by comparing sensor values and move there.
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

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
void move_forward(void);
int obstacle_ahead(void);
void turn_right(void);
void should_stop_move_forward(void);
void should_stop_turn(void);

/* Bot State:
 * 0: move forward mode
 * 1: turn mode */
int bot_state = 0;

// Speed
const int turn_speed = 200;
const int move_speed = 800;

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
        	// moving forward mode
    		case 0:
    			move_forward();
    			should_stop_move_forward();
    			break;

    		// Turn mode
    		case 1:
    			turn_right();
    			should_stop_turn();
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
	right_motor_set_speed(move_speed);
	left_motor_set_speed(move_speed);
}

/* Switch bot state to turning mode if obstacle ahead*/
void should_stop_move_forward(void) {
	if(obstacle_ahead()) {
		bot_state = 1;

		// DEV feedback
		char str[100]; // resulting string of sprintf will be stored here
		int str_length = sprintf(str, "Obstacle detected. Switching to turn mode. \n");
		e_send_uart1_char(str, str_length);
	}
}

/**************** Turn ******************/

// turn the bot clockwise
void turn_right(void) {
	right_motor_set_speed(-1 * turn_speed);
	left_motor_set_speed(turn_speed);
}

/* Switch bot state to move forward mode if no obstacle ahead */
void should_stop_turn(void) {
	if(!obstacle_ahead()) {
		bot_state = 0;

		// DEV feedback
		char str[100];
		int str_length = sprintf(str, "Path clear. Switching to move forward mode. \n");
		e_send_uart1_char(str, str_length);
	}
}

/*********** Obstacle Detection *********************/

/*	1 : obstacle ahead
 *  0 : no obstacle ahead
 */
int obstacle_ahead(void) {
	const int THR = 500;	// threshold value
	int obstacle_detected = 0;

	// The 4 front sensors
	int sensors[4] = {0, 1, 6, 7};

	for(int i=0; i<4; i++) {
		if(get_prox(sensors[i]) > THR) {
			obstacle_detected = 1;
		}

		// DEV feedback
		char str[100];
		char split = (i == 4) ? '\n' : '|';
		int str_length = sprintf(str, " s%d: %d (%d) %c", sensors[i], get_prox(i), get_calibrated_prox(i), split);
		e_send_uart1_char(str, str_length);
	}

	return obstacle_detected;
}
