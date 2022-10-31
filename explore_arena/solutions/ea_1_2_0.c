/* Explore Arena v1.2.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Singh Gulia
 *
 * Date: 27 Oct 2022
 *
 * Algorithm:
 * > Move forward
 * > If obstacle Detected, turn
 * > If obstacle clear, move forward
 *
 * Enhancements:
 *
 * 1. Turn Both Sides
 * Turn to both sides alternatively
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
int path_is_clear(void);
void turn(void);
void turn_left(void);
void turn_right(void);
void should_stop_move_forward(void);
void should_stop_turn(void);

/******* Global Variables ********/

/* Bot State:
 * 0: move forward mode
 * 1: turn mode */
int bot_state = 0;

// Speed
const int TURN_SPEED = 200;
const int MOVE_SPEED = 800;

// Obstacle threshold value : IR reading above this val => obstacle nearby
const int OBS_THR = 400;

/* Turn Direction
 * 1 : Turn Right
 * 0 : Turn Left;
 */
int turn_direction = 1;
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
        	// moving forward mode
    		case 0:
    			move_forward();
    			should_stop_move_forward();
    			break;

    		// Turn mode
    		case 1:
    			turn();
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
	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
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

/* Turn the bot */
void turn(void) {
	if(turn_direction) {
		turn_right();
	} else {
		turn_left();
	}
}

// turn the bot clockwise
void turn_right(void) {
	right_motor_set_speed(-1 * TURN_SPEED);
	left_motor_set_speed(TURN_SPEED);
}

// turn the bot counter clockwise
void turn_left(void) {
	right_motor_set_speed(TURN_SPEED);
	left_motor_set_speed(-1 * TURN_SPEED);
}

/* If path is clear, set bot_state = 0
 * i.e stop turning & start moving forward */
void should_stop_turn(void) {
	if(path_is_clear()) {
		bot_state = 0;
		turn_direction = !turn_direction;

		// DEV feedback
		char str[100];
		int str_length = sprintf(str, "Path clear. Switching to move forward mode. \n");
		e_send_uart1_char(str, str_length);
	}
}

/*********** Obstacle Detection *********************/

/* return:
 *  1 : obstacle ahead
 *  0 : no obstacle ahead
 */
int obstacle_ahead(void) {
	int obstacle_detected = 0;

	// The 4 front sensors
	int sensors[4] = {0, 1, 6, 7};

	for(int i=0; i<4; i++) {
		if(get_prox(sensors[i]) > OBS_THR) {
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


/* Check if the path ahead is clear. The threshold value is set
 * lower than the threshold value of obstacle_ahead function so
 * that the bot turns further towards the free space. This is to
 * avoid vibration of the bot while travelling along the THR border.
 *
 * return:
 * 	1 : path clear
 *  0 : path not clear
 */
int path_is_clear(void) {
	int path_is_not_clear = 0;

	// The 4 front sensors
	int sensors[4] = {0, 1, 6, 7};


	for(int i=0; i<4; i++) {
		// Compared to 70% of Obstacle Threshold value
		if(get_prox(sensors[i]) > (0.7 * OBS_THR)) {
			path_is_not_clear =  1;
		}

		// DEV feedback
		char str[100];
		char split = (i == 4) ? '\n' : '|';
		int str_length = sprintf(str, " s%d: %d (%d) %c", sensors[i], get_prox(i), get_calibrated_prox(i), split);
		e_send_uart1_char(str, str_length);
	}

	return !path_is_not_clear;
}

/************** THE END ;D ******************/
