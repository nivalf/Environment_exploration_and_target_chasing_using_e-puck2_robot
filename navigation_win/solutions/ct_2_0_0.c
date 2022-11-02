/* Chase Target v2.0.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Gulia
 *
 * Date: 2 Nov 2022
 *
 * Algorithm:
* Scan
	if s0 && s7 > range_thr			: Target detected
* If target detected
	move (PID)
  else
	switch to scan mode
 *
 * Updates:
 * 1. Integrate distance sensor
 * 2. PID for back/front movement
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
#include "sensors/VL53L0X/VL53L0X.h"

#include "epuck1x/utility/utility.h"

/********* CONSTANTS ***********/

// Speed
#define TURN_SPEED 400

#define TARGET_DISTANCE 20	// 20mm : 2cm

// Range to detect target
#define RANGE_THR 70

/**********************************/

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
void move(void);
int target_detected(void);
void turn(void);
int get_turn_direction(void);
int get_distance_to_target(void);
int get_move_speed(void);

/******* Global Variables ********/

/* Bot State:
 * 0: scan mode
 * 1: move forward mode
 * 2: standy mode
 * 3: move back mode*/
int bot_state = 0;

int scan_state = 0;

int turn_direction = -1;

// PID Values
int kp = 1.5;
int kd = 1.5;
int ki = 2;

int integral = 0; // initialize
int last_error = 0;

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

    // Initiate distance sensor
    VL53L0X_start();


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
				turn();
				bot_state = 1;
				break;

        	// moving mode
    		case 1:
    			if(target_detected()) {
    				resetTime();
    				move();
    			} else {
    				bot_state = 0;
    				// reset
    				integral = 0;
    				last_error = 0;
    			}
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

// Get distance to the target in mm
int get_distance_to_target(void) {
	return VL53L0X_get_dist_mm();
}

/**************** Turn ******************/

/* If any object detected on right side, set 1. else -1
 *
 * 1: clockwise
 * -1: counter clockwise
 */
int get_turn_direction(void) {
	int turn_direction = -1;
	for(int i=0; i<4; i++) {
		if(get_prox(i) > RANGE_THR) {
			turn_direction = 1;
		}
	}

	return turn_direction;
}

/* Turn the bot
 *
 * turn direction = 1: turn clockwise
 * turn direction = -1: turn counter clockwise
 */
void turn(void) {
	int turn_direction = get_turn_direction();

	right_motor_set_speed((-1) * (turn_direction) * TURN_SPEED);
	left_motor_set_speed((turn_direction) * TURN_SPEED);
}

/**************** Move Forward/Backward ******************/

/* Get move speed using PID logic */
int get_move_speed(void){
	// get current distance to target
	int current_distance = get_distance_to_target();

	// get dt
	int dt = getDiffTimeMsAndReset()*0.001;
	// ignore the first value of dt
	dt = last_error == 0? 1 : dt;

	// calculate the error
	int error = -1 * (TARGET_DISTANCE - current_distance);

	// calculate the integral;
	integral = integral + (error*dt);

	// calculate derivative
	int derivative = (error - last_error)/dt;

	// control variable
	int move_speed = (kp * error) + (ki * integral) + (kd * derivative);

	// Clamp the control variable [-1000,1000]
	move_speed = move_speed > 1000 ? 1000 : move_speed < -1000 ? -1000 : move_speed;

	// save for next iteration
	last_error = error;
	return move_speed;
}

/* Move forward/backward - controlled by PID */
void move(void) {
	const int MOVE_SPEED = get_move_speed();

	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
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

/************** THE END ;D ******************/
