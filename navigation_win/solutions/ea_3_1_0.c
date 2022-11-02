

/* Explore Arena v3.1.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Singh Gulia
 *
 * Date: 1 Nov 2022
 *
 *Algorithm
	* Position the bot parallel to one side (Manual now. Code in later versions)

	* Move the bot  - Long step
		If s0||s7>THR OR s1||s6>1.5THR 		: Obstacle ahead
			switch to turn mode
	* Turn the bot 90 degree (turn_count: 0,1 cw, 2,3 ccw)
		increment turn count (%4)
		switch to short step
	* Move the bot - Short step
		If s0||s7>THR OR s1||s6>1.5THR 		: Obstacle ahead
			increment incomplete_step_count
			switch to turn mode
		else
			reset incomplete_step_count
			switch to turn mode after completing step
	* Turn the bot 90 degree (turn_count: 0,1 cw, 2,3 ccw)
		increment turn count (%4)
		if incomplete_step_count > 1
			increment turn_count by 2 (to change direction of short steps)
		switch to long step
 *
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
#include "leds.h"
#include "spi_comm.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"


// header files for gyro
#include "behaviors.h"
#include "sensors/imu.h"
#include "epuck1x/utility/utility.h"

/********* MACRO DEFINITIONS ********/
#define TURN_SPEED 500
#define MOVE_SPEED 800
#define OBS_THR 300
#define SHORT_STEP 100

/************************************/

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
void move_forward(void);
int obstacle_ahead(void);
void turn(void);
void increment_turn_counter(void);
int get_turn_direction(void);
int turn_complete(void);
int exit_short_step(void);
void confirm_short_step_direction(void);
void start_count_down(void);

/******* Global Variables ********/

/* Bot State:
 * 0: long_step
 * 1: short_step_turn
 * 2: short_step
 * 3: long_step_turn
 * 4: long_step 	: same as 0, but initiated by condition
 */
int bot_state = 0;

/* Turn Counter => Decides next turn direction
 * 0,1 : cw
 * 2,3 : ccw;
 */
int turn_counter = 0;

/* Counter for consecutive incomplete steps */
int incomplete_step_count = 0;

float turn_angle_rad = 0.0;

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

    // Accelerometer & gyro
	calibrate_acc();
	calibrate_gyro();

    // initialize UART1 channel
    serial_start();

    // initialize LEDs
    void clear_leds(void);
    void spi_comm_start(void);

    // Count down to start program
    start_count_down();

    /* Infinite loop. */
    while (1) {
    	// delay in milliseconds. 20Hz
        chThdSleepMilliseconds(50);

		// DEV feedback
	//	char str[100];
	//	int str_length = sprintf(str, "bot_state: %d \n", bot_state);
	//	e_send_uart1_char(str, str_length);

        switch(bot_state) {
        	case 0:
        		// long step
        		move_forward();
        		bot_state = 1;
        		break;
        	case 1:
        		// short step turn
        		if(obstacle_ahead()){
        			turn();
        			turn_angle_rad = 0.0;
        			bot_state = 2;
        		}
        		break;
        	case 2:
        		// short step
        		if(turn_complete()){
					left_motor_set_pos(0);
        			move_forward();
        			increment_turn_counter();
        			bot_state = 3;
        		}
        		break;
        	case 3:
        		// long step turn
        		if(exit_short_step()){
        			turn();
        			turn_angle_rad = 0.0;
        			bot_state = 4;
        		}
        		break;
        	case 4:		// same as case 0, but initiated by condition
        		// long step
        		if(turn_complete()){
        			move_forward();
        			increment_turn_counter();
        			confirm_short_step_direction();
        			bot_state = 1;
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

// Move the bot forward
void move_forward(void) {
	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
}

/* Exit short step mode if:
 * 1. Obstacle ahead. This is incomplete step => Increment counter
 * 2. Step complete. => Reset counter
 */
int exit_short_step(void){
	if(obstacle_ahead()){							// incomplete step
		incomplete_step_count++;
		return 1;
	} else if(left_motor_get_pos() >= SHORT_STEP){	// complete step
		incomplete_step_count = 0;
		return 1;
	}
	return 0;
}

/* Reverse short step direction if incomplete step count > 1
 * => If short steps turn incomplete twice in a row means bot is
 * at an edge. Thus turn counter is incremented twice to reverse
 * the short step direction.
 */
void confirm_short_step_direction(void) {
	if(incomplete_step_count > 1) {
		// increment twice
		increment_turn_counter();
		increment_turn_counter();
		// reset
		incomplete_step_count = 0;
	}
}

/**************** Turn ******************/

/* Turn the bot
 *
 * turn direction = 1: turn clockwise
 * turn direction = -1: turn counter clockwise
 */
void turn(void) {
	int turn_direction = get_turn_direction();

	right_motor_set_speed((-1) * turn_direction * TURN_SPEED);
	left_motor_set_speed(turn_direction * TURN_SPEED);

	// reset timer
	getDiffTimeMsAndReset();
}

/* Get the turn direction
 * Turn counter value:
 * 		0,1 : cw	-> 1
 * 		2,3 : ccw	-> -1
 */
int get_turn_direction(void) {
	return turn_counter == 0 || turn_counter == 1 ? 1 : -1;
}

/* Increment turn counter with limit 4. (0, 1, 2, 3) */
void increment_turn_counter(void) {
	turn_counter = (turn_counter + 1) % 4;
}

/* Check if 90 degree turn is complete
 * Check if turn_angle_rad >= 90 degree
 * */
int turn_complete(void) {
	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

	turn_angle_rad += get_gyro_rate(2)*getDiffTimeMsAndReset()*0.001;
	return abs(turn_angle_rad) >= M_PI/2;
}

/*********** Obstacle Detection *********************/

/* Fn to detect obstacle ahead.
 * Check for s0 & s7 with OBS_THR, s1 & s6 with 1.5*OBS_THR
 *  s0||s7>THR OR s1||s6>1.5THR
 *
 *  return:
 *  1 : obstacle ahead
 *  0 : no obstacle ahead
 */
int obstacle_ahead(void) {
	const int thr = OBS_THR;
	const int thr2 = 1.5 * OBS_THR;

	// DEV: Uncomment this if prox is not giving values since gyro is using messagebus
//    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//    proximity_msg_t prox_values;
//	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

	int obstacle_detected = get_prox(0)>thr || get_prox(7)>thr || get_prox(1)>thr2 || get_prox(6)>thr2;

		// DEV feedback
	//	char str[100];
	//	int str_length = sprintf(str, " s6: %d, s7: %d, s0: %d, s1: %d", get_prox(6), get_prox(7), get_prox(0), get_prox(1));
	//	e_send_uart1_char(str, str_length);

	return obstacle_detected;
}

/********************************************************/

void start_count_down(void){
	flow_led();
    chThdSleepMilliseconds(3000);
    snake_led();
    clear_leds();
    set_front_led(1);
}

/************** THE END ;D ******************/
