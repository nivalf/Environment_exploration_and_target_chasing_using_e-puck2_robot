#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

// header for proximity sensor
#include "sensors/proximity.h"

// header files for UART
#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"

#include "motors.h"

#include "behaviors.h"
#include "sensors/imu.h"
#include "epuck1x/utility/utility.h"
#include "selector.h"

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
void set_target_turn_angle(void);
int cmp(const void *a, const void *b);
int get_target_prox_sensor_number(void);
void move_forward(void);
int obstacle_in_proximity(void);
void turn_right(void);
void turn_left(void);
void turn(void);
void send_feedback_data(void);
int get_turn_direction(void);
int target_in_range(void);
void change_sensor_select_count(void);
void explore_arena(void);
void chase_target(void);

struct prox
{
    int value;
    int sensor_no;
};

int sensor_select_count = 0;
int bot_state = 0; 	// 0: moving fwd, 1: turning

/* Angle for turning
 *  +ve : turn right
 *  -ve : turn left
 */
float target_turn_angle = 0.0;
float current_turn_angle = 0.0;

// Speed
const int turn_speed = 400;
const int move_speed = 1000;

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

	// Calibrate the sensors in use
    calibrate_ir();
	calibrate_acc();
	calibrate_gyro();

    // initialize UART1 channel
    serial_start();

    // Enable obstacle avoidance. Threshold: proximity < 300
//	enable_obstacle_avoidance();


    /* Infinite loop. */
    while (1) {
    	// waits
        chThdSleepMilliseconds(100);

        // Send feedback data to the serial monitor
        send_feedback_data();

        switch(get_selector()) {
        	case 0:
        		// EXPLORATION MODE
        		explore_arena();
                break;
			default:
				// TARGET CHASE MODE
				chase_target();
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

/*************** MAIN FUNCTIONS ***************************/

// TASK 1: Explore the arena while avoiding collision
void explore_arena(void) {
    switch(bot_state) {
    	// moving forward mode
		case 0:
			move_forward();
			if(obstacle_in_proximity()) {
				set_target_turn_angle();
				change_sensor_select_count();
				bot_state = 1;
			}
			break;

		// Turn mode
		case 1:
			turn();
			break;
    }
}

// TASK 2: Chase a target
void chase_target(void) {
    switch(bot_state) {
    	// moving forward mode
		case 0:
//        				move_forward();
			if(target_in_range()) {
				sensor_select_count = 0; // to select the closest one. Change to proper location
				set_target_turn_angle();
				bot_state = 1;
			}
			break;

		// Turn mode
		case 1:
			turn();
			break;
    }
}

/*************** Helper Functions *************************/

/*
 * DEV
 *
 * - Add a reset function when selector state changes
 * -
 */

// Move the bot forward;
void move_forward(void) {
	right_motor_set_speed(move_speed);
	left_motor_set_speed(move_speed);
}

/**************** Turn ******************/

void turn(void) {
	int turn_direction = get_turn_direction();

//	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
	current_turn_angle += turn_direction * get_gyro_rate(2) * getDiffTimeMsAndReset() * 0.001;

	if(abs(current_turn_angle) >= abs(target_turn_angle)) {
		// reset
		target_turn_angle = 0.0;
		current_turn_angle = 0.0;
		// set to move forward
		bot_state = 0;
	} else if (turn_direction == 1) {
		turn_right();
	} else if (turn_direction == -1) {
		turn_left();
	}
}

// turn the bot clockwise
void turn_right(void) {
	right_motor_set_speed(-1 * turn_speed);
	left_motor_set_speed(turn_speed);
}

// turn the bot counter clockwise
void turn_left(void) {
	right_motor_set_speed(turn_speed);
	left_motor_set_speed(-1 * turn_speed);
}

/* Set the target angle in radians to turn for next movement
		 Position of the robot sensors:
				forward

		-M_PI/8			  7	  0 (15 deg) 		M_PI/8
		-M_PI/4			6		1 (45 deg) 		M_PI/4
		-M_PI/2		  5	    	  2 (90 deg) 	M_PI/2
		-5*M_PI/4		 4	   3 (150 deg) 		5*M_PI/4
*/
void set_target_turn_angle(void) {
	int sensor_angle_arr[8] = { M_PI/8, M_PI/4, M_PI/2, 5*M_PI/4, (-1)*M_PI/8, (-1)*M_PI/4, (-1)*M_PI/2, (-5)*M_PI/4 };
	int target_sensor = get_target_prox_sensor_number();
	target_turn_angle = sensor_angle_arr[target_sensor];
}

/* Get the proximity sensor number of next direction */
int get_target_prox_sensor_number(void) {
	struct prox prox_values[8];
	    for (int sensor = 0; sensor < 8; sensor++)
	    {
	    	prox_values[sensor].value = get_prox(sensor);
	    	prox_values[sensor].sensor_no = sensor;
	    }

	//sort
	qsort(prox_values, 8, sizeof(prox_values[0]), cmp);


	return prox_values[sensor_select_count].sensor_no;
}


// jump sequence
void change_sensor_select_count(void) {
	sensor_select_count = (sensor_select_count + 3) % 8;
}

/* Compare function for sort
 * Sorts in descending order of sensor values => increasing order of distance
 */
int cmp(const void *a, const void *b) {
    struct prox *a1 = (struct prox *)a;
    struct prox *a2 = (struct prox *)b;
    if ((*a1).value > (*a2).value)
        return -1;
    else if ((*a1).value < (*a2).value)
        return 1;
    else
        return 0;
}

/* Direction of turn.
 * 		1 : clockwise (right)
 * 		-1 : counterclockwise (left)
 */
int get_turn_direction() {
	return target_turn_angle < 0 ? -1 : 1;
}

/*********** Obstacle Detection *********************/

/*	1 : obstacle in proximity
 *  0 : no obstacle in proximity
 */
int obstacle_in_proximity(void) {
	const int thr = 500;	// threshold value
	int obstacle_detected = 0;

	for(int sensor = 0; sensor < 8; sensor++) {
		if(get_prox(sensor) > thr) {
			obstacle_detected = 1;
		}
	}
	return obstacle_detected;
}

/******************* TASK 2 ADDITIONAL HELPERS ******************/

/*	1 : target in range
 *  0 : target not in range
 */
int target_in_range(void) {
	const int thr = 500;	// threshold value
	int target_detected = 0;

	for(int sensor = 0; sensor < 8; sensor++) {
		if(get_prox(sensor) < thr) {
			target_detected = 1;
		}
	}
	return target_detected;
}

/*************** Feedback **************************/

// Send data to the terminal
void send_feedback_data(void) {

	// Prox sensor values:
	for(int sensor = 0; sensor < 8; sensor ++ ) {
		int prox_value = get_prox(sensor);
		char str[100]; // resulting string of sprintf will be stored here
		char split = (sensor == 7) ? '\n' : '|';
		int str_length = sprintf(str, " s%d: %d %c", sensor, prox_value, split);
		e_send_uart1_char(str, str_length);
	}

	// Gyro values:
	char str[100]; // resulting string of sprintf will be stored here
	int str_length = sprintf(str, "current angle: %f, target angle: %f \n", current_turn_angle, target_turn_angle);
	e_send_uart1_char(str, str_length);

}
