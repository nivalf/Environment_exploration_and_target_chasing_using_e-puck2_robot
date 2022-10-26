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

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function Declarations
float get_turn_angle(void);
int cmp(const void *a, const void *b);
int get_prox_sensor_number(void);
void move_forward(void);
int has_obstacle_ahead(void);
void turn_right(void);
void turn(void);

struct prox
{
    int value;
    int sensor_no;
};

int sensor_select_count = 0;
int bot_state = 0; 	// 0: moving fwd, 1: turning

float target_turn_angle = 0;
float current_turn_angle_rad = 0;

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
    	// waits 1 second
        chThdSleepMilliseconds(1000);

        // **************** Read Proximity values ****************** //

//        // get values from the 8 IR sensors. (0-7)
//        int prox_readings[8];
//        int calibrated_prox_readings[8];
//        int ambient_light[8];
//
//        // convert this to a separate fn. Use array pointer to pass on the value.
//        // Test in playground before implementing.
//        for(int sensor = 0; sensor < 8; sensor ++ ) {
//        	prox_readings[sensor] = get_prox(sensor);
//        	calibrated_prox_readings[sensor] = get_calibrated_prox(sensor);
//        	ambient_light[sensor] = get_ambient_light(sensor);
//        }
//
//        // **************** Stream Proximity values to the terminal ****************** //
//
//        // Print the IR values to terminal
//        for(int sensor = 0; sensor < 8; sensor ++ ) {
//        	char str[100]; // resulting string of sprintf will be stored here
//        	char split = (sensor == 7) ? '\n' : '|';
//        	int str_length = sprintf(str, " %d, %d, %d %c", prox_readings[sensor], calibrated_prox_readings[sensor], ambient_light[sensor], split);
//        	e_send_uart1_char(str, str_length);
//        }

        // *********************** ***************************//


        switch(bot_state) {
        	// moving forward
			case 0:
				move_forward();
				if(has_obstacle_ahead()) {
					current_turn_angle_rad = 0;
					target_turn_angle = get_turn_angle();
					bot_state = 1;
				}
				break;

			case 1:
				turn();
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

void turn(void) {
//	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
	current_turn_angle_rad += -1 * get_gyro_rate(2)*getDiffTimeMsAndReset()*0.001;

	if(current_turn_angle_rad >= target_turn_angle) {
		// reset
		target_turn_angle = 0;
		current_turn_angle_rad = 0;
		// set to move forward
		bot_state = 0;
	} else {
		turn_right();
	}
}

// turn the bot clockwise
void turn_right(void) {
	const int speed = 200;
	right_motor_set_speed(-1 * speed);
	left_motor_set_speed(speed);
}

// Move the bot forward;
void move_forward(void) {
	const int speed = 1000;

	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
}

// 1 if obstacle ahead else 0
int has_obstacle_ahead(void) {
	const int prox_thr = 350;
	return get_calibrated_prox(0) < prox_thr || get_calibrated_prox(7) < prox_thr ? 1 : 0;
}

/* Get the angle in radians to turn for next movement*/
float get_turn_angle(void) {
	int chosen_prox_sensor = get_prox_sensor_number();
	float turn_angle = ((M_PI/4) * chosen_prox_sensor) - M_PI/8;
	return turn_angle;
}

/* Get the proximity sensor number of next direction */
int get_prox_sensor_number(void) {
	struct prox calibrated_prox_values[8];
	    for (int sensor = 0; sensor < 8; sensor++)
	    {
	    	calibrated_prox_values[sensor].value = get_calibrated_prox(sensor);
	    	calibrated_prox_values[sensor].sensor_no = sensor;
	    }

	//sort
	qsort(calibrated_prox_values, 8, sizeof(calibrated_prox_values[0]), cmp);

	// jump sequence
	sensor_select_count = (sensor_select_count + 3) % 8;

	return calibrated_prox_values[sensor_select_count].sensor_no;
}

// Compare function for sort
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
