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
void set_target_turn_angle(void);
int cmp(const void *a, const void *b);
int get_target_prox_sensor_number(void);
void move_forward(void);
int has_obstacle_ahead(void);
void turn_right(void);
void turn_left(void);
void turn(void);
void send_feedback_data(void);

struct prox
{
    int value;
    int sensor_no;
};

int sensor_select_count = 0;
int bot_state = 0; 	// 0: moving fwd, 1: turning

float target_turn_angle = 0.0;
float current_turn_angle = 0.0;
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

        switch(bot_state) {
        	// moving forward
			case 0:
				move_forward();
				if(has_obstacle_ahead()) {
					set_target_turn_angle();
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

void turn_gyro(void) {
//	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
	current_turn_angle += -1 * get_gyro_rate(2)*getDiffTimeMsAndReset()*0.001;

	if(current_turn_angle >= target_turn_angle) {
		// reset
		target_turn_angle = 0.0;
		current_turn_angle = 0.0;
		// set to move forward
		bot_state = 0;
	} else {
		turn_right();
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

// Move the bot forward;
void move_forward(void) {
	right_motor_set_speed(move_speed);
	left_motor_set_speed(move_speed);
}

// 1 if obstacle ahead else 0
int has_obstacle_ahead(void) {
	const int thr = 1000;	// threshold value
	return get_prox(0) > thr || get_prox(1) > thr || get_prox(2) > thr || get_prox(3) > thr || get_prox(4) > thr || get_prox(5) > thr || get_prox(6) > thr || get_prox(7) > thr ? 1 : 0;
}

/* Set the target angle in radians to turn for next movement*/
void set_target_turn_angle(void) {
	int target_prox_sensor = get_target_prox_sensor_number();
	target_turn_angle = ((M_PI/4) * (target_prox_sensor + 1)) - M_PI/8;
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

	// jump sequence
	sensor_select_count = (sensor_select_count + 3) % 8;

	return prox_values[sensor_select_count].sensor_no;
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
	int str_length = sprintf(str, "current angle: %f, target angel: %f \n", current_turn_angle, target_turn_angle);
	e_send_uart1_char(str, str_length);

}
