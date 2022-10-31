/* Explore Arena v2.0.0
 *
 * Authors:
 * Flavin Lee John
 * Ankur Singh Gulia
 *
 * Date: 31 Oct 2022
 *
 * Algorithm:
 * > Move forward
 * > If obstacle in proximity, add repulsion force
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
int obstacle_in_proximity(void);

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

    	move_forward();
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
	int left_speed = 0.8 * MOTOR_SPEED_LIMIT;
	int right_speed = 0.8 * MOTOR_SPEED_LIMIT;

	if(obstacle_in_proximity()) {

		int prox_values[8];

		for(int i=0; i<8; i++) {
			prox_values[i] = get_prox(i);

			// DEV feedback
			char str[100];
			char split = (i == 6) ? '\n' : '|';
			int str_length = sprintf(str, " s%d: %d (%d) %c", i, get_prox(i), get_calibrated_prox(i), split);
			e_send_uart1_char(str, str_length);
		}

		left_speed = MOTOR_SPEED_LIMIT/2 - prox_values[0]*8 - prox_values[1]*4 - prox_values[2]*2;
		right_speed = MOTOR_SPEED_LIMIT/2 - prox_values[7]*8 - prox_values[6]*4 - prox_values[5]*2;

	}

	right_motor_set_speed(left_speed);
	left_motor_set_speed(right_speed);

	// DEV feedback
	char str[100];
	int str_length = sprintf(str, "Speed set. left_speed: %d, right_speed: %d \n", left_speed, right_speed);
	e_send_uart1_char(str, str_length);
}


/*********** Obstacle Detection *********************/

/*	1 : obstacle in proximity
 *  0 : no obstacle in proximity
 */
int obstacle_in_proximity(void) {
	const int THR = 300;	// threshold value
	int obstacle_is_in_proximity = 0;

	// The 6 front sensors
	int sensors[6] = {0, 1, 2, 5, 6, 7};

	for(int i=0; i<6; i++) {
		if(get_prox(sensors[i]) > THR) {
			obstacle_is_in_proximity = 1;
		}

		// DEV feedback
		char str[100];
		char split = (i == 6) ? '\n' : '|';
		int str_length = sprintf(str, " s%d: %d (%d) %c", sensors[i], get_prox(i), get_calibrated_prox(i), split);
		e_send_uart1_char(str, str_length);
	}

	return obstacle_is_in_proximity;
}
