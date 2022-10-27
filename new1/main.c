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
int get_target_prox_sensor_number(void);
void move_forward(void);
int obstacle_detected(void);
void turn_right(void);
void turn_left(void);
void turn(void);
void sensor_main(void);

struct prox
{
    int value;
    int sensor_no;
};

int sensor_select_count = 0;
int bot_state = 0; 	// 0: moving fwd, 1: turning


const int turn_speed = 400;
const int move_speed = 1000;
int target_sensor;
float x;
float y;

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

              // **************** Read Proximity values ****************** //

        // get values from the 8 IR sensors. (0-7)
        int prox_readings[8];
        // convert this to a separate fn. Use array pointer to pass on the value.
        // Test in playground before implementing.
        for(int sensor = 0; sensor < 8; sensor ++ ) {
        	prox_readings[sensor] = get_prox(sensor);
        	char str[100]; // resulting string of sprintf will be stored here
			char split = (sensor == 7) ? '\n' : '|';
			int str_length = sprintf(str, "S %d: %d, %c",sensor, prox_readings[sensor], split);
			e_send_uart1_char(str, str_length);
        }

        // *********************** ***************************//


        switch(bot_state) {
        	// moving forward
			case 0:
				move_forward();
				if(obstacle_detected()) {
					sensor_main();
					bot_state = 1;
				}
				break;

			case 1:
				turn();
				break;
        }
        char str[100]; // resulting string of sprintf will be stored here
		int str_length = sprintf(str, "bot_state : %d \n",bot_state);
		e_send_uart1_char(str, str_length);

    }

}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

/*************** Helper Functions *************************/
void sensor_main(void){
	target_sensor= get_target_prox_sensor_number();
	x=get_prox(target_sensor)+(0.3*get_prox(target_sensor));
	y=get_prox(target_sensor)-(0.3*get_prox(target_sensor));
}
void turn(void){

	if (get_prox(0)<= x && get_prox(0)>=y || get_prox(7)<= x   && get_prox(7)>=y) {
		bot_state =0;
	}
//	else if (target_sensor<=4){
//		turn_right();
//	}
	else {
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

// Move the bot forward;
void move_forward(void) {
	right_motor_set_speed(move_speed);
	left_motor_set_speed(move_speed);
}

// 1 if obstacle ahead else 0
int obstacle_detected(void) {
	const int thr = 500;	// threshold value
	int obs_detected = 0;

	for(int sensor = 0; sensor < 8; sensor++) {
		if(get_prox(sensor) > thr) {
			obs_detected = 1;
		}
	}
	char str[100]; // resulting string of sprintf will be stored here
	int str_length = sprintf(str, "obs_dect : %d \n",obs_detected);
	e_send_uart1_char(str, str_length);

	return obs_detected;
}

/* Get the proximity sensor number of next direction */
int get_target_prox_sensor_number(void) {
	struct prox prox_values[8];
	//int prox prox_values[8];

	for (int sensor = 0; sensor < 8; sensor++)
	    {
	    	prox_values[sensor].value = get_prox(sensor);
	    	prox_values[sensor].sensor_no = sensor;
	    }

	//sort
	for (int i=0; i<8; i++)
		for (int j=i+1; j<8; j++)
		{
			if (prox_values[i].value < prox_values[j].value) {
				int tmp_v = prox_values[i].value;
				prox_values[i].value = prox_values[j].value;
				prox_values[j].value = tmp_v;
				int tmp_n = prox_values[i].sensor_no;
				prox_values[i].sensor_no = prox_values[j].sensor_no;
				prox_values[j].sensor_no = tmp_n;
			}
		}
	char str[100]; // resulting string of sprintf will be stored here
	int str_length = sprintf(str, "target sensor: %d \n",target_sensor);
	e_send_uart1_char(str, str_length);
	return prox_values[sensor_select_count].sensor_no;
}
