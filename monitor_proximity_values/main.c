/* STREAM PROXIMITY SENSOR VALUES TO TERMINAL
 * This program is to read values from all 8 PROXIMITY SENSORS
 * sensors and stream the data to the computer terminal. */
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

#include "sensors/imu.h"
#include "epuck1x/utility/utility.h"

// Define inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
	// Initiate inter-process communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    mpu_init();

    // Start the proximity sensor
    proximity_start();
    // Calibrate the proximity sensor
    calibrate_ir();

    // initialize UART1 channel
    serial_start();

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(100);

        // get values from the 8 IR sensors. (0-7)
        int prox_readings[8];
        int calibrated_prox_readings[8];
        int ambient_light[8];

        //	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
        	int current_turn_angle_rad = 0;
        	current_turn_angle_rad += -1 * get_gyro_rate(2)*getDiffTimeMsAndReset()*0.001;

        // convert this to a separate fn. Use array pointer to pass on the value.
        // Test in playground before implementing.
        for(int sensor = 0; sensor < 8; sensor ++ ) {
        	prox_readings[sensor] = get_prox(sensor);
        	calibrated_prox_readings[sensor] = get_calibrated_prox(sensor);
        	ambient_light[sensor] = get_ambient_light(sensor);
        }

        char str[100]; // resulting string of sprintf will be stored here
    	int str_length = sprintf(str, " %d,  \n", current_turn_angle_rad );
    	e_send_uart1_char(str, str_length);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
