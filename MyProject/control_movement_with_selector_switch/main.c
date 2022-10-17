#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "selector.h"
#include "motors.h"
#include "leds.h"
#include "spi_comm.h"


void toggle_bot_spin(int toggle);
void blink_green_leds(void);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    // initialize
    motors_init();
    clear_leds();
    spi_comm_start();

    int counter = 0;
    int toggle = -1;

    /* Infinite loop. */
    while (1) {
    	const int selector = get_selector();

    	/* Change the direction of spin whenever counter resets*/
    	if(counter == 0) {
    		toggle_bot_spin(toggle);
    		toggle = toggle * (-1);
    	}

    	/* Increment the counter & reset when count==selector */
    	counter = (counter + 1) % selector;

    	//waits 0.5 second
        chThdSleepMilliseconds(500);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

/* Function to toggle the spin of the robot.
 * Takes an argument 'toggle' to change direction of spin
*/
void toggle_bot_spin(int toggle) {
	const int speed = 300;

	left_motor_set_speed(toggle * speed);
	right_motor_set_speed(toggle * -1 * speed);
}

/*Function to blink the green body LEDs*/
void blink_green_leds(void) {
	set_rgb_led('LED2',0,2,0);
	set_rgb_led('LED4',0,2,0);
	set_rgb_led('LED6',0,2,0);
	set_rgb_led('LED8',0,2,0);
}
