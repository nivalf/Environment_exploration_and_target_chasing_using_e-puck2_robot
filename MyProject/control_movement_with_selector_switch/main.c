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


void spin_bot(int direction);
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

    /* direction: 1-clockwise, -1-counter-clockwise */
    int direction = 1;
    /* Counter for direction change*/
    int counter = 0;

    /* Start spinning the bot */
    spin_bot(direction);

    /* Infinite loop. */
    while (1) {
    	const int selector = get_selector();

    	/* Change the direction of spin whenever counter resets
    	 * If selector is 0, make no change in spin */
    	if(counter == 0 && selector != 0) {
    		spin_bot(direction);
    		direction = direction * (-1);
    	}

    	/* Blink the green LEDs */
    	blink_green_leds();

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

/* Function to direction the spin of the robot.
 * Takes an argument 'direction' to change direction of spin
*/
void spin_bot(int direction) {
	const int speed = 300;

	left_motor_set_speed(direction * speed);
	right_motor_set_speed(direction * -1 * speed);
}

/*Function to blink the green body LEDs*/
void blink_green_leds(void) {
	set_rgb_led('LED2',0,2,0);
	set_rgb_led('LED4',0,2,0);
	set_rgb_led('LED6',0,2,0);
	set_rgb_led('LED8',0,2,0);
}
