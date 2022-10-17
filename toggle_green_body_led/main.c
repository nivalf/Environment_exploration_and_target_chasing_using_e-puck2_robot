#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "leds.h"
#include "spi_comm.h"


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    // Initiate and turn off all eight ring LEDs
    clear_leds();
    spi_comm_start();

    /* Infinite loop. */
    while (1) {
    	/* Toggle green led
    	 	 LED2 is an RGB LED. (So are LED4, LED6 & LED8)
    	 	 0: Off
    	 	 1: On
    	 	 2: Toggle
    	 */
    	set_rgb_led(LED2, 8, 0, 0);

    	//waits 1 second
        chThdSleepMilliseconds(500);
    	set_rgb_led(LED2, 0, 0, 0);

    	//waits 1 second
        chThdSleepMilliseconds(500);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
