#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <pi_regulator.h>
#include <fft.h>
#include <arm_math.h>

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}
>>>>>>> lea_branch

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //inits the motors
    motors_init();
    audio_init();
    serial_start();
    mic_start(&processAudioData);

    pi_regulator_start();

    /* Infinite loop. */
    while (1) {

    	//waits 1 second
        chThdSleepMilliseconds(1000);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
