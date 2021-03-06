#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <motors.h>
#include <audio/microphone.h>
#include "sensors/VL53L0X/VL53L0X.h" // est ce que c'est des " ou <

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <behaviour.h>
#include <leds.h>

int main(void)
{
    halInit();
    chSysInit();

    //inits the leds, the motors and the audio anglesVL
    clear_leds();
    motors_init();
    audio_init();

    //starts the microphones processing thread.
    mic_start(&processAudioData);

    //starts the threads for the pi regulator and the TOF
    behaviour_start();
    VL53L0X_start();

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
