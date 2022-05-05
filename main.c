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
#include <motor_configuration.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include <camera/po8030.h>

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
			115200,
			0,
			0,
			0,
	};

	sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}
//chprintf((BaseSequentialStream *)&SD3, "distance = %d \n",distance);

<<<<<<< HEAD
=======
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
>>>>>>> main

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();

	serial_start(); //communication with pc

<<<<<<< HEAD
	motors_init(); //inits the motors
	VL53L0X_start(); //starts time of flight
    dcmi_start();   //starts the camera
	while (1){
		//set_speed();
		color_detection();
		chThdSleepMilliseconds(500);
	}
=======
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
>>>>>>> main
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}




