#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>
#include <arm_math.h>
//SALUT

//simple PI regulator implementation
int16_t pi_regulator_angle(float angle, float goal){

	 float error = 0;
	 float speed = 0;
	 static float sum_error = 0;

	 error = angle - goal;

	 sum_error += error;

	 if(fabs(error)< ERROR_THRESHOLD){
		 return 0;
	 }
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;

	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;

	}
	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    float angle = 0;

    while(1){

        time = chVTGetSystemTime();

        angle =  get_angle();

        speed_correction =  pi_regulator_angle(angle, GOAL_ANGLE);

        if(abs(speed_correction) < ROTATION_THRESHOLD){

        	speed_correction = 0;
        }

        chprintf((BaseSequentialStream *)&SD3, "speed_correction = %d \n",speed_correction);
        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed  + ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed  - ROTATION_COEFF * speed_correction);

        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



