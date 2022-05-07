#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/VL53L0X/VL53L0X.h"

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>
#include <arm_math.h>

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
uint8_t get_mode(uint16_t distance )
{
	uint16_t freq=get_freq();
	if (freq == 16 && distance>200)
	{return 1;}
	if (freq == 16 && distance<200)
	{return 0;}
	if (freq==19)
	{return 2;}
	else{ return 0;}
}
static THD_WORKING_AREA(waPiRegulator,256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    int8_t mode = 0;
    uint16_t distance=0;
    float angle = 0;
    while(1)
    {
		time = chVTGetSystemTime();
		angle =  get_angle();
		distance = VL53L0X_get_dist_mm();
	    mode = get_mode(distance);
	    switch(mode)
	    {
	    	case 0:
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				break;

	    	case 1:
				speed = 500;

				speed_correction = - pi_regulator_angle(angle, GOAL_ANGLE);

				if(abs(speed_correction) < ROTATION_THRESHOLD){

					speed_correction = 0;
				//applies the speed from the PI regulator and the correction for the rotation
				right_motor_set_speed(speed  + ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed  - ROTATION_COEFF * speed_correction);
				}
				break;

	    	case 2:
				right_motor_set_speed(-1100);
				left_motor_set_speed(1100);
				break;
	    }
        chThdSleepUntilWindowed(time, time + MS2ST(80));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



