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

int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = 80 * error + 0.35 * sum_error;

    return (int16_t)speed;
}

uint8_t get_mode(uint16_t distance )
{

	uint16_t freq=get_freq();
	chprintf((BaseSequentialStream *)&SD3, "freq  = %d\n", freq);
	if (freq == 16 && distance>200)
	{
		return 1;}
	if (freq == 16 && distance<200)
	{return 0;}
	if (freq==19)
	{return 2;}

	if(freq== 23){
		return 3;
	}
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

		distance = VL53L0X_get_dist_mm();
	    mode = get_mode(distance);
	    chprintf((BaseSequentialStream *)&SD3, "mode  = %d\n", mode);
	    switch(mode)
	    {
	    	case 0:

				right_motor_set_speed(0);
				left_motor_set_speed(0);
				break;

	    	case 1:
				speed = 500;
				angle =  get_angle();
				speed_correction = - pi_regulator_angle(angle, GOAL_ANGLE);
				chprintf((BaseSequentialStream *)&SD3, "speed_correction  = %d\n", speed_correction);
				if(abs(speed_correction) < ROTATION_THRESHOLD){
					speed_correction = 0;
				}
				//applies the speed from the PI regulator and the correction for the rotation
				right_motor_set_speed(speed  + ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed  - ROTATION_COEFF * speed_correction);
				break;

	    	case 2:
				right_motor_set_speed(-1100);
				left_motor_set_speed(1100);
				break;
	    	case 3 :
				angle =  get_angle();
				speed=  pi_regulator(distance, 200);
				speed_correction = - pi_regulator(angle , 0);

				chprintf((BaseSequentialStream *)&SD3, "speed_correction  = %d\n", speed_correction);
				if(abs(speed_correction) < 10){
					speed_correction = 0;
				}
				right_motor_set_speed(speed  +20 * speed_correction);
				left_motor_set_speed(speed   - 20* speed_correction);

	    }
        chThdSleepUntilWindowed(time, time + MS2ST(80));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



