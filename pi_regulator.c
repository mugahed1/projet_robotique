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

#define MODE_STOP   	0
#define MODE_FLEUR    	1
#define MODE_BUTINE     2
#define MODE_BEE    	3
#define FORWARD         1
#define BACKWARD        -1
#define SPEED 			500
#define SPEED_TURN 		500
#define DISTANCE_GOAL	200
#define DISTANCE_GOAL_L 180
#define DISTANCE_GOAL_H 220

#define FREQ_FLOWER		16	//250Hz
#define FREQ_BEE		23  //359HZ


#define FREQ_FLOWER_L		(FREQ_FLOWER-1)
#define FREQ_FLOWER_H		(FREQ_FLOWER+1)
#define FREQ_BEE_L			(FREQ_BEE-1)
#define FREQ_BEE_H			(FREQ_BEE+1)

static int16_t orientation;
static uint8_t mode;
static bool butine;

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

// fonction permettant la selection du mode en fonction de la frequence
void update_mode(uint16_t freq_index, uint16_t distance ){

	if(freq_index >= FREQ_FLOWER_L && freq_index <= FREQ_FLOWER_H){
		if((distance>= DISTANCE_GOAL_L && distance <= DISTANCE_GOAL_H) || (butine)){
			butine = 1;
			mode = MODE_BUTINE;
		}
		else if(distance >= DISTANCE_GOAL){
			orientation = FORWARD;
			mode = MODE_FLEUR;
		}
		else if(distance <= DISTANCE_GOAL) {
			orientation = BACKWARD;
			mode = MODE_FLEUR;
		}
	}
	else if(freq_index >= FREQ_BEE_L && freq_index <= FREQ_BEE_H){
		butine = 0;
		mode = MODE_BEE;
	}
	else {
		butine = 0;
		mode = MODE_STOP;
	}
}

static THD_WORKING_AREA(waPiRegulator,256);

static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t freq_index;
    int16_t speed_correction = 0;
    uint16_t distance=0;
    float angle = 0;

    orientation = 0;
    while(1)
    {
		time = chVTGetSystemTime();

		angle =  get_angle();
		speed_correction = - pi_regulator_angle(angle, GOAL_ANGLE);
		if(abs(speed_correction) < ROTATION_THRESHOLD){ // revoir la valeur des threshold
			speed_correction = 0;
		}
		distance = VL53L0X_get_dist_mm();
		freq_index = get_freq_index();
	    update_mode(freq_index, distance);

	    switch(mode)
	    {
	    	case 0:
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				break;
	    	case 1:
	    		right_motor_set_speed(orientation*SPEED  + ROTATION_COEFF * speed_correction);
	    		left_motor_set_speed(orientation*SPEED   - ROTATION_COEFF * speed_correction);
				break;
	    	case 2:
				right_motor_set_speed(-SPEED_TURN);
				left_motor_set_speed(SPEED_TURN);
				break;
	    	case 3 :
	    		break;
	    }
        chThdSleepUntilWindowed(time, time + MS2ST(80));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



