#include "ch.h"
#include "hal.h"
#include <math.h>

#include "sensors/VL53L0X/VL53L0X.h"

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <behaviour.h>

#define MODE_STOP   	0
#define MODE_FLOWER    	1
#define MODE_BUTINE     2
#define MODE_BEE    	3
#define MODE_NO_FLOWER  4
#define MODE_LEADER	   	5

#define FORWARD         1
#define BACKWARD        -1
#define ON				1
#define OFF				0

#define SPEED 			500
#define SPEED_TURN 		1000

#define DISTANCE_GOAL	100
#define DISTANCE_GOAL_L 80
#define DISTANCE_GOAL_H 120

#define NB_ROTATION 	4
#define NSTEP 			250
#define NSTEP_FORWARD 	2500
#define HALF_TURN 		650
#define ONE_TURN 		2*HALF_TURN

#define FREQ_FLOWER		16	//250Hz
#define FREQ_ACTION		19  //296HZ
#define FREQ_FOLLOW		23  //359HZ
#define FREQ_LEADER		26  //406HZ

#define FREQ_FLOWER_L		(FREQ_FLOWER-1)
#define FREQ_FLOWER_H		(FREQ_FLOWER+1)
#define FREQ_ACTION_L		(FREQ_ACTION-1)
#define FREQ_ACTION_H		(FREQ_ACTION+1)
#define FREQ_FOLLOW_L		(FREQ_FOLLOW-1)
#define FREQ_FOLLOW_H		(FREQ_FOLLOW+1)
#define FREQ_LEADER_L		(FREQ_LEADER-1)
#define FREQ_LEADER_H		(FREQ_LEADER+1)

static int16_t orientation;
static uint8_t mode;


//==================================================================================================================
//PRIVATE FUNCTIONS
//==================================================================================================================

//simple PI regulator implementation
int16_t pi_regulator_angle(float angle, float goal){

	float error = 0;
	float speed_correction = 0;
	static float sum_error = 0;

	error = angle - goal;

	sum_error += error;

	//disables the PI regulator if the error is to small
	if(fabs(error)< ERROR_THRESHOLD){
		return 0;
	}

	 //we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;

	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed_correction = KP * error + KI * sum_error;

    return (int16_t)speed_correction;
}

//determine the mode of the robot.
void update_mode(uint16_t freq_index, uint16_t distance ){

	// if flower frequency then the robot moves towards it.
	if(freq_index >= FREQ_FLOWER_L && freq_index <= FREQ_FLOWER_H){
		if((distance>= DISTANCE_GOAL_L && distance <= DISTANCE_GOAL_H)){
			mode = MODE_STOP;
		}
		else if(distance >= DISTANCE_GOAL){
			orientation = FORWARD;
			mode = MODE_FLOWER;
		}
		else if(distance <= DISTANCE_GOAL) {
			orientation = BACKWARD;
			mode = MODE_FLOWER;
		}
	}

	// if bee frequency then follow the bee.
	else if(freq_index >= FREQ_FOLLOW_L && freq_index <= FREQ_FOLLOW_H){
		if((distance>= DISTANCE_GOAL_L && distance <= DISTANCE_GOAL_H)){
			mode = MODE_STOP;
		}
		else if(distance >= DISTANCE_GOAL){
			orientation = FORWARD;
			mode = MODE_BEE;
		}
		else if(distance <= DISTANCE_GOAL) {
			orientation = BACKWARD;
			mode = MODE_BEE;
		}
	}

	// if freq leader then check if the bee follows the robot.
	else if(freq_index >= FREQ_LEADER_L && freq_index <= FREQ_LEADER_H){
		if((distance>= DISTANCE_GOAL_L && distance <= DISTANCE_GOAL_H)){
			mode = MODE_LEADER;
		}
		else {
			mode = MODE_STOP;
		}
	}

	// if freq action then butine or not depending on the presence of a flower.
	else if(freq_index >= FREQ_ACTION_L && freq_index <= FREQ_ACTION_H){
		if((distance>= DISTANCE_GOAL_L && distance <= DISTANCE_GOAL_H)){
			mode = MODE_BUTINE;
		}
		else {
			mode = MODE_NO_FLOWER;
		}
	}

	//if other frequency then stop.
	else {
		mode = MODE_STOP;
	}
}

static THD_WORKING_AREA(waBehaviour,256);
static THD_FUNCTION(Behaviour, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t freq_index =0;
    int16_t speed_correction = 0;
    uint16_t distance=0;
    float angle = 0;

    orientation = 0;
    mode = 0;

    while(1)
    {
		time = chVTGetSystemTime();

		//computes a correction factor to let the robot rotate to be in front of the sound source.
		angle =  get_angle();
		speed_correction = - pi_regulator_angle(angle, GOAL_ANGLE);

		//if the speed correction is neglectable stop rotation.
		if(abs(speed_correction) < ROTATION_THRESHOLD){
			speed_correction = 0;
		}

		//get the distance and the index frequency
		distance = VL53L0X_get_dist_mm();
		freq_index = get_freq_index();

		//determine the mode.
	    update_mode(freq_index, distance);

	    switch(mode)
	    {
	    	case MODE_STOP:
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				break;

	    	case MODE_FLOWER:
	    		right_motor_set_speed(orientation*SPEED  + ROTATION_COEFF * speed_correction);
	    		left_motor_set_speed(orientation*SPEED   - ROTATION_COEFF * speed_correction);
				break;

	    	case MODE_BUTINE:
				right_motor_set_speed(-SPEED_TURN);
				left_motor_set_speed(SPEED_TURN);
				chThdSleepMilliseconds(ONE_TURN);
				break;

	    	case MODE_NO_FLOWER:
	    		orientation = FORWARD;
				for (int compteur = 0 ; compteur < NB_ROTATION ; compteur++){
					set_led(compteur, ON);
					right_motor_set_speed(orientation*SPEED_TURN);
					left_motor_set_speed(-orientation*SPEED_TURN);
					chThdSleepMilliseconds(abs(NSTEP));
					orientation = - orientation;
					set_led(compteur, OFF);
				}
				break;

	    	case MODE_BEE :
	    		right_motor_set_speed(orientation*SPEED  + ROTATION_COEFF * speed_correction);
	    		left_motor_set_speed(orientation*SPEED   - ROTATION_COEFF * speed_correction);
	    		break;

	    	case MODE_LEADER:

				right_motor_set_speed(SPEED_TURN);
				left_motor_set_speed(-SPEED_TURN);
				chThdSleepMilliseconds(HALF_TURN);

				right_motor_set_speed(SPEED_TURN);
				left_motor_set_speed(SPEED_TURN);
				chThdSleepMilliseconds(NSTEP_FORWARD);

				right_motor_set_speed(SPEED_TURN);
				left_motor_set_speed(-SPEED_TURN);
				chThdSleepMilliseconds(HALF_TURN);
				break;
	    }
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//==================================================================================================================
//PUBLIC FUNCTIONS
//==================================================================================================================


void behaviour_start(void){
	chThdCreateStatic(waBehaviour, sizeof(waBehaviour), NORMALPRIO, Behaviour, NULL);
}



